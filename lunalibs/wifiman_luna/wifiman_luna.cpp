#include "wifiman_luna.h"

#include <cstring>
#include <algorithm>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"

#define WIFI_TAG "WifiMan"

WifiMan::WifiMan() {
    if (init_nvs() != ESP_OK || init_wifi_driver() != ESP_OK) {
        ESP_LOGE(WIFI_TAG, "Initialization failed");
    }
}

WifiMan::~WifiMan() {
    esp_wifi_stop();
    esp_wifi_deinit();
    nvs_flash_deinit();
}

esp_err_t WifiMan::init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    return ret;
}

esp_err_t WifiMan::init_wifi_driver() {
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    // 初始化WiFi驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &WifiMan::wifi_event_handler, this, &wifi_event_instance_));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &WifiMan::ip_event_handler, this, &ip_event_instance_));

    return ESP_OK;
}

esp_err_t WifiMan::configure_sta(const std::string& ssid, const std::string& password) {
    ssid_ = ssid;
    password_ = password;

    wifi_config_t wifi_config = {};
    strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), ssid_.c_str(), sizeof(wifi_config.sta.ssid));
    strncpy(reinterpret_cast<char*>(wifi_config.sta.password), password_.c_str(), sizeof(wifi_config.sta.password));

    return esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
}

esp_err_t WifiMan::connect() {
    if (ssid_.empty()) {
        ESP_LOGE(WIFI_TAG, "SSID not configured");
        return ESP_ERR_INVALID_ARG;
    }

    state_ = State::CONNECTING;
    return esp_wifi_start();
}

esp_err_t WifiMan::start_scan(uint16_t max_aps) {
    scan_config_.show_hidden = false;
    scan_config_.scan_type = WIFI_SCAN_TYPE_ACTIVE;
    scan_config_.scan_time.active.min = 100;
    scan_config_.scan_time.active.max = 300;

    ap_records_.clear();
    ap_records_.resize(max_aps);
    this->number = max_aps;
    scan_config_.bssid = nullptr;

    state_ = State::SCANNING;
    return esp_wifi_scan_start(&scan_config_, false);
}

std::optional<std::vector<std::string>> WifiMan::get_scan_results(int timeout_ms) {
    if (state_ != State::SCANNING) {
        return std::nullopt;
    }

    // 等待扫描完成
    uint16_t found = 0;
    esp_err_t ret = esp_wifi_scan_get_ap_records(&this->number, ap_records_.data());
    if (ret == ESP_OK) {
        std::vector<std::string> ssids;
        for (const auto& ap : ap_records_) {
            if (ap.ssid[0] != '\0') {
                ssids.emplace_back(reinterpret_cast<const char*>(ap.ssid));
                found++;
            }
        }
        state_ = State::IDLE;
        return ssids;
    }
    return std::nullopt;
}

// 事件处理实现
void WifiMan::wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    WifiMan* self = static_cast<WifiMan*>(arg);
    
    switch (id) {
    case WIFI_EVENT_STA_START:
        ESP_LOGI(WIFI_TAG, "STA Started");
        esp_wifi_connect();
        break;
        
    case WIFI_EVENT_STA_CONNECTED:
        self->state_ = State::CONNECTED;
        if (self->connection_cb_) {
            self->connection_cb_(true);
        }
        break;
        
    case WIFI_EVENT_STA_DISCONNECTED:
        self->state_ = State::IDLE;
        if (self->connection_cb_) {
            self->connection_cb_(false);
        }
        break;
        
    case WIFI_EVENT_SCAN_DONE:
        if (self->scan_cb_) {
            auto results = self->get_scan_results();
            if (results) {
                self->scan_cb_(*results);
            }
        }
        break;
    }
}

void WifiMan::ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = static_cast<ip_event_got_ip_t*>(data);
        ESP_LOGI(WIFI_TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}