// wifiman_luna.h
#ifndef WIFIMAN_LUNA
#define WIFIMAN_LUNA

#include <vector>
#include <string>
#include <functional>
#include <optional>

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

class WifiMan {
public:
    // 事件回调类型
    using ConnectionCallback = std::function<void(bool connected)>;
    using ScanCompleteCallback = std::function<void(const std::vector<std::string>& ssids)>;

    // 构造函数/析构函数
    explicit WifiMan();
    ~WifiMan();

    // 配置方法
    esp_err_t configure_sta(const std::string& ssid, const std::string& password);
    
    // 连接控制
    esp_err_t connect();
    esp_err_t disconnect();

    // 扫描功能
    esp_err_t start_scan(uint16_t max_aps = 20);
    std::optional<std::vector<std::string>> get_scan_results(int timeout_ms = 5000);

    // 回调注册
    void set_connection_callback(ConnectionCallback cb) { connection_cb_ = cb; }
    void set_scan_callback(ScanCompleteCallback cb) { scan_cb_ = cb; }

private:
    // 内部状态
    enum class State {
        IDLE,
        CONNECTING,
        CONNECTED,
        SCANNING
    };

    // 初始化流程
    esp_err_t init_nvs();
    esp_err_t init_wifi_driver();

    // 事件处理
    static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);
    static void ip_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data);

    // 成员变量
    std::string ssid_;
    std::string password_;
    uint16_t number = 0;
    State state_ = State::IDLE;
    wifi_scan_config_t scan_config_ = {};
    std::vector<wifi_ap_record_t> ap_records_;
    ConnectionCallback connection_cb_;
    ScanCompleteCallback scan_cb_;
    esp_event_handler_instance_t wifi_event_instance_ = nullptr;
    esp_event_handler_instance_t ip_event_instance_ = nullptr;
};

#endif