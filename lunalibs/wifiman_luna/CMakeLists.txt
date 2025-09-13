cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "wifiman_luna.cpp"
    INCLUDE_DIRS "."
    REQUIRES "driver" "esp_wifi" "esp_event" "esp_netif" "nvs_flash"
    PRIV_REQUIRES "wpa_supplicant" "lwip" "esp_partition"
)
