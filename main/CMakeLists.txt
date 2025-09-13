idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES "driver" "freertos" "esp_system" "esp_wifi" "log"
)