cmake_minimum_required(VERSION 3.16)

idf_component_register(
    SRCS "st7789_basic.cpp"
    INCLUDE_DIRS "."
    REQUIRES "driver" "freertos" "esp_system" "log"
)