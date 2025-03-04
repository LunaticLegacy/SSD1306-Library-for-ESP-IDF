cmake_minimum_required(VERSION 3.16)

# 注册SSD1306驱动组件
idf_component_register(
    SRCS "ssd1306_luna.cpp"  # 这里添加驱动源文件，假设你有一个ssd1306.c文件
    INCLUDE_DIRS "."  # 包含当前目录，以便找到ssd1306.h
    REQUIRES driver  # 如果你需要额外的依赖，如ESP32的 I2C 驱动，可以在这里指定
)
