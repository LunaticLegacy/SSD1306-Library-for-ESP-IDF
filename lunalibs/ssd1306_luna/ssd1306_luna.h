/**
 * Library for SSD1306 OLED in ESP-IDF Environment
 * (For C++ Only and ESP-IDF Only)
 * Author: 月と猫 - LunaNeko
 */

#ifndef SSD1306_LUNA
#define SSD1306_LUNA

#include <driver/i2c_master.h>

#include <esp_log.h>

#include <vector>

#include <concepts>
#include <type_traits>
#include <cstdint>

class SSD1306 {
private:

    // flush操作时的缓冲区。
    uint8_t* page_buffer;

    // 显示屏地址
    uint8_t address = 0x3C;
    
    // I2C总线配置，这里是主机。
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0
        }
    };

    i2c_master_bus_handle_t bus = nullptr;  // bus handle，暂时不给任何值。

    i2c_device_config_t device_config = {
        .dev_addr_length = static_cast<i2c_addr_bit_len_t>(0),
        .device_address = address,
        .scl_speed_hz = 400000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        }
    };

    i2c_master_dev_handle_t device = {};

    // 用于初始化SSD1306的指令集，以及下方表达指令和操作数的flag。
    // 指令集将被标志为static，为所有类通用的数据，且不可更改。这些值在编译阶段硬编码入数据。
    static constexpr uint8_t ssd1306_init_cmds[25] = {
        0xAE, // Display OFF
        0xD5, 0x80, // Set display clock divide ratio/oscillator frequency
        0xA8, 0x3F, // Set multiplex ratio
        0xD3, 0x00, // Set display offset
        0x40, // Set start line to 0
        0x8D, 0x14, // Enable charge pump regulator
        0x20, 0x02, // Set memory addressing mode to page addressing mode
        0xA1, // Set segment re-map to normal
        0xC8, // Set COM output scan direction to remapped
        0xDA, 0x12, // Set COM pins hardware configuration
        0x81, 0xCF, // Set contrast control
        0xD9, 0xF1, // Set pre-charge period
        0xDB, 0x40, // Set VCOMH deselect level
        0xA4, // Enable display RAM content
        0xA6, // Set normal display (not inverted)
        0xAF  // Display ON
    };

    // 一个用于封装总初始化内容的东西。
    void total_init();

    // 在同一个I2C会话中，传输1个byte。
    esp_err_t transfer_byte(const uint8_t byte);

    // 在同一个I2C会话中，传输1条或若干条指令。
    esp_err_t transfer_cmd(const uint8_t* cmds, const size_t num_of_cmds);

    // 在同一个I2C会话中，传输单条指令。
    esp_err_t transfer_cmd(const uint8_t& cmd);

    // 在同一个I2C会话中，将1页图像输入到屏幕中并显示。
    esp_err_t transfer_image_for_page(const uint8_t& page, const uint8_t &col_begin, 
        const uint8_t* image, const size_t num_of_cols);


protected:
    // 屏幕缓冲区，需要设置一个指针数组指针。格式：“行-页”，其中二级指针代表页。
    uint8_t** screen_buffer = nullptr;

    uint16_t width, height; // 屏幕尺寸。
    uint16_t num_pages;     // 页数量，每8行1页。每页width列，每列1个uint8_t。

public:

    SSD1306();
    SSD1306(uint16_t width, uint16_t height);

    // 初始化I2C总线。
    void init_i2c_bus();

    // 初始化屏幕。
    void init_screen();

    // 初始化屏幕缓冲区。
    void init_screen_buffer();

    // 清屏。
    void clear_screen_buffer();

    // 画一个像素点，以屏幕左上角为(0, 0)。注意：被画的像素点会被存到缓冲区，而不会立即显示。
    void draw_pixel(const uint16_t& x, const uint16_t& y, bool value);

    // 刷新屏幕缓冲区到显示屏。
    void flush();
    
    // 打印屏幕缓冲区。
    void print_buffer();

    ~SSD1306();

};

// 我需要完成一个二值化图像。
struct BinaryImage {
    uint16_t width, height;
    uint8_t* data;  // 表达数据的一维数组。
};

#include "font_luna.h"  // 字体文件

// 继承SSD1306类，实现一些基本图形的绘制功能。
class SSD1306_SHAPES : public SSD1306 {
private:
    const FontDef* current_font = &Font_6x8;  // 当前字体

    uint16_t cursor_x = 0, cursor_y = 0;

public:

    // 默认构造函数和下方重载。
    SSD1306_SHAPES() : SSD1306() {};
    SSD1306_SHAPES(uint16_t width, uint16_t height) : SSD1306(width, height) {};

    void move_cursor(uint16_t x, uint16_t y) {
        this->cursor_x = x;
        this->cursor_y = y;
    }

    // ---------------------------------------------------------------
    // 图形
    // ---------------------------------------------------------------

    void draw_line(const uint16_t& x0, const uint16_t& y0, 
        const uint16_t& x1, const uint16_t& y1, bool value);

    // 圆形绘制（中点圆算法）
    void draw_circle(const uint16_t& x_center, const uint16_t& y_center, 
            const uint16_t& radius, bool value, bool fill = false);

    // 矩形绘制
    void draw_rectangle(const uint16_t& x0, const uint16_t& y0,
                const uint16_t& width, const uint16_t& height, 
                bool value, bool fill = false);

    // 三角形绘制
    void draw_triangle(const uint16_t& x0, const uint16_t& y0,
                const uint16_t& x1, const uint16_t& y1,
                const uint16_t& x2, const uint16_t& y2,
                bool value, bool fill = false);

    // 菱形绘制
    void draw_diamond(const uint16_t& x_center, const uint16_t& y_center,
            const uint16_t& width, const uint16_t& height,
            bool value, bool fill = false);

    // 绘制自定义点阵图。
    void draw_image(BinaryImage image, uint16_t x, uint16_t y);


    // ---------------------------------------------------------------
    // 字体
    // ---------------------------------------------------------------

    // 设置字体
    void set_font(const FontDef* font) {
        this->current_font = font;
    }
    
    // 获取字符串宽度
    uint16_t get_string_width(const char* str);
    
    // 写单个字符
    void write_char(char c, uint16_t x, uint16_t y, bool value = true);
    
    // 写字符串
    void write_string(const char* str, uint16_t x, uint16_t y, bool value = true);
    
    // 在当前位置写字符串
    void printf(const char* format, ...);

};

#endif  // SSD1306_LUNA
