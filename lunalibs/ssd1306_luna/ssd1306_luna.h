/**
 * Library for SSD1306 OLED in ESP-IDF Environment
 * (For C++ Only)
 * Author: 月と猫 - LunaNeko
 */


#ifndef SSD1306_LUNA
#define SSD1306_LUNA

#include <driver/i2c_master.h>
#include <vector>


class SSD1306 {
private:
    uint16_t width, height;
    uint16_t num_pages; 
    // 屏幕缓冲区，需要设置一个指针数组，且长度不定。
    uint8_t** screen_buffer;

    uint8_t address = 0x3C;

    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_20,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 32,
        .flags = {
            .enable_internal_pullup = 1,
            .allow_pd = 0
        }
    };

    i2c_master_bus_handle_t bus = {};  // bus handle，暂时不给任何值。

    i2c_device_config_t device_config = {
        .dev_addr_length = static_cast<i2c_addr_bit_len_t>(0),
        .device_address = address,
        .scl_speed_hz = 5000,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 1,
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
        0x20, 0x00, // Set memory addressing mode to horizontal
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
    // flag部分，1为操作码，0为操作数。
    static constexpr uint8_t ssd1306_init_flags[25] = {
        1, // 0xAE - Command
        1, 0, // 0xD5, 0x80
        1, 0, // 0xA8, 0x3F
        1, 0, // 0xD3, 0x00
        1, // 0x40
        1, 0, // 0x8D, 0x14
        1, 0, // 0x20, 0x00
        1, // 0xA1
        1, // 0xC8
        1, 0, // 0xDA, 0x12
        1, 0, // 0x81, 0xCF
        1, 0, // 0xD9, 0xF1
        1, 0, // 0xDB, 0x40
        1, // 0xA4
        1, // 0xA6
        1  // 0xAF
    };

    // 发送数据。
    esp_err_t transfer_byte(const uint8_t& byte);
    esp_err_t send_cmd(const uint8_t& cmd);
    esp_err_t send_data(const uint8_t& data);

    // 屏幕缓冲区相关。
    void init_screen_buffer();

public:

    SSD1306();
    SSD1306(uint16_t width, uint16_t height);

    void init_i2c_bus();

    void init_screen();

    void draw_pixel(const uint16_t& x, const uint16_t& y, bool value);

    void flush();
    void print_buffer();

    ~SSD1306();

};

#endif  // SSD1306_LUNA
