/**
 * Library for SSD1306 OLED in ESP-IDF Environment
 * (For C++ Only and ESP-IDF Only)
 * Author: 月と猫 - LunaNeko
 */

#ifndef SSD1306_LUNA
#define SSD1306_LUNA

#include <driver/i2c_master.h>

#include <esp_log.h>
#include <esp_task_wdt.h>

#include <concepts>
#include <type_traits>
#include <cstdint>

class SSD1306 {
private:
    TaskHandle_t current_task = nullptr;

    uint16_t width, height; // 屏幕尺寸。
    uint16_t num_pages;     // 页数量，每8行1页。每页width列，每列1个uint8_t。

    // 屏幕缓冲区，需要设置一个指针数组指针。格式：“行-页”，其中二级指针代表页。
    uint8_t** screen_buffer = nullptr;
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
        .scl_speed_hz = 100000,
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

    // 后续内容是否包含控制字节？
    static constexpr uint8_t have_continue[25] = {
        0, 
        1, 0, 
        1, 0, 
        1, 0,
        0,
        1, 0,
        1, 0,
        0,
        0,
        1, 0,
        1, 0,
        1, 0,
        1, 0,
        0, 
        0, 
        0
    };

    void total_init();
    /**
     * @brief 
     * 在同一个I2C会话中，传输1个byte。
     * 
     * @param[in] byte 输入1个byte。
     * @return 返回esp_err_t类型的错误码。
     */
    esp_err_t transfer_byte(const uint8_t byte);


    /**
     * @brief 
     * 在同一个I2C会话中，传输1条或若干条指令。
     * 
     * @param[in] cmds 输入一个被传入的命令数组。
     * @param[in] num_of_cmds 输入指令的条数。
     * @return 返回esp_err_t类型的错误码。
     */
    esp_err_t transfer_cmd(const uint8_t* cmds, const size_t num_of_cmds);

    /**
     * @brief 
     * 在同一个I2C会话中，传输单条指令。
     * 
     * @param[in] cmds 直接输入被传入的命令。
     * @return 返回esp_err_t类型的错误码。
     */
    esp_err_t transfer_cmd(const uint8_t& cmd);

    /**
     * @brief 
     * 在同一个I2C会话中，将1页图像输入到屏幕中并显示。
     * 注意：这个东西只能发送同1页的内容。如果有多页内容，请分页显示。
     * @param[in] page 输入页号，范围仅限[0, 7]之间。
     * @param[in] col_begin 输入起始列号。
     * @param[in] image 输入图像。
     * @param[in] num_of_cols 输入图像的列数。
     * @return 返回esp_err_t类型的错误码。
     */
    esp_err_t transfer_image_for_page(const uint8_t& page, const uint8_t &col_begin, 
        const uint8_t* image, const size_t num_of_cols);

    /**
     * @brief 
     * 初始化屏幕缓冲区。
     */
    void init_screen_buffer();

public:

    SSD1306();
    SSD1306(uint16_t width, uint16_t height);

    /**
     * @brief 
     * 初始化I2C总线。
     * @return void
     */
    void init_i2c_bus();

    /**
     * @brief 
     * 初始化显示屏，发送初始化指令。
     * @return void
     */
    void init_screen();

    /**
     * @brief 
     * 画一个像素点，以屏幕左上角为(0, 0)。
     * 注意：被画的像素点会被缓存到单片机内的缓冲区，而不会立即显示。
     * @param[in] x x坐标
     * @param[in] y y坐标
     * @param[in] value 是否点亮
     * @return void
     */
    void draw_pixel(const uint16_t& x, const uint16_t& y, bool value);

    /**
     * @brief 
     * 刷新屏幕缓冲区到显示屏。
     * @return void
     */
    void flush();
    
    /**
     * @brief 
     * 打印屏幕缓冲区。
     * @return void
     */
    void print_buffer();

    ~SSD1306();

};

class SSD1306_IMGS : public SSD1306 {
    friend class SSD1306;
};

#endif  // SSD1306_LUNA
