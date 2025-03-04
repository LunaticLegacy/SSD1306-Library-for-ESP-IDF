#include "ssd1306_luna.h"
#include <cstdint>

#include <iostream>

// private:
esp_err_t SSD1306::transfer_byte(const uint8_t& byte) {
    // 发送数据，按照byte发送。
    return i2c_master_transmit(this->device, &byte, 1, 0);
};

esp_err_t SSD1306::send_cmd(const uint8_t& cmd) {
    // 发送命令 (Opcode) - 需要加上控制字节 0x80
    uint8_t buffer[2] = {0x80, cmd};  // 0x80 = Command 单字节模式
    return i2c_master_transmit(this->device, buffer, 2, 0);
};

esp_err_t SSD1306::send_data(const uint8_t& data) {
    // 发送数据 (Operand) - 需要加上控制字节 0x40
    uint8_t buffer[2] = {0x40, data};  // 0x40 = Data 单字节模式
    return i2c_master_transmit(this->device, buffer, 2, 0);
};

void SSD1306::init_screen_buffer() {
    this->num_pages = this->height / 8;

    screen_buffer = new uint8_t*[this->width];
    // 然后给每一列储存一个height/8个byte。
    for (uint16_t i = 0; i < this->width; i++) {
        screen_buffer[i] = new uint8_t[this->height / 8]();
    }
    printf(" | Screen buffer initialized. \n");
};

// public:
SSD1306::SSD1306() : width(128), height(64) {
    this->init_i2c_bus();
    this->init_screen();
    this->init_screen_buffer();
};

SSD1306::SSD1306(uint16_t width, uint16_t height) : width(width), height(height) {
    this->init_i2c_bus();
    this->init_screen();
    this->init_screen_buffer();
};

SSD1306::~SSD1306() {
    // 释放资源。
    i2c_master_bus_rm_device(this->device);
    i2c_del_master_bus(this->bus);
};

void SSD1306::init_i2c_bus() {
    bool result;
    result = i2c_new_master_bus(&this->bus_config, &this->bus);
    if (result != ESP_OK) {
        printf(" | Error: Failed to init I2C bus. \n");
    }    
    result = i2c_master_bus_add_device(this->bus, &this->device_config, &this->device);
    if (result != ESP_OK) {
        printf(" | Error: Failed to add device. \n");
    }
    printf(" | I2C hardware initialized. \n");
};

void SSD1306::init_screen() {
    bool result = true;
    // 先发送设备地址。
    uint8_t transmit_data;
    transmit_data = (this->address << 1) + 0x00;  // 发送地址，并设置模式为write。

    result = i2c_master_transmit(this->device, &transmit_data, 1, 0);
    if (result != ESP_OK) {
        printf(" | Error: Failed to config address for I2C. \n");
        return;
    }
    // 等待ACK后，发送数据。transmit指令会处理ACK。
    printf(" | Address initialized. \n");
    printf(" | Initializing screen. \n");
    
    for (int8_t cmd_index = 0; cmd_index < 25; cmd_index++) {
        // 检查操作数和操作码。
        if (SSD1306::ssd1306_init_flags[cmd_index] == 1) {
            result = this->send_cmd(SSD1306::ssd1306_init_cmds[cmd_index]);
        } else {
            result = this->send_data(SSD1306::ssd1306_init_cmds[cmd_index]);
        }

        if (result != ESP_OK) {
            printf(" | Error: Failed to send command %d. \n", cmd_index);
            return;
        }
    }
    printf(" | Screen hardware initialized. \n");
};

void SSD1306::draw_pixel(const uint16_t& x, const uint16_t& y, bool value) {
    // 画图时间到，但我需要一个缓冲区。
    if (x >= this->width || y >= this->height) return;

    uint16_t page = y / 8;  // 计算页
    uint16_t bit_pos = y % 8; // 计算在页中的位置
    
    if (value) {
        screen_buffer[x][page] |= (1 << bit_pos);  // 1
    } else {
        screen_buffer[x][page] &= ~(1 << bit_pos); // 0
    }
    printf(" | Pixel drawn. \n");
};

void SSD1306::flush() {
    // 遍历每页
    for (uint8_t page = 0; page < num_pages; page++) {
        // 设置页地址
        send_cmd(0xB0 + page);  // 发送页地址（页号）
        
        // 设置列地址
        send_cmd(0x00);  // 低位列地址
        send_cmd(0x10);  // 高位列地址

        // 发送数据（逐列）
        for (uint8_t x = 0; x < width; x++) {
            send_data(screen_buffer[x][page]);  // 每列数据发送
        }
    }
}

// 用于打印缓冲区（调试时查看内容）
void SSD1306::print_buffer() {
    printf(" | Buffer: \n");

    for (uint8_t page = 0; page < num_pages; page++) {  // 遍历每一页（8 行）
        for (uint8_t bit_pos = 0; bit_pos < 8; bit_pos++) {  // 遍历页内的 8 行
            for (uint8_t x = 0; x < width; x++) {  // 遍历列
                bool pixel_on = (screen_buffer[x][page] & (1 << bit_pos));  // 逐位读取
                std::cout << (pixel_on ? "#" : ".");  // 1 = "#"，0 = "."
            }
            std::cout << std::endl;  // 换行，代表每 8 位的某一行
        }
    }

    printf(" | Buffer ended. \n");
}
