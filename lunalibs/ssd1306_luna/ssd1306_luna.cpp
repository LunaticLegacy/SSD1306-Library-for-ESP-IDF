#include "ssd1306_luna.h"

#include <cstdint>
#include <iostream>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_task_wdt.h>  // 看门狗管理。

#define feed esp_task_wdt_reset

// private:
esp_err_t SSD1306::transfer_byte(const uint8_t byte) {
    // 从通信的流程上，i2c_master_transmit会先发送设备地址后，再发送所有输入的数据。
    esp_err_t result = i2c_master_transmit(this->device, &byte, 1, -1);
    if (result != ESP_OK) {
        ESP_LOGE("LUNA", "| Byte transmission failed.");
    }
    return result;
};

esp_err_t SSD1306::transfer_cmd(const uint8_t* cmds, const size_t cmd_length) {
    // 我需要拼接命令 - 在每一条命令之前都需要放一个0x00，表示这是命令。
    // 新建缓冲区。
    size_t total_size = 2 * cmd_length;  // 需要发送的数据包总尺寸。
    uint8_t* cmd_buffer = new uint8_t[total_size];
    for (uint16_t i = 0; i < cmd_length; i ++) {
        cmd_buffer[2 * i] = (i < cmd_length - 1) ? 0x80 : 0x00; // 如果不是最后一条指令，则控制位的高位为1，否则为0。
        cmd_buffer[2 * i + 1] = cmds[i];
    }
    // 然后发送命令。
    esp_err_t result = i2c_master_transmit(this->device, cmd_buffer, 2 * cmd_length, -1);
    if (result != ESP_OK) {
        ESP_LOGE("LUNA", "| Byte transmission failed.");
    }
    delete[] cmd_buffer;
    cmd_buffer = nullptr;  // 防一手野指针。
    return result;
};

esp_err_t SSD1306::transfer_cmd(const uint8_t& cmd){
    return transfer_cmd(&cmd, 1);
};

esp_err_t SSD1306::transfer_image_for_page(const uint8_t& page, const uint8_t &col_begin, 
    const uint8_t* image, const size_t num_of_cols) {
    
    if (page >= this->num_pages) return ESP_FAIL;   // 页码不能越界。
    if (col_begin >= this->width) return ESP_FAIL;  // 同样，行号也不行。

    // 选择页，然后从起始列到结束列，发送对应数据。
    // 这是命令-数据的混合内容，其中有3条命令：选择页、选择起始列、选择结束列。
    // 然后才是发送数据。
    size_t image_length = num_of_cols;
    size_t total_length = image_length + 3;

    uint8_t* cmd_buffer = new uint8_t[2 * total_length];

    // 分解列地址为高4位和低4位。傻逼SSD1306，非要将列地址设置为2份半字节。
    uint8_t col_high = (uint8_t)((col_begin >> 4) & (uint8_t)0x0F) + (uint8_t)0x10;  // 高4位（指令）
    uint8_t col_low = col_begin & (uint8_t)0x0F;                 // 低4位（指令）

    uint8_t command_bytes[3] = {(uint8_t)((uint8_t)0xB0 + page), col_low, col_high};

    // 开始拼接数据。
    for (uint16_t i = 0; i < total_length; i++) {
        // 处理控制字节。
        if (i < 3) {  // 前3位为指令
            cmd_buffer[2 * i] = 0x80;  // HEX: 0b10000000 = 0x80 发送指令，非尾帧
        } else if (i < total_length - 1) {  // 后续为数据，注意处理尾帧
            cmd_buffer[2 * i] = 0xC0;  // HEX: 0b11000000 = 0xC0 发送数据，非尾帧
        } else {
            cmd_buffer[2 * i] = 0x40;  // HEX: 0b01000000 = 0x40 发送数据，尾帧
        }

        // 处理数据字节。
        // 前3条是指令数据。
        if (i < 3) {
            cmd_buffer[2 * i + 1] = command_bytes[i];
        }
        // 剩下的全是图像数据。
        if (i >= 3) {
            cmd_buffer[2 * i + 1] = image[i - 4];
        }

    }
    // 发送数据。
    esp_err_t result = i2c_master_transmit(this->device, cmd_buffer, 2 * total_length, -1);
    if (result != ESP_OK) {
        ESP_LOGE("LUNA", "| Byte transmission failed.");
    }

    // 发送结束后调度，清缓存。
    delete[] cmd_buffer;
    return result;
};

void SSD1306::init_screen_buffer() {
    this->num_pages = this->height / 8;
    // 先计算页数。
    // 现在有8页，每页有128列。
    this->screen_buffer = new uint8_t*[this->num_pages]();  // 这里也需要初始化。
    // 再构建内容。
    for (uint16_t i = 0; i < this->num_pages; i++) {
        // 对每一页，构造一个新的数组。并且初始化每一个内容为0。
        this->screen_buffer[i] = new uint8_t[this->width](0);
    }
    // 然后对cmd_buffer进行初始化，
    this->page_buffer = new uint8_t[this->width];

    ESP_LOGI("LUNA", "| Screen buffer initialized.");


};

void SSD1306::init_i2c_bus() {
    esp_err_t result = i2c_new_master_bus(&this->bus_config, &this->bus);
    if (result != ESP_OK) {
        ESP_LOGE("LUNA", "| I2C bus initialization failed.");
        return;
    }
    result = i2c_master_bus_add_device(this->bus, &this->device_config, &this->device);
    if (result != ESP_OK) {
        ESP_LOGE("LUNA", "| I2C bus initialization failed.");
        return;
    }
    feed();  // 喂狗
    ESP_LOGI("LUNA", "| I2C bus and hardware initialized.");
};

void SSD1306::init_screen() {
    esp_err_t ret;

    // 等待ACK后，发送数据。transmit指令会处理ACK。
    ESP_LOGI("LUNA", "| Address initialized, now initializing screen.");
    ret = transfer_cmd(SSD1306::ssd1306_init_cmds, 25);  // 共25条指令。
    if (ret != ESP_OK) {
        ESP_LOGE("LUNA", "| Data transmission failed.");
        return;
    }
    std::cout << "| Screen init command sent." << std::endl;
    feed();      // 喂狗
    i2c_master_bus_wait_all_done(this->bus, -1);
    ESP_LOGI("LUNA", "| Screen hardware initialized.");
};

// public:
SSD1306::SSD1306() : width(128), height(64) {
    this->total_init();
};

SSD1306::SSD1306(uint16_t width, uint16_t height) : width(width), height(height) {
    this->total_init();    
};

void SSD1306::total_init() {
    // 获取当前任务句柄
    this->current_task = xTaskGetCurrentTaskHandle();
    // 将当前任务扔给看门狗让它看着
    ESP_ERROR_CHECK(esp_task_wdt_add(this->current_task));

    this->init_screen_buffer();
    this->init_i2c_bus();
    this->init_screen();

    //this->transfer_cmd((uint8_t)0xAE);
};

SSD1306::~SSD1306() {
    // 等待所有I2C任务执行完毕。
    std::cout << "| Waiting for I2C tasks to finish." << std::endl;
    i2c_master_bus_wait_all_done(this->bus, -1);

    // 释放资源。
    i2c_master_bus_rm_device(this->device);
    i2c_del_master_bus(this->bus);
    // 清理屏幕缓冲区中，用new规定的堆内存。
    for (uint16_t i = 0; i < this->num_pages; i++) {
        delete[] this->screen_buffer[i];
    }
    delete[] this->screen_buffer;

    // 删除任务。
    if (this->current_task) {
        ESP_ERROR_CHECK(esp_task_wdt_delete(this->current_task));
    }
};

void SSD1306::draw_pixel(const uint16_t& x, const uint16_t& y, bool value) {
    // 画图时间到，但我需要一个缓冲区。
    // 注意：这里有数组越界的问题。
    if (x >= this->width || y >= this->height) {
        return;
    };  // 超越边界则不画。
    // 现在的情况是按页码分。
    uint16_t page = (y / 8);  // 计算页，页码的index从0开始。
    uint16_t bit_pos = y % 8; // 计算在页中的位置
    
    if (value) {
        this->screen_buffer[page][x] |= (1 << bit_pos);  // 1
    } else {
        this->screen_buffer[page][x] &= ~(1 << bit_pos); // 0
    }
};

void SSD1306::flush() {    
    // 准备一个足够大的缓冲区，用于批量传输
    esp_err_t ret;
    // 对每一页而言：
    for (uint16_t page = 0; page < this->num_pages; page++) {
        // 逐页传输数据。我有一个按页并选择初始和终止位置的函数。
        // 该死，第5页没发出去。
        ret = this->transfer_image_for_page((uint8_t)page, 0, 
            this->screen_buffer[page], this->width);
        if (ret != ESP_OK) {
            ESP_LOGE("LUNA", "| Data transmission failed.");
            return;
        }
        feed();
        i2c_master_bus_wait_all_done(this->bus, -1);
    }
    ESP_LOGI("LUNA", "Screen flush completed.");
};


// 用于打印缓冲区（调试时查看内容）
void SSD1306::print_buffer() {
    std::cout << " | Buffer: " << std::endl;
    for (uint8_t page = 0; page < num_pages; page++) {  // 遍历每一页
        for (uint8_t bit_pos = 0; bit_pos < 8; bit_pos++) {  // 遍历页内的 8 行
            for (uint8_t x = 0; x < width; x++) {  // 遍历列
                bool pixel_on = (screen_buffer[page][x] & (1 << bit_pos));  // 逐位读取
                std::cout << (pixel_on ? "#" : ".");  // 1 = "#"，0 = "."
            }
            std::cout << std::endl;  // 换行，代表每 8 位的某一行
        }
    }
    std::cout << " | Buffer printed." << std::endl;
}
