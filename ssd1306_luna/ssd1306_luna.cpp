#include "ssd1306_luna.h"

#include <cstdarg>
#include <cstdint>
#include <cstdio>

#include <iostream>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <vector>

// class SSD1306:
// protected:
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

void SSD1306::total_init() {
    this->init_screen_buffer();
    this->init_i2c_bus();
    this->init_screen();

    //this->transfer_cmd((uint8_t)0xAE);
};

// public:
SSD1306::SSD1306() : width(128), height(64) {
    this->total_init();
};

SSD1306::SSD1306(uint16_t width, uint16_t height) : width(width), height(height) {
    this->total_init();    
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
    i2c_master_bus_wait_all_done(this->bus, -1);
    ESP_LOGI("LUNA", "| Screen hardware initialized.");
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

void SSD1306::clear_screen_buffer() {
    for (uint16_t i = 0; i < this->num_pages; i++) {
        for (uint16_t j = 0; j < this->width; j++) {
            this->screen_buffer[i][j] = 0;
        }
    }
}

SSD1306::~SSD1306() {
    // 等待所有I2C任务执行完毕。
    i2c_master_bus_wait_all_done(this->bus, -1);

    // 释放资源。
    i2c_master_bus_rm_device(this->device);
    i2c_del_master_bus(this->bus);
    // 清理屏幕缓冲区中，用new规定的堆内存。
    for (uint16_t i = 0; i < this->num_pages; i++) {
        delete[] this->screen_buffer[i];
    }
    delete[] this->screen_buffer;
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
};

// end class SSD1306

// class SSD1306_SHAPES

// public:
// ---------------------------------------------------------------
// 图形绘制
// ---------------------------------------------------------------
// Bresenham直线算法
void SSD1306_SHAPES::draw_line(const uint16_t& x0, const uint16_t& y0, 
    const uint16_t& x1, const uint16_t& y1, bool value) {
    int16_t dx = abs(x1 - x0);
    int16_t dy = abs(y1 - y0);
    int16_t sx = x0 < x1 ? 1 : -1;
    int16_t sy = y0 < y1 ? 1 : -1;
    int16_t err = (dx > dy ? dx : -dy) / 2;
    int16_t e2;

    int16_t x = x0, y = y0;

    while (true) {
        draw_pixel(x, y, value);
        if (x == x1 && y == y1) break;
        e2 = err;
        if (e2 > -dx) { err -= dy; x += sx; }
        if (e2 < dy) { err += dx; y += sy; }
    }
};

// 中点圆算法
void SSD1306_SHAPES::draw_circle(const uint16_t& x_center, const uint16_t& y_center, 
      const uint16_t& radius, bool value, bool fill) {
    int16_t x = 0;
    int16_t y = radius;
    int16_t d = 1 - radius;

    auto plot_points = [&](int16_t x, int16_t y) {
    if (fill) {
        // 填充从圆心到点的线
        for (int16_t i = -x; i <= x; i++) {
            this->draw_pixel(x_center + i, y_center + y, value);
            this->draw_pixel(x_center + i, y_center - y, value);
        }
        for (int16_t i = -y; i <= y; i++) {
            this->draw_pixel(x_center + x, y_center + i, value);
            this->draw_pixel(x_center - x, y_center + i, value);
        }
        } else {
            this->draw_pixel(x_center + x, y_center + y, value);
            this->draw_pixel(x_center - x, y_center + y, value);
            this->draw_pixel(x_center + x, y_center - y, value);
            this->draw_pixel(x_center - x, y_center - y, value);
            this->draw_pixel(x_center + y, y_center + x, value);
            this->draw_pixel(x_center - y, y_center + x, value);
            this->draw_pixel(x_center + y, y_center - x, value);
            this->draw_pixel(x_center - y, y_center - x, value);
        }
    };

    while (x < y) {
        plot_points(x, y);
        x++;
        if (d < 0) {
            d += 2 * x + 1;
        } else {
            y--;
            d += 2 * (x - y) + 1;
        }
    }
};

// 矩形绘制
void SSD1306_SHAPES::draw_rectangle(const uint16_t& x0, const uint16_t& y0,
        const uint16_t& width, const uint16_t& height, 
        bool value, bool fill) {
    // 如果封住
    if (fill) {
        for (uint16_t y = y0; y < y0 + height; y++) {
            for (uint16_t x = x0; x < x0 + width; x++) {
                draw_pixel(x, y, value);
            }
        }
    } else {  // 否则
        // 绘制四条边
        for (uint16_t x = x0; x < x0 + width; x++) {
            draw_pixel(x, y0, value);
            draw_pixel(x, y0 + height - 1, value);
        }
        for (uint16_t y = y0; y < y0 + height; y++) {
            draw_pixel(x0, y, value);
            draw_pixel(x0 + width - 1, y, value);
        }
    }
};

// 三角形绘制
void SSD1306_SHAPES::draw_triangle(const uint16_t& x0, const uint16_t& y0,
       const uint16_t& x1, const uint16_t& y1,
       const uint16_t& x2, const uint16_t& y2,
       bool value, bool fill) {
    if (!fill) {
        draw_line(x0, y0, x1, y1, value);
        draw_line(x1, y1, x2, y2, value);
        draw_line(x2, y2, x0, y0, value);
    } else {
        // 填充三角形 - 使用扫描线算法
        int16_t min_y = std::min({y0, y1, y2});
        int16_t max_y = std::max({y0, y1, y2});

        for (int16_t y = min_y; y <= max_y; y++) {
            std::vector<int16_t> intersects;

            // 检查每条边与当前扫描线的交点
            if ((y0 <= y && y < y1) || (y1 <= y && y < y0))
                intersects.push_back(x0 + (y - y0) * (x1 - x0) / (y1 - y0));
            if ((y1 <= y && y < y2) || (y2 <= y && y < y1))
                intersects.push_back(x1 + (y - y1) * (x2 - x1) / (y2 - y1));
            if ((y2 <= y && y < y0) || (y0 <= y && y < y2))
                intersects.push_back(x2 + (y - y2) * (x0 - x2) / (y0 - y2));

            if (intersects.size() >= 2) {
                std::sort(intersects.begin(), intersects.end());
                for (int16_t x = intersects[0]; x <= intersects[1]; x++) {
                    draw_pixel(x, y, value);
                }
            }
        }
    }
};

// 菱形绘制
void SSD1306_SHAPES::draw_diamond(const uint16_t& x_center, const uint16_t& y_center,
      const uint16_t& width, const uint16_t& height,
      bool value, bool fill) {
    // 计算四个顶点
    uint16_t half_w = width / 2;
    uint16_t half_h = height / 2;

    if (!fill) {
    // 绘制四条边
        draw_line(x_center - half_w, y_center, x_center, y_center - half_h, value);
        draw_line(x_center, y_center - half_h, x_center + half_w, y_center, value);
        draw_line(x_center + half_w, y_center, x_center, y_center + half_h, value);
        draw_line(x_center, y_center + half_h, x_center - half_w, y_center, value);
    } else {
        // 填充菱形
        for (int16_t y = -half_h; y <= half_h; y++) {
            int16_t x_width = half_w * (half_h - abs(y)) / half_h;
            for (int16_t x = -x_width; x <= x_width; x++) {
                draw_pixel(x_center + x, y_center + y, value);
            }
        }
    }
};

// 绘制自定义点阵图
void SSD1306_SHAPES::draw_image(BinaryImage image, uint16_t x, uint16_t y) {
    if (!image.data) return;
    // 图像尺度，溢出的部分会被draw_pixel方法自动truncate掉。注意data是一个一维内容。
    for (uint16_t i = 0; i < image.width * image.height; i++) {
        for (uint16_t j = 0; j < image.width; j++) {
            draw_pixel(x + j, y + i / image.width, image.data[i]);
        }
    }

}


// ---------------------------------------------------------------
// 字体
// ---------------------------------------------------------------

uint16_t SSD1306_SHAPES::get_string_width(const char* str) {
    if (!current_font) return 0;
    uint16_t width = 0;
    while (*str) {
        width += current_font->width;
        str++;
    }
    return width;
}

void SSD1306_SHAPES::write_char(char c, uint16_t x, uint16_t y, bool value) {
    // 检查是否超出显示范围
    if (x >= this->width || y >= this->height) return;
    if (!current_font) return;
    
    // 计算字符在字体数据中的偏移
    uint32_t char_offset = (c - 32) * current_font->width;
    
    // 绘制字符
    for (uint8_t i = 0; i < current_font->width; i++) {
        uint8_t line = current_font->data[char_offset + i];
        for (uint8_t j = 0; j < 8; j++) {
            if (line & (1 << j)) {
                draw_pixel(x + i, y + j, value);
            }
        }
    }
}

void SSD1306_SHAPES::write_string(const char* str, uint16_t x, uint16_t y, bool value) {
    uint16_t cursor_x = x;
    // 由于printf没能实现对换行符的处理，这里只能自行实现。
    while (*str) {
        // 检查是否需要换行，如果抵达边界或检测到换行符：
        if (cursor_x + current_font->width > width or *str == '\n') {
            // 移动光标。
            cursor_x = x;   // 回到当前标记的左侧，而不是0。
            y += current_font->height;  // y进入到下一行
            // 换行符的场合，直接进入下1个字符。
            if (*str == '\n') {
                str++;
                continue;
            }
        }
        // 绘制字符
        write_char(*str, cursor_x, y, value);
        cursor_x += current_font->width;
        str++;
        

    }
}

void SSD1306_SHAPES::printf(const char* format, ...) {
    char buffer[128];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    write_string(buffer, cursor_x, cursor_y, true);
    cursor_x += strlen(buffer) * current_font->width;
}