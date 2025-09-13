#include <cstdint>
#include <stdio.h>

#include "soc/gpio_periph.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_mac.h"
#include "esp_system.h"
// 头文件用的。
#include "./st7789_basic.hpp"

namespace Luna
{

// 开机初始化指令。
static const uint8_t init_cmds[] = {
    SWRESET,
    SLPOUT,
    DISPON,
};

// class ST7789:
// public:
// 禁止默认构造。

// 重载构造函数：屏幕分辨率和引脚编号均存在的场合。
ST7789_Basic::ST7789_Basic(int16_t width, int16_t height, int16_t din, int16_t clk, int16_t cs, int16_t dc, int16_t rst, int16_t bl) 
        : _width(width), _height(height), _din(din), _clk(clk) , _cs(cs), _dc(dc), _rst(rst), _bl(bl) {
    this->initSPI();
}

// 初始化函数
void ST7789_Basic::begin() {
    // 配置引脚
    printf("Initializing ST7789...\n");
    gpio_set_direction((gpio_num_t)this->_rst, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)this->_dc, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)this->_bl, GPIO_MODE_OUTPUT);

    // 拉高硬件复位。
    gpio_set_level((gpio_num_t)this->_rst, 1);

    // 复位屏幕
    printf("Resetting screen...\n");

    // 初始化指令
    for (uint8_t cmd : init_cmds) {
        writeCommand(cmd);

        if (cmd == SLPOUT || cmd == SWRESET || cmd == DISPON) {
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }

    // 颜色模式，RGB565
    setColorMode(0x05);

    // 设置列和行的默认窗口
    setWindow(0, 0, this->_width - 1, this->_height - 1);

    // 打开显示
    writeCommand(DISPON);

    // 打开背光
    gpio_set_level((gpio_num_t)this->_bl, 1);
    printf("Initialization complete.\n");
}

// 绘制点
void ST7789_Basic::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= this->_width) || (y < 0) || (y >= this->_height)) {
        printf("Pixel out of bounds: x=%d, y=%d\n", x, y);
        return;
    }

    setWindow(x, y, x, y);

    writeCommand(DRAW);

    // 写入颜色数据
    uint8_t colorData[2] = {
        static_cast<uint8_t>(color >> 8),  // 高字节
        static_cast<uint8_t>(color & 0xFF) // 低字节
    };
    writeData(colorData, 2);
}



// 绘制线段
void ST7789_Basic::drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color) {
    int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
    int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
    int err = dx + dy, e2;

    while (true) {
        drawPixel(x0, y0, color);

        if (x0 == x1 && y0 == y1) break;  // 结束条件
        e2 = 2 * err;
        if (e2 >= dy) { err += dy; x0 += sx; }
        if (e2 <= dx) { err += dx; y0 += sy; }
    }
}

// 绘制矩形
void ST7789_Basic::drawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    setWindow(x, y, x + w - 1, y + h - 1);

    // 准备颜色数据
    uint8_t colorData[2] = {
        static_cast<uint8_t>(color >> 8),  // 高字节
        static_cast<uint8_t>(color & 0xFF) // 低字节
    };

    setWindow(x, y, x + w - 1, y + h - 1);
    writeCommand(DRAW);

    // 这里可能吃看门狗
    for (int i = 0; i < h; i++) {
        writeData(colorData, 2);
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}


// 填充屏幕
void ST7789_Basic::fillScreen(uint16_t color) {
    drawRectangle(0, 0, this->_width, this->_height, color);
}


// 设置背光。
void ST7789_Basic::setBacklight(bool state){
    gpio_set_level((gpio_num_t)this->_bl, state);
};

// private:
// 首先，需要一个初始化SPI协议的函数。
void ST7789_Basic::initSPI() {
    // 将所有涉及的信号线加入上拉电阻

    printf("Initializing SPI bus...\n");
    this->_buscfg.mosi_io_num = this->_din;
    this->_buscfg.miso_io_num = -1;
    this->_buscfg.sclk_io_num = this->_clk;
    this->_buscfg.quadwp_io_num = -1;
    this->_buscfg.quadhd_io_num = -1;
    this->_buscfg.max_transfer_sz = this->_width * this->_height * 2 + 10;
    this->_buscfg.intr_flags = ESP_INTR_FLAG_IRAM;  // 指定中断标志
    this->_buscfg.isr_cpu_id = (esp_intr_cpu_affinity_t)ESP_INTR_CPU_AFFINITY_1;  // 指定核心

    esp_err_t ret;
    // 如果采用1口，就是SPI1_HOST，那么它本来就是已经初始化了的。
    ret = spi_bus_initialize(SPI2_HOST, &_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Failed to initialize SPI bus: %d\n", ret);  
        return;
    }

    printf("Configuring SPI device...\n");
    this->_devcfg.clock_speed_hz = 10 * 1000 * 1000;
    this->_devcfg.spics_io_num = this->_cs;
    this->_devcfg.queue_size = 7;
    this->_devcfg.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;
    this->_devcfg.mode = 0;

    ret = spi_bus_add_device(SPI2_HOST, &_devcfg, &_spi);
    if (ret != ESP_OK) {
        printf("Failed to add SPI device: %d\n", ret);
        return;
    }
}

void ST7789_Basic::setColorMode(uint8_t mode) {
    writeCommand(COLMOD);  // 接口像素格式命令
    uint8_t transfer_data[1] = { mode };
    
    writeData(transfer_data, 1);
    this->_colorMode = mode;
}

// 写入指令。
void ST7789_Basic::writeCommand(uint8_t command) {
    printf("Writing command: 0x%02X\n", command);

    gpio_set_level((gpio_num_t)this->_dc, 0);  // 拉低数据/命令引脚，进入指令模式
    spi_transaction_t t = {
        .length = 8,
        .tx_buffer = &command,
    };

    esp_err_t ret = spi_device_polling_transmit(this->_spi, &t);
    if (ret != ESP_OK) {
        printf("SPI command error: %d\n", ret);
    }

}

// 写入数据。
void ST7789_Basic::writeData(const uint8_t data[], size_t len) {

    printf("Writing data: ");
    for (size_t i = 0; i < len; i++) {
        printf("%02X ", data[i]);
    }
    printf("\n");
    // 必须显式规定数组长度 - 函数不知道数组具体有多长。
    gpio_set_level((gpio_num_t)this->_dc, 1);  // 数据模式
    // 初始化 SPI 事务
    spi_transaction_t t = {};
    t.flags = 0;                               // 默认无特殊标志
    t.length = 8 * len;                        // 总发送bit数
    t.tx_buffer = data;                        // 数据指针，直接指向传入的数据数组

    esp_err_t ret = spi_device_polling_transmit(this->_spi, &t);
    if (ret != ESP_OK) {
        printf("SPI data transmission error: %d\n", ret);
    }

}


// 设置窗口。
void ST7789_Basic::setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // 设置列地址
    writeCommand(CASET);  // CASET
    data[0] = x0 >> 8;   // 起始列高字节
    data[1] = x0 & 0xFF; // 起始列低字节
    data[2] = x1 >> 8;   // 结束列高字节
    data[3] = x1 & 0xFF; // 结束列低字节
    writeData(data, 4);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 设置行地址
    writeCommand(RASET);  // RASET
    data[0] = y0 >> 8;   // 起始行高字节
    data[1] = y0 & 0xFF; // 起始行低字节
    data[2] = y1 >> 8;   // 结束行高字节
    data[3] = y1 & 0xFF; // 结束行低字节
    writeData(data, 4);
    vTaskDelay(pdMS_TO_TICKS(10));

    // 更新窗口大小。
    this->_windowWidth = x1 - x0 + 1;
    this->_windowHeight = y1 - y0 + 1;

}


} // namespace Luna

