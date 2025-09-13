#include <cstdint>
#include <stdio.h>

#include "soc/gpio_periph.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
// 头文件用的。
#include "./st7789_basic.hpp"

// 喂狗用。
#define feed esp_task_wdt_reset

namespace Luna
{

// 开机初始化指令。
// 傻逼waveshare，你的屏幕把我坑进去6个小时。
static constexpr uint8_t init_cmds[] = {
    SWRESET,                // 软件复位
    SLPOUT,                 // 退出睡眠模式
    // 添加更多初始化命令
    COLMOD,     0x55,       // 设置色彩格式为16位RGB565（使用常见 0x55）
    0x21,                   // 反色
    MADCTL,     0x08,       // MADCTL_BGR
    DISPON,                 // 开启显示
};

// class ST7789:
// public:
// 禁止默认构造。

// 重载构造函数：载入配置结构体。
ST7789_Basic::ST7789_Basic(const ST7789_Config& config)
        : _width(config.width), _height(config.height), _din(config.din), 
        _clk(config.clk) , _cs(config.cs), _dc(config.dc), 
        _rst(config.rst), _bl(config.bl), _clockSpeed(config.spi_clock_hz) {
    _spi = nullptr;
    this->initSPI();
}

// 初始化函数
esp_err_t ST7789_Basic::begin() {
    // 配置引脚 
    printf("Initializing ST7789...\n");

    // 设置背光、片选和硬件重置端口为OUTPUT模式。
    gpio_set_direction((gpio_num_t)this->_rst, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)this->_dc, GPIO_MODE_OUTPUT);
    gpio_set_direction((gpio_num_t)this->_bl, GPIO_MODE_OUTPUT);

    // 拉高硬件复位，复位屏幕。
    printf("Resetting screen...\n");
    gpio_set_level((gpio_num_t)this->_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(1));
    gpio_set_level((gpio_num_t)this->_rst, 0);
    vTaskDelay(pdMS_TO_TICKS(250));
    gpio_set_level((gpio_num_t)this->_rst, 1);
    vTaskDelay(pdMS_TO_TICKS(250));

    // 初始化指令
    size_t i = 0;
    while (i < sizeof(init_cmds)) {
        uint8_t cmd = init_cmds[i];
        writeCommand(cmd);
        i++;
        
        // 检查是否有数据参数跟随命令
        bool hasData = false;
        int dataLen = 0;
        
        // 打表。
        switch(cmd) {
            case COLMOD: dataLen = 1; hasData = true; break;  // COLMOD
            case MADCTL: dataLen = 1; hasData = true; break;  // MADCTL
            case PORCH_CTRL: dataLen = 5; hasData = true; break;  // PORCH
            case GATE_CTRL: dataLen = 1; hasData = true; break;  // GATECTRL
            case VCOMS_CTRL: dataLen = 1; hasData = true; break;  // VCOMS
            case 0xC0: dataLen = 1; hasData = true; break;  // LCMCTRL
            case 0xC2: dataLen = 1; hasData = true; break;  // VDVVRHEN
            case 0xC3: dataLen = 1; hasData = true; break;  // VRHS
            case 0xC4: dataLen = 1; hasData = true; break;  // FRCTRL2
            case 0xC6: dataLen = 1; hasData = true; break;  // CABCCTRL
            case 0xD0: dataLen = 2; hasData = true; break;  // PVGAMCTRL
            default: dataLen = 0; hasData = false; break;   // 其他无参指令
        }
        
        if (hasData && (i + dataLen <= sizeof(init_cmds))) {
            writeData(&init_cmds[i], dataLen);
            i += dataLen;
        }
        
        // 给屏幕一些反应时间
        if (cmd == SLPOUT || cmd == SWRESET) {
            vTaskDelay(pdMS_TO_TICKS(200));
        } else {
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }
    printf(" | Initialization command running successfully.\n");

    // 打开背光
    gpio_set_level((gpio_num_t)this->_bl, 1);
    printf("Initialization complete.\n");

    return ESP_OK;
}

// 绘制点
void ST7789_Basic::drawPixel(int16_t x, int16_t y, uint16_t color) {
    // 边界检查。
    if ((x < 0) || (x >= this->_width) || (y < 0) || (y >= this->_height)) {
        printf("Pixel out of bounds: x=%d, y=%d\n", x, y);
        return;
    }

    // 设置当前绘制区窗口。
    setWindow(x, y, x, y);

    // 写命令。
    writeCommand(DRAW);

    // 设置颜色数据局部变量，并写颜色。该状态下的颜色只是16bit，我需要未来的适配。
    // 先实现最小原型。
    uint8_t colorData[2] = {
        static_cast<uint8_t>(color & 0x00FF),  // 低字节（面板可能要求低字节先）
        static_cast<uint8_t>(color >> 8),  // 高字节
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
// 每次构建一行数据，或者一个合理大小的缓冲区
void ST7789_Basic::drawRectangle(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color
) {
    setWindow(x, y, x + w - 1, y + h - 1);
    writeCommand(DRAW);

    // 构建一行像素缓冲区
    const size_t rowBytes = w * 2;
    uint8_t rowData[rowBytes];
    // 测试
    for (int i = 0; i < w; i++) {
        // 发送低字节优先以匹配某些面板的字节序
        rowData[i*2]   = static_cast<uint8_t>(color & 0x00FF);
        rowData[i*2+1] = static_cast<uint8_t>(color >> 8);
    }

    // 开始写数据，将像素缓冲区写入其中。
    for (int row = 0; row < h; row++) {
        writeData(rowData, rowBytes);
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
    this->_buscfg.mosi_io_num = this->_din;  // data in
    this->_buscfg.miso_io_num = -1;          // data out - 这块屏幕不需要它
    this->_buscfg.sclk_io_num = this->_clk;  // 时钟源
    this->_buscfg.quadwp_io_num = -1;        // 这是什么？
    this->_buscfg.quadhd_io_num = -1;        // 这啥？
    this->_buscfg.max_transfer_sz = this->_width * this->_height * 2 + 10;
    // 移除可能导致问题的中断标志和CPU亲和性设置
    this->_buscfg.intr_flags = 0;
    this->_buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_0;

    esp_err_t ret;

    // 如果采用1口，就是SPI1_HOST，那么它本来就是已经成功初始化了的。
    ret = spi_bus_initialize(SPI2_HOST, &_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Failed to initialize SPI bus: %d\n", ret);  
        return;
    }

    printf("Configuring SPI device...\n");
    this->_devcfg.clock_speed_hz = this->_clockSpeed;   // 时钟频率
    this->_devcfg.spics_io_num = this->_cs;             // chip select端口选择
    this->_devcfg.queue_size = 7;                       // 指令队列最大长度
    this->_devcfg.flags = SPI_DEVICE_HALFDUPLEX | SPI_DEVICE_NO_DUMMY;  // ？
    this->_devcfg.mode = 0;                             // ？

    // 在SPI总线加入设备。
    ret = spi_bus_add_device(SPI2_HOST, &_devcfg, &_spi);
    if (ret != ESP_OK) {
        printf("Failed to add SPI device: %d\n", ret);
        return;
    }
}

// 写入指令。
inline void ST7789_Basic::writeCommand(uint8_t command) {
    printf("Writing command: 0x%02X\n", command);
    
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        printf("WARNING: SPI device not initialized!\n");
        return;
    }

    gpio_set_level((gpio_num_t)this->_dc, 0);  // 拉低数据/命令引脚，进入指令模式
    spi_transaction_t t = {};
    t.length = 8;
    t.tx_buffer = &command;
    
    esp_err_t ret = spi_device_polling_transmit(this->_spi, &t);
    if (ret != ESP_OK) {
        printf("SPI command error: %d\n", ret);
    }
}

// 写入数据。
inline void ST7789_Basic::writeData(const uint8_t data[], size_t len) {
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        printf("SPI device not initialized!\n");
        return;
    }

    // 调试信息。
    // printf("Writing data: ");
    // for (size_t i = 0; i < len; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");

    // 必须显式规定数组长度 - 函数不知道数组具体有多长。
    gpio_set_level((gpio_num_t)this->_dc, 1);  // 数据模式
    // 初始化 SPI 事务
    spi_transaction_t t = {};
    t.flags = 0;                       // 默认无特殊标志
    t.length = 8 * len;                // 总发送bit数
    t.tx_buffer = data;                // 数据指针，直接指向传入的数据数组

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