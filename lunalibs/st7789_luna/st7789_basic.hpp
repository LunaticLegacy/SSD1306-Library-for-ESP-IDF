#ifndef __ST7789_BASIC_H
#define __ST7789_BASIC_H

#include <cstdint>

// 这个东西采用全双工通信，SPI。且具有一个背光接口。
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"

// 导入我的字体库。
#include "./font_luna.h"

// 屏幕指令头文件 - 宏定义，单独封装为一个头文件。
#include "./st7789_cmd_table.h"

namespace Luna{

// 配置内容结构体。
struct ST7789_Config {
    int16_t width;
    int16_t height;
    gpio_num_t din;
    gpio_num_t clk;
    gpio_num_t cs;
    gpio_num_t dc;
    gpio_num_t rst;
    gpio_num_t bl;
    uint32_t spi_clock_hz;
};

class ST7789_Basic {
public:
    // 默认构造函数：采用默认值。
    ST7789_Basic() = delete;

    // 禁止复制实例。
    ST7789_Basic(const ST7789_Basic& other) = delete;

    // 重载构造函数：支持载入配置结构体。
    ST7789_Basic(const ST7789_Config& config);

    // 初始化屏幕
    esp_err_t begin();

    // 绘制像素
    void drawPixel(int16_t x, int16_t y, uint16_t color);

    // 绘制线
    void drawLine(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
    
    // 绘制矩形
    void drawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);

    // 填充屏幕
    void fillScreen(uint16_t color);

    // 将全局缓冲区内容写入显示器，需手动调用以更新屏幕
    void flush();

    // 析构函数，释放缓冲区
    virtual ~ST7789_Basic();

    // 设置背光
    void setBacklight(bool state);

    // 向屏幕发送命令
    void writeCommand(uint8_t command);

    // 向屏幕发送数据
    void writeData(const uint8_t data[], size_t len);

    // 获取屏幕宽高
    uint16_t getHeight() const { return this->_height; }
    uint16_t getWidth() const { return this->_width; }
    

private:
    // 硬件参数。
    int16_t _width, _height;  // 屏幕分辨率。
    gpio_num_t _din, _clk, _cs, _dc, _rst, _bl;  // 引脚编号。
    // SPI相关：din：数据输入引脚。clk：时钟引脚。cs：片选引脚。
    // SPI无关：dc：数据/命令引脚。rst：复位引脚。bl：背光引脚。
    // dc拉低时接指令，拉高时接数据。

    // SPI总线配置。
    spi_bus_config_t _buscfg;
    spi_device_interface_config_t _devcfg;
    spi_device_handle_t _spi;  // 操作头。

    // 用于DMA的类内全局SPI事务。
    spi_transaction_t transaction_dma;

    // 时钟频率设置。
    uint32_t _clockSpeed;

    // 颜色模式。
    uint8_t _colorMode;

    // 当前窗口长宽。
    int16_t _windowWidth, _windowHeight;

    // 总像素数。
    int32_t _totalpx;

    // 未初始化的数组，等屏幕宽高数据出现后再初始化。
    uint16_t *front_buffer = nullptr;   // 前缓冲区。
    SemaphoreHandle_t signal;           // DMA已完成的信号量。qhandle_t
    // q_handle_t是一个指针。

    // 初始化SPI配置，该函数需在构造函数时调用。这个东西同时也要作为初始化DMA的内容。
    void initSPI();

    // 设置窗口范围。
    void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

    // 用于DMA的回调函数。
    static void dmaDoneCallback(spi_transaction_t* t) {
        // 从事务中获取ST7789_Basic实例
        if (t == nullptr) {
            printf("| WARNING: Invalid transaction.\n");
            return;
        }
        
        if (t->user == nullptr) {
            printf("| WARNING: No user found.\n");
            return;
        }
        
        ST7789_Basic* obj = static_cast<ST7789_Basic*>(t->user);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(obj->signal, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
};

// =-=-=-=-=-=-=-=-=-=-=-=-= 图形库开始 =-=-=-=-=-=-=-=-=-=-=-=-=

class ST7789 : private ST7789_Basic {
public:
    ST7789() = delete;
    ST7789(const ST7789& other) = delete;

    ST7789(const ST7789_Config& config);

    void printf(const char* arg, ...);

private:
    // 导入正在使用的字体。
    const FontDef* using_font = &Font_6x8;

};

} // namespace Luna


#endif // ST7789_BASIC_H