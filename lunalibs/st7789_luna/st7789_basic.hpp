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

// 屏幕指令头文件 - 宏定义，单独封装为一个头文件。
#include "./st7789_cmd_table.h"

namespace Luna{

// 配置内容结构体。
struct ST7789_Config {
    int16_t width;
    int16_t height;
    int16_t din;
    int16_t clk;
    int16_t cs;
    int16_t dc;
    int16_t rst;
    int16_t bl;
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

    // 设置背光
    void setBacklight(bool state);

    // 暂时提到public，调试完毕后再扔到private。
    // 向屏幕发送命令
    inline void writeCommand(uint8_t command);

    // 向屏幕发送数据
    inline void writeData(const uint8_t data[], size_t len);
    

private:
    // 硬件参数。
    int16_t _width, _height;  // 屏幕分辨率。
    int16_t _din, _clk, _cs, _dc, _rst, _bl;  // 引脚编号。
    // SPI相关：din：数据输入引脚。clk：时钟引脚。cs：片选引脚。
    // SPI无关：dc：数据/命令引脚。rst：复位引脚。bl：背光引脚。
    // dc拉低时接指令，拉高时接数据。

    // SPI总线配置。
    spi_bus_config_t _buscfg;
    spi_device_interface_config_t _devcfg;
    spi_device_handle_t _spi;  // 操作头。

    // 时钟频率设置。
    uint32_t _clockSpeed;

    // 颜色模式。
    uint8_t _colorMode;

    // 当前窗口长宽。
    int16_t _windowWidth, _windowHeight;

    // 初始化SPI配置，该函数需在构造函数时调用。
    void initSPI();


    // 设置窗口范围。
    void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

};

} // namespace Luna


#endif // ST7789_BASIC_H