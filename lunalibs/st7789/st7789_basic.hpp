#ifndef __ST7789_BASIC_H
#define __ST7789_BASIC_H

// 这个东西采用全双工通信，SPI。且具有一个背光接口。
#include <cstdint>
// SPI协议用哪个？我的单片机是主机。
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_mac.h"


#ifndef __ST7789_CMD_TABLE
#define __ST7789_CMD_TABLE
// 命令表。
// 屏幕基础操作指令。
#define NOP 0x00                    // 无操作
#define SWRESET 0x01                // 软件重置（Software Reset）
#define SLPIN 0x10                  // 进入睡眠
#define SLPOUT 0x11                 // 离开睡眠
#define INVOFF 0x20                 // 屏幕反转关
#define INVON 0x21                  // 屏幕反转开
#define DISPOFF 0x28                // 关屏，只是不将缓存内容输出到屏幕中
#define DISPON 0x29                 // 开屏，和上方相反

#define MADCTL 0x36                 // 设置显示方向，需要输入1个byte的数据以决定方向
/*
* bit顺序：可忽略bit0和bit1
* bit2：更新方向(Display Data Latch Order)，0=从左到右，1=从右到左。
* bit3：按RGB还是BGR读取。0=RGB顺序，1=BGR顺序。
* bit4到bit7：建议填0，详见说明书215页。
*/ 
#define COLMOD 0x3A                 // 设置颜色模式，需要输入1个byte的数据以决定颜色模式。

/*
* bit0到bit2：011=每个像素12bit（RGB444），101=每个像素16bit（RGB565），110=每个像素18bit（RGB666），111=每个像素24bit（RGB888）。
* bit3以上：建议全0。
*/
#define RDCOLMOD 0x0C

// 选地址指令。
#define CASET 0x2A                  // 按列设置地址。
#define RASET 0x2B                  // 按行设置地址。
/*
* 这部分指令使用较为复杂：
* 在输入指令码后，需要输入4个byte，分别按列/行进行地址选取。数据码中每2个byte为1个实际决定坐标的数字，即该坐标为uint16_t类。
* 例：
* 0x2A - (然后拉高dc) - 0x00 0x00 0x00 0x7F - (然后拉低dc)
* 此时，决定列为0列开始，127列结束。
* 随后：
* 0x2B - (然后拉高dc) - 0x00 0x00 0x00 0xFF - (然后拉低dc)
* 此时，决定行为0行开始，255行结束。
* 即上述操作选中以(0, 0)到(127, 255)为对角的矩形区域。
*/


// 绘制指令。
#define DRAW 0x2C                  // 将颜色数据写入帧内存中。
/*
* 最多可以在未来接入3个byte的数据，取决于显示器接受颜色的模式。（RGB565 RGB555 RGB444 RGB888）
*/

#endif

namespace Luna{

class ST7789_Basic {
public:
    // 默认构造函数：采用默认值。
    ST7789_Basic() = delete;

    // 禁止复制实例。
    ST7789_Basic(const ST7789_Basic& other) = delete;

    // 重载构造函数：屏幕分辨率和引脚编号均存在的场合。
    ST7789_Basic(int16_t width, int16_t height, int16_t din, int16_t clk, int16_t cs, int16_t dc, int16_t rst, int16_t bl);

    // 初始化屏幕
    void begin();

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
    void writeCommand(uint8_t command);

    // 向屏幕发送数据
    void writeData(const uint8_t data[], size_t len);
    

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

    // 颜色模式。
    uint8_t _colorMode;

    // 当前显示情况
    int16_t _windowWidth = this->_width, _windowHeight = this->_height;

    // 首先，需要一个初始化SPI协议的函数。
    void initSPI();

    // 设置颜色模式。
    void setColorMode(uint8_t mode);

    // 设置窗口。
    void setWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);

};

} // namespace Luna


#endif // ST7789_BASIC_H