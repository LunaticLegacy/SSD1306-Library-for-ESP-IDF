#ifndef __ST7789_GUI_HPP
#define __ST7789_GUI_HPP

#include "./st7789_basic.hpp"
#include "./font_luna.h"


namespace Luna
{
    
// =-=-=-=-=-=-=-=-=-=-=-=-= 图形库开始 =-=-=-=-=-=-=-=-=-=-=-=-=

class ST7789 : public ST7789_Basic {
public:
    ST7789() = delete;
    ST7789(const ST7789& other) = delete;

    ST7789(const ST7789_Config& config);

    void printf(const char* arg, ...);

    // 移动鼠标指针，不重载。
    void moveCursor(const uint16_t x, const uint16_t y) {
        this->cursor.x = x; this->cursor.y = y;
        this->cursor.begin_x = x; this->cursor.begin_y = y;
    }

    void changeCursorColor(const uint16_t color) {
        this->cursor.color = color;
    }

    // 画框。
    void drawFrame(
        uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
        uint16_t color_inner, uint16_t color_frame, uint16_t frame_width
    );

private:
    // 导入正在使用的字体。
    const FontDef* using_font = &Font_6x8;

    // 保存一个uint16_t数组，作为当前屏幕鼠标指针位置。
    typedef struct cursor {
        uint16_t x = 0;
        uint16_t y = 0;
        uint16_t begin_x = 0;
        uint16_t begin_y = 0;
        uint16_t color = 0xFFFF;
    } Cursor;
    // 指针。
    Cursor cursor;

    // 绘制字符串，目前暂不支持绘制非ASCII字符。
    void drawChar(
        uint16_t x, uint16_t y, char c, const FontDef* font, uint16_t color
    );
};

// 组件，必须绑定到特定屏幕上。
// 这是一个抽象类。
class ComponentBase {
public:

    // 禁止默认构造。
    ComponentBase() = delete;

    // 需要输入ST7789的内容。
    explicit ComponentBase(ST7789* base);

    // 虚析构函数。
    virtual ~ComponentBase();

    virtual void draw() = 0;

    virtual ST7789* getDriver() const { return target_driver; }

private:
    ST7789* target_driver;

    uint16_t x, y;
}

} // namespace Luna

#endif
