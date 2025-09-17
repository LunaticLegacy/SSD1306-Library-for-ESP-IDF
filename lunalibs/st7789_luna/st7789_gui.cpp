#include <cstdio>
#include <cstdint>
#include <cstdarg>
#include <cstdlib>
#include <cstring>

#include "./st7789_basic.hpp"
#include "./st7789_gui.hpp"

namespace Luna
{
// =-=-=-=-=-=-=-=-=-=-=-=-= 图形库开始 =-=-=-=-=-=-=-=-=-=-=-=-=

ST7789::ST7789(const ST7789_Config& config) :
    ST7789_Basic(config) {
        // 先直接调用父类构造函数，然后重置鼠标指针位置。
        this->cursor.x = 32;
        this->cursor.y = 32;
    }

void ST7789::printf(const char* fmt, ...) {
    char buffer[256];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    for (size_t i = 0; i < std::strlen(buffer); i++) {
        // 非显示字符的场合进行特殊处理，但不绘图。
        switch(buffer[i]) {
            case '\n':  // 换行
                cursor.x = cursor.begin_x;
                cursor.y += using_font->height;
                break;
            case '\r':  // 回车，只复位X，不动Y
                cursor.x = cursor.begin_x;
                break;
            case '\t':  // 制表符，每4个字符宽度
                cursor.x += using_font->width * 4;
                if (cursor.x + using_font->width > getWidth()) {
                    cursor.x = cursor.begin_x;
                    cursor.y += using_font->height;
                }
                break;
            default: {  // 普通字符
                drawChar(this->cursor.x, this->cursor.y, buffer[i], this->using_font, 
                    this->cursor.color);
                // 加宽。
                this->cursor.x += this->using_font->width;
                // 超越边界的场合，换行。
                if (this->cursor.x + this->using_font->width > getWidth()) {
                    this->cursor.x = this->cursor.begin_x;
                    this->cursor.y += this->using_font->height;
                }
                break;
            }
        }
    }
    return;
}

void ST7789::drawFrame(
    uint16_t x, uint16_t y, uint16_t w, uint16_t h, 
    uint16_t color_inner, uint16_t color_frame, uint16_t frame_width
) {
    this->drawRectangle(x, y, w, h, color_frame);
    this->drawRectangle(x + frame_width, y + frame_width, w - 2 * frame_width, h - 2 * frame_width, color_inner);
}

void ST7789::drawChar(
    uint16_t x, uint16_t y, char c, const FontDef* font, uint16_t color
) {
    if (!font) return;
    
    if (c < 32 || c > 126) return;

    // 解析字体尺寸。
    uint8_t font_width = font->width, font_height = font->height;
    // 初始化x和y的位置。
    uint16_t now_x = x, now_y = y;

    // 选中字母。
    Alphabet letter = font->data[c-32];
            
    // 遍历像素并写入缓冲区。6列8行，所以int8_t的次序是从上到下解析。
    for (int16_t k = 0; k < font_width; k++) {
        // 当前位图列。
        uint8_t column_bitmap = letter.letter[k];
        for (int16_t j = 0; j < font_height; j++) {
            // font_height = 8，此时必须一个一个地、自上而下地解析。
            // 该像素是否为1？
            uint8_t mask = (1<<j);
            // uint8_t mask = ((1<<(font_height-1))>>j);
            if ((mask & column_bitmap)) {
                this->front_buffer[now_x * this->getWidth() + now_y] = color;
            } else {
                this->front_buffer[now_x * this->getWidth() + now_y] = 0xFFFF;
            }
            // 继续解析。
            now_y++;
        }
        // 下一列。
        now_y = y; now_x++;
    }
    
}

// -=-=-=-=-=-=-=-=-=-=-= 组件 =--=-=-=-=-=-=-=-=-=-=-
// 开始写组件。
ComponentBase::ComponentBase(ST7789* base)
    : target_driver(base) {
    // 这里的构造函数……就算了吧。
} 

ComponentBase::~ComponentBase() {

}

// =========== 告示牌 ============
Sign::Sign(ST7789* base, SignInfo&& sign_info) 
    : ComponentBase(base), sign_info(std::move(sign_info)) {

}

} // namespace Luna
