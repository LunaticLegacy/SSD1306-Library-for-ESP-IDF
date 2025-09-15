#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "soc/gpio_periph.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
// 添加ets_delay_us函数的头文件
#include "esp_rom_sys.h"
// heap_caps for DMA-capable free
#include "esp_heap_caps.h"

// 头文件用的。
#include "./st7789_basic.hpp"
// 导入我的字体库。
#include "./font_luna.h"

// 喂狗用。
#define feed esp_task_wdt_reset

namespace Luna
{

// 开机初始化指令。
static constexpr uint8_t init_cmds[] = {
    SWRESET,                // 软件复位
    SLPOUT,                 // 退出睡眠模式
    // 添加更多初始化命令
    COLMOD,     0x05,       // 设置色彩格式为16位RGB565（使用常见 0x55）
    MADCTL,     0x20,       // MADCTL设置为BGR，反转上下。
    PORCH_CTRL, 0x08, 0x08, 0x08, 0x33, 0x33, // 前后廊设置
    GATE_CTRL,  0x11,       // 门控设置
    VCOMS_CTRL, 0x35,       // VCOM设置
                      // 反色
    DISPON,                 // 开启显示
};

// class ST7789:
// public:
// 禁止默认构造。

// 重载构造函数：载入配置结构体。
ST7789_Basic::ST7789_Basic(const ST7789_Config& config)
        : _width(config.width), _height(config.height), _din(config.din), 
        _clk(config.clk) , _cs(config.cs), _dc(config.dc), 
        _rst(config.rst), _bl(config.bl), _clockSpeed(config.spi_clock_hz),
        _totalpx(config.width * config.height) {
    _spi = nullptr;
    std::printf("| Hello, ST7789!\n");

    // 分配数组。
    this->front_buffer = (uint16_t*)heap_caps_malloc(this->_totalpx * sizeof(uint16_t), MALLOC_CAP_DMA);

    if (this->front_buffer == nullptr) {
        std::printf("ERROR: Failed to allocate memory for screen buffer.\n");
        heap_caps_free(this->front_buffer);
        return;
    }

    // 页临时缓冲区。
    this->temp_page_buffer = (uint16_t*)heap_caps_malloc(this->_totalpx * sizeof(uint16_t) / 64, MALLOC_CAP_DMA);
    if (this->temp_page_buffer == nullptr) {
        std::printf("ERROR: Failed to allocate memory for temporal page buffer.\n");
        heap_caps_free(this->temp_page_buffer);
        return;
    }

    // 为脏页分配空间。
    this->dirty_map = new uint8_t[8];
    if (this->dirty_map == nullptr) {
        std::printf("ERROR: Failed to allocate bitmap for dirty page.\n");
        delete[] this->dirty_map;
        return;
    }
    memset(this->dirty_map, 0, 8);
    
    // 分配5个事务，并将其压入队列中。
    for (uint16_t i = 0; i < this->transaction_num; i++) {
        spi_transaction_t* t = new spi_transaction_t({});
        // 分配失败的场合，立即释放已分配的资源。
        if (t == nullptr) {
            std::printf("ERROR: Failed to allocate memory for transaction deques\n");
            for (auto& p : this->transaction_deque) {
                delete p;
                p = nullptr;
            }
            return;
        }
        transaction_deque.push_back(t);
    }

    std::printf("| Memory allocated.\n");
    // 初始化为全黑。用memset也不影响，对吧 ;)
    std::memset(this->front_buffer, 0, this->_totalpx * sizeof(uint16_t));

    this->initSPI();
    
    // 检查SPI初始化是否成功
    if (this->_spi == nullptr) {
        std::printf("ERROR: SPI initialization failed\n");
        return;
    }
}

// 初始化函数
esp_err_t ST7789_Basic::begin() {
    // 配置引脚 
    std::printf("Initializing ST7789...\n");

    // 检查SPI是否已初始化
    if (this->_spi == nullptr) {
        std::printf("ERROR: SPI device not initialized. Call initSPI() first.\n");
        return ESP_FAIL;
    }

    // 设置背光、片选和硬件重置端口为OUTPUT模式。
    esp_err_t ret;
    ret = gpio_set_direction((gpio_num_t)this->_rst, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        std::printf("Failed to set RST pin direction: %d\n", ret);
        return ret;
    }
    
    ret = gpio_set_direction((gpio_num_t)this->_dc, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        std::printf("Failed to set DC pin direction: %d\n", ret);
        return ret;
    }
    
    ret = gpio_set_direction((gpio_num_t)this->_bl, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        std::printf("Failed to set BL pin direction: %d\n", ret);
        return ret;
    }

    // 拉高硬件复位，复位屏幕。
    std::printf("Resetting screen...\n");
    ret = gpio_set_level((gpio_num_t)this->_rst, 1);
    if (ret != ESP_OK) {
        std::printf("Failed to set RST pin level high: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ret = gpio_set_level((gpio_num_t)this->_rst, 0);
    if (ret != ESP_OK) {
        std::printf("Failed to set RST pin level low: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(250));
    
    ret = gpio_set_level((gpio_num_t)this->_rst, 1);
    if (ret != ESP_OK) {
        std::printf("Failed to set RST pin level high: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(250));


    std::printf("Executing init commands...\n");
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

    vTaskDelay(pdMS_TO_TICKS(50));
    std::printf(" | Initialization commandset running successfully.\n");

    // 打开背光
    ret = gpio_set_level((gpio_num_t)this->_bl, 1);
    if (ret != ESP_OK) {
        std::printf("Failed to set BL pin level: %d\n", ret);
        return ret;
    }

    // 然后重置屏幕。
    this->fillScreen(0x0000);
    this->flush();

    std::printf("Initialization complete.\n");

    return ESP_OK;
}

// 绘制点
void ST7789_Basic::drawPixel(int16_t x, int16_t y, uint16_t color) {
    // 边界检查。
    if ((x < 0) || (x >= this->_width) || (y < 0) || (y >= this->_height)) {
        std::printf("Pixel out of bounds: x=%d, y=%d\n", x, y);
        return;
    }

    // 设置像素。
    this->front_buffer[y * this->_width + x] = color;

    // 更新位图。
    this->dirty_map[y / (this->_height / 8)] |= (
        (1 << (7 - (x / (this->_width / 8))))
    );

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
void ST7789_Basic::drawRectangle(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    // 边界裁剪
    x = std::max<uint16_t>(x, 0);
    y = std::max<uint16_t>(y, 0);
    w = std::min<uint16_t>(w, this->_width - x);
    h = std::min<uint16_t>(h, this->_height - y);
    
    for (int yy = y; yy < y + h; yy++) {
        uint16_t* dst = &this->front_buffer[yy * this->_width + x];
        // 一次性填充整行
        for (int xx = 0; xx < w; xx++) {
            dst[xx] = color;
        }
    }

    // 并将涉及位图设为脏页，先写范围。
    uint8_t dx_begin = x / (this->_width / 8), dx_end = (x + w) / (this->_width / 8),
        dy_begin = y / (this->_height / 8), dy_end = (y + h) / (this->_height / 8);

    // 然后再进行位操作。
    for (uint8_t k = dy_begin; k < dy_end; k++) {
        for (uint8_t j = dx_begin; j < dx_end; j++) {
            this->dirty_map[k] |= (1<<j);
        }
    }
    
}

// 填充屏幕
void ST7789_Basic::fillScreen(uint16_t color) {
    // 将缓冲区全部填写为 color
    std::fill(this->front_buffer, this->front_buffer + this->_totalpx, color);

    // 再将位图全部位写为0xFF。
    memset(this->dirty_map, 0xFF, 8);
}


// 将缓冲区内容推到显示器，需要手动调用。（无锁队列，启动！）
void ST7789_Basic::flush() {
    esp_err_t ret;

    setWindow(0, 0, _width-1, _height-1);
    writeCommand(RAMWR);

    // 设置总大小。及缓冲区情况。
    size_t total_bytes = _totalpx * 2;
    uint8_t* ptr = reinterpret_cast<uint8_t*>(front_buffer);

    // 设置DC引脚为数据模式
    ret = gpio_set_level((gpio_num_t)this->_dc, 1);
    if (ret != ESP_OK) {
        std::printf("Failed to set DC pin level: %d\n", ret);
        return;
    }
    
    // 添加一个小延迟确保GPIO电平稳定
    esp_rom_delay_us(1);

    // 设置SPI传输事务。
    spi_transaction_t* t = this->transaction_deque.front();
    this->transaction_deque.pop_front();

    // 先重置内部的一切。
    std::memset(t, 0, sizeof(*t));
    t->length = total_bytes * 8;    // 长度，单位：bit
    t->tx_buffer = ptr;             // 发送的缓冲区
    t->user = this;                 // 事务发起用户，使用this指针

    // 轮询，发送事务。
    ret = spi_device_polling_transmit(_spi, t);
    if (ret != ESP_OK) {
        std::printf("SPI transmit error: %d\n", ret);
    }
    this->transaction_deque.push_back(t);

    // 再将位图全部位写为0x00。
    memset(this->dirty_map, 0x00, 8);
}

// 将缓冲区内容推到显示器，自动刷新脏页。
void ST7789_Basic::flushDirty() {
    esp_err_t ret;

    const int block_w = 8;
    const int block_h = 8;
    int blocks_x = (_width + block_w - 1) / block_w;
    int blocks_y = (_height + block_h - 1) / block_h;
    
    for (int by = 0; by < blocks_y; ++by) {
        for (int bx = 0; bx < blocks_x; ++bx) {
            int map_index = by * blocks_x + bx;
            if (!(this->dirty_map[map_index / 8] & (1 << (map_index % 8)))) {
                continue; // 这个块不脏
            }

            // 计算脏块的像素范围
            int x0 = bx * block_w;
            int y0 = by * block_h;
            int x1 = std::min(x0 + block_w - 1, _width - 1);
            int y1 = std::min(y0 + block_h - 1, _height - 1);

            // 设置窗口
            setWindow(x0, y0, x1, y1);
            writeCommand(RAMWR);

            // 写入页缓冲区。
            for (int y = y0; y <= y1; ++y) {
                for (int x = x0; x <= x1; ++x) {
                    this->temp_page_buffer[(y - y0) * block_w + (x - x0)] = this->front_buffer[y * _width + x];
                }
            }

            // 选择当前窗口内所有buffer，并发送。
            // 先上数据位。
            ret = gpio_set_level((gpio_num_t)this->_dc, 1);
            if (ret != ESP_OK) {
                std::printf("Failed to set DC pin: %d\n", ret);
                return;
            }
            esp_rom_delay_us(1);

            spi_transaction_t* t = this->transaction_deque.front();
            this->transaction_deque.pop_front();
            std::memset(t, 0, sizeof(*t));
            
            t->length = (x1 - x0 + 1) * 2 * 8;                 // 每行像素字节数，乘以页码列数。
            t->tx_buffer = (uint8_t*)this->temp_page_buffer;   // 当前行首地址
            t->user = this;
            ret = spi_device_polling_transmit(_spi, t);
            if (ret != ESP_OK) {
                std::printf("SPI transmit error: %d\n", ret);
            }
            this->transaction_deque.push_back(t);

            // 清除脏标志
            this->dirty_map[map_index / 8] &= ~(1 << (map_index % 8));
        }
    }
}


// 析构函数，释放缓冲区
ST7789_Basic::~ST7789_Basic() {
    // 清理buffer
    if (this->front_buffer) {
        heap_caps_free(this->front_buffer);
        this->front_buffer = nullptr;
    }

    // 删除事务队列内事务。
    for (uint16_t i = 0; i < this->transaction_num; i++) {
        delete this->transaction_deque.front();
        this->transaction_deque.front() = nullptr;
        this->transaction_deque.pop_front();
    }
}

// 设置背光。
void ST7789_Basic::setBacklight(bool state){
    gpio_set_level((gpio_num_t)this->_bl, state);
};

// private:
// 首先，需要一个初始化SPI协议的函数。
void ST7789_Basic::initSPI() {
    // 将所有涉及的信号线加入上拉电阻。

    std::printf("Initializing SPI bus...\n");
    this->_buscfg.mosi_io_num = this->_din;  // data in
    this->_buscfg.miso_io_num = -1;          // data out - 这块屏幕不需要它
    this->_buscfg.sclk_io_num = this->_clk;  // 时钟源
    this->_buscfg.quadwp_io_num = -1;        // 这是什么？
    this->_buscfg.quadhd_io_num = -1;        // 这啥？
    // max_transfer_sz is in BYTES; limit to one row (width * 2 bytes for RGB565)
    this->_buscfg.max_transfer_sz = this->_totalpx * 8;
    // 移除可能导致问题的中断标志和CPU亲和性设置
    this->_buscfg.intr_flags = 0;
    this->_buscfg.isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO;

    esp_err_t ret;

    // 初始化SPI总线
    ret = spi_bus_initialize(SPI2_HOST, &_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        std::printf("Failed to initialize SPI bus: %d\n", ret);  
        return;
    }

    std::printf("Configuring SPI device...\n");
    this->_devcfg.clock_speed_hz = this->_clockSpeed;     // 时钟频率
    this->_devcfg.spics_io_num = this->_cs;               // chip select端口选择
    this->_devcfg.queue_size = 8;                        // 增加指令队列大小
    this->_devcfg.flags = SPI_DEVICE_HALFDUPLEX;          // 半双工模式
    this->_devcfg.mode = 0;                               // SPI模式0
    this->_devcfg.pre_cb = nullptr;
    // 修复lambda表达式问题，使用静态函数替代
    this->_devcfg.post_cb = ST7789_Basic::dmaDoneCallback;

    // 在SPI总线加入设备。
    ret = spi_bus_add_device(SPI2_HOST, &_devcfg, &_spi);

    if (ret != ESP_OK) {
        std::printf("Failed to add SPI device: %d\n", ret);
        _spi = nullptr; // 确保_spi为nullptr表示初始化失败
        return;
    }
    
    std::printf("SPI device configured successfully.\n");
}

// 写入指令。
void ST7789_Basic::writeCommand(uint8_t command) {
    //std::printf("Writing command: 0x%02X\n", command);
    
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        std::printf("WARNING: SPI device not initialized!\n");
        return;
    }

    // 检查GPIO是否正确设置
    if (this->_dc < 0 || this->_dc >= GPIO_NUM_MAX) {
        std::printf("ERROR: Invalid DC pin: %d\n", this->_dc);
        return;
    }

    // 设置DC引脚为命令模式
    esp_err_t ret = gpio_set_level((gpio_num_t)this->_dc, 0);  // 拉低数据/命令引脚，进入指令模式
    if (ret != ESP_OK) {
        std::printf("Failed to set DC pin level: %d\n", ret);
        return;
    }
    
    // 添加一个小延迟确保GPIO电平稳定
    esp_rom_delay_us(1);

    // 获取SPI事务。
    spi_transaction_t *t = this->transaction_deque.front();
    this->transaction_deque.pop_front();

    // 初始化SPI事务
    std::memset(t, 0, sizeof(*t));
    t->length = 8;           // 1 byte = 8 bits
    t->tx_buffer = &command; // 发送缓冲区指向命令字节
    t->user = this;          // 添加user字段确保回调函数正常工作
    //std::printf("SPI transferring ready.\n");

    // 执行SPI传输
    ret = spi_device_polling_transmit(this->_spi, t);
    if (ret != ESP_OK) {
        std::printf("SPI command error: %d\n", ret);
    }
    this->transaction_deque.push_back(t);
}

// 写入数据，或者说是命令的操作数。
void ST7789_Basic::writeData(const uint8_t data[], size_t len) {
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        std::printf("SPI device not initialized!\n");
        return;
    }

    // 检查参数
    if (data == nullptr || len == 0) {
        std::printf("Invalid data or length\n");
        return;
    }

    // 设置DC引脚为数据模式
    esp_err_t ret = gpio_set_level((gpio_num_t)this->_dc, 1);
    if (ret != ESP_OK) {
        std::printf("Failed to set DC pin level: %d\n", ret);
        return;
    }
    
    // 添加一个小延迟确保GPIO电平稳定
    esp_rom_delay_us(1);

    // 配置SPI事务
    spi_transaction_t *t = this->transaction_deque.front();
    this->transaction_deque.pop_front();

    // 初始化 SPI 事务
    t->flags = 0;            // 默认无特殊标志
    t->length = 8 * len;     // 总发送bit数
    t->tx_buffer = data;     // 数据指针，直接指向传入的数据数组
    t->user = this;          // 添加user字段确保回调函数正常工作

    // 执行SPI传输
    ret = spi_device_polling_transmit(this->_spi, t);
    if (ret != ESP_OK) {
        std::printf("SPI data transmission error: %d\n", ret);
    }

    this->transaction_deque.push_back(t);
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

} // namespace Luna