#include <cstdint>
#include <stdio.h>
#include <cmath>

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
// 添加ets_delay_us函数的头文件
#include "esp_rom_sys.h"

// 头文件用的。
#include "./st7789_basic.hpp"
// 导入我的字体库。
#include "./font_luna.h"

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
    MADCTL,     0x08,       // MADCTL设置为BGR
    PORCH_CTRL, 0x08, 0x08, 0x08, 0x33, 0x33, // 前后廊设置
    GATE_CTRL,  0x11,       // 门控设置
    VCOMS_CTRL, 0x35,       // VCOM设置
    INVON,                  // 反色
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
    printf("| Hello, ST7789!\n");

    // 分配数组。
    this->front_buffer = (uint16_t*)heap_caps_malloc(this->_totalpx * sizeof(uint16_t), MALLOC_CAP_DMA);
    this->signal = xSemaphoreCreateBinary();
    
    // 检查内存分配是否成功
    if (this->front_buffer == nullptr || this->signal == nullptr) {
        printf("ERROR: Failed to allocate memory for screen buffers or semaphore\n");
        // 如果分配失败，释放已分配的资源
        if (this->front_buffer) {
            heap_caps_free(this->front_buffer);
            this->front_buffer = nullptr;
        }
        if (this->signal) {
            vSemaphoreDelete(this->signal);
            this->signal = nullptr;
        }
        return;
    }

    printf("| Memory allocated.\n");
    // 初始化为全黑。用memset也不影响，对吧 ;)
    memset(this->front_buffer, 0, this->_totalpx * sizeof(uint16_t));

    this->initSPI();
    
    // 检查SPI初始化是否成功
    if (this->_spi == nullptr) {
        printf("ERROR: SPI initialization failed\n");
        return;
    }
}

// 初始化函数
esp_err_t ST7789_Basic::begin() {
    // 配置引脚 
    printf("Initializing ST7789...\n");

    // 检查SPI是否已初始化
    if (this->_spi == nullptr) {
        printf("ERROR: SPI device not initialized. Call initSPI() first.\n");
        return ESP_FAIL;
    }

    // 设置背光、片选和硬件重置端口为OUTPUT模式。
    esp_err_t ret;
    ret = gpio_set_direction((gpio_num_t)this->_rst, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        printf("Failed to set RST pin direction: %d\n", ret);
        return ret;
    }
    
    ret = gpio_set_direction((gpio_num_t)this->_dc, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        printf("Failed to set DC pin direction: %d\n", ret);
        return ret;
    }
    
    ret = gpio_set_direction((gpio_num_t)this->_bl, GPIO_MODE_OUTPUT);
    if (ret != ESP_OK) {
        printf("Failed to set BL pin direction: %d\n", ret);
        return ret;
    }

    // 拉高硬件复位，复位屏幕。
    printf("Resetting screen...\n");
    ret = gpio_set_level((gpio_num_t)this->_rst, 1);
    if (ret != ESP_OK) {
        printf("Failed to set RST pin level high: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
    
    ret = gpio_set_level((gpio_num_t)this->_rst, 0);
    if (ret != ESP_OK) {
        printf("Failed to set RST pin level low: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(250));
    
    ret = gpio_set_level((gpio_num_t)this->_rst, 1);
    if (ret != ESP_OK) {
        printf("Failed to set RST pin level high: %d\n", ret);
        return ret;
    }
    
    vTaskDelay(pdMS_TO_TICKS(250));


    printf("Executing init commands...\n");
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
    printf(" | Initialization command running successfully.\n");

    // 打开背光
    ret = gpio_set_level((gpio_num_t)this->_bl, 1);
    if (ret != ESP_OK) {
        printf("Failed to set BL pin level: %d\n", ret);
        return ret;
    }

    // 然后重置屏幕。
    this->fillScreen(0x0000);
    this->flush();

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
    // 仅写入全局缓冲区，实际显示需调用 flush()
    this->front_buffer[y * this->_width + x] = color;
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
}

// 填充屏幕
void ST7789_Basic::fillScreen(uint16_t color) {
    // 将缓冲区全部填写为 color
    std::fill(this->front_buffer, this->front_buffer + this->_totalpx, color);
}

// 将缓冲区内容推到显示器，需要手动调用
void ST7789_Basic::flush() {
    if (!this->signal) {
        printf("ERROR: Signal semaphore not initialized\n");
        return;  // 防止未初始化情况
    }

    // 等待上一次 DMA 完成
    if (xSemaphoreTake(signal, pdMS_TO_TICKS(1000)) != pdTRUE) {
        printf("ERROR: Timeout waiting for previous DMA transfer\n");
        return;
    }

    // 设置写入窗口（整屏）
    setWindow(0, 0, _width-1, _height-1);

    // 创建 SPI 事务
    memset(&this->transaction_dma, 0, sizeof(this->transaction_dma));
    this->transaction_dma.length = _totalpx * 16;       // bit
    this->transaction_dma.tx_buffer = front_buffer;     // 直接使用front_buffer
    // 传递this指针以便在回调函数中使用
    this->transaction_dma.user = this;
    this->transaction_dma.flags = 0;

    // 开启 DMA 传输
    esp_err_t ret = spi_device_queue_trans(_spi, &this->transaction_dma, portMAX_DELAY);
    if (ret != ESP_OK) {
        printf("ERROR: Failed to queue SPI transaction: %d\n", ret);
        // 如果发送失败，释放信号量避免死锁
        xSemaphoreGive(this->signal);
    }
}

// 析构函数，释放缓冲区
ST7789_Basic::~ST7789_Basic() {
    // 清理buffer
    if (this->front_buffer) {
        delete[] this->front_buffer;
        this->front_buffer = nullptr;
    }

    // 删除信号量。
    if (this->signal) {
        vSemaphoreDelete(this->signal);
    }
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
    this->_buscfg.max_transfer_sz = this->_totalpx * 2;    // 最大传输数
    // 移除可能导致问题的中断标志和CPU亲和性设置
    this->_buscfg.intr_flags = 0;

    esp_err_t ret;

    // 初始化SPI总线
    ret = spi_bus_initialize(SPI2_HOST, &_buscfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        printf("Failed to initialize SPI bus: %d\n", ret);  
        return;
    }

    printf("Configuring SPI device...\n");
    this->_devcfg.clock_speed_hz = this->_clockSpeed;     // 时钟频率
    this->_devcfg.spics_io_num = this->_cs;               // chip select端口选择
    this->_devcfg.queue_size = 16;                        // 增加指令队列大小
    this->_devcfg.flags = SPI_DEVICE_HALFDUPLEX;          // 半双工模式
    this->_devcfg.mode = 0;                               // SPI模式0
    this->_devcfg.pre_cb = nullptr;
    // 修复lambda表达式问题，使用静态函数替代
    this->_devcfg.post_cb = ST7789_Basic::dmaDoneCallback;

    // 在SPI总线加入设备。
    ret = spi_bus_add_device(SPI2_HOST, &_devcfg, &_spi);

    if (ret != ESP_OK) {
        printf("Failed to add SPI device: %d\n", ret);
        _spi = nullptr; // 确保_spi为nullptr表示初始化失败
        return;
    }
    
    printf("SPI device configured successfully.\n");
}

// 写入指令。
void ST7789_Basic::writeCommand(uint8_t command) {
    //printf("Writing command: 0x%02X\n", command);
    
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        printf("WARNING: SPI device not initialized!\n");
        return;
    }

    // 检查GPIO是否正确设置
    if (this->_dc < 0 || this->_dc >= GPIO_NUM_MAX) {
        printf("ERROR: Invalid DC pin: %d\n", this->_dc);
        return;
    }

    // 设置DC引脚为命令模式
    esp_err_t ret = gpio_set_level((gpio_num_t)this->_dc, 0);  // 拉低数据/命令引脚，进入指令模式
    if (ret != ESP_OK) {
        printf("Failed to set DC pin level: %d\n", ret);
        return;
    }
    
    // 添加一个小延迟确保GPIO电平稳定
    esp_rom_delay_us(1);
    
    // 初始化SPI事务
    this->transaction_dma.length = 8;           // 1 byte = 8 bits
    this->transaction_dma.tx_buffer = &command; // 发送缓冲区指向命令字节
    this->transaction_dma.user = this;          // 添加user字段确保回调函数正常工作
    //printf("SPI transferring ready.\n");

    // 执行SPI传输
    ret = spi_device_polling_transmit(this->_spi, &this->transaction_dma);
    if (ret != ESP_OK) {
        printf("SPI command error: %d\n", ret);
    }
}

// 写入数据。
void ST7789_Basic::writeData(const uint8_t data[], size_t len) {
    // 检查SPI设备是否已正确初始化
    if (this->_spi == nullptr) {
        printf("SPI device not initialized!\n");
        return;
    }

    // 检查参数
    if (data == nullptr || len == 0) {
        printf("Invalid data or length\n");
        return;
    }

    // 调试信息。
    // printf("Writing data: ");
    // for (size_t i = 0; i < len; i++) {
    //     printf("%02X ", data[i]);
    // }
    // printf("\n");

    // 设置DC引脚为数据模式
    esp_err_t ret = gpio_set_level((gpio_num_t)this->_dc, 1);
    if (ret != ESP_OK) {
        printf("Failed to set DC pin level: %d\n", ret);
        return;
    }
    
    // 添加一个小延迟确保GPIO电平稳定
    esp_rom_delay_us(1);

    // 初始化 SPI 事务
    this->transaction_dma.flags = 0;            // 默认无特殊标志
    this->transaction_dma.length = 8 * len;     // 总发送bit数
    this->transaction_dma.tx_buffer = data;     // 数据指针，直接指向传入的数据数组
    this->transaction_dma.user = this;          // 添加user字段确保回调函数正常工作

    // 执行SPI传输
    ret = spi_device_polling_transmit(this->_spi, &this->transaction_dma);
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

// =-=-=-=-=-=-=-=-=-=-=-=-= 图形库开始 =-=-=-=-=-=-=-=-=-=-=-=-=

ST7789::ST7789(const ST7789_Config& config) :
    ST7789_Basic(config) {
        // 先直接调用父类构造函数。

    }

void ST7789::printf(const char* arg, ...) {

}

} // namespace Luna