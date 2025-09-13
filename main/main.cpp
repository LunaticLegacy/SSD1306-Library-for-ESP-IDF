#include <stdio.h>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"


#include "st7789_luna/st7789_basic.hpp"

// 喂狗函数，用于看门狗。
#define feed esp_task_wdt_reset

// --------- ST7789初始化 ---------
// 宏定义
#define ST7789_WIDTH            240
#define ST7789_HEIGHT           240
#define PIN_ST7789_DIN          23
#define PIN_ST7789_CLK          18
#define PIN_ST7789_CS           5
#define PIN_ST7789_DC           4
#define PIN_ST7789_RST          16
#define PIN_ST7789_BL           17
#define ST7789_CLOCK_SPEED_MHZ  5

// 全局实例，在栈上分配的全局实例
Luna::ST7789_Basic screen(Luna::ST7789_Config{
    .width = (int16_t)ST7789_WIDTH, 
    .height = (int16_t)ST7789_HEIGHT, 
    .din = (int16_t)PIN_ST7789_DIN, 
    .clk = (int16_t)PIN_ST7789_CLK, 
    .cs = (int16_t)PIN_ST7789_CS, 
    .dc = (int16_t)PIN_ST7789_DC, 
    .rst = (int16_t)PIN_ST7789_RST, 
    .bl = (int16_t)PIN_ST7789_BL, 
    .spi_clock_hz = ST7789_CLOCK_SPEED_MHZ * 1000000
});

int main() {
    printf("| Entered main().\n");
    // 初始化屏幕信息。
    screen.begin();

    screen.fillScreen(0x0000);      // 清屏
    screen.drawRectangle(0,   0, 40, 40, 0xF800); // 红
    screen.drawRectangle(100, 100, 40, 40, 0x07E0); // 绿
    screen.drawRectangle(200, 200, 40, 40, 0x001F); // 蓝

    return 0;
}

extern "C" void app_main(void) {
    main();
}