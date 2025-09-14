#include <stdio.h>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"

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
#define ST7789_CLOCK_SPEED_MHZ  40

// 全局实例，在栈上分配的全局实例
Luna::ST7789_Basic screen(Luna::ST7789_Config{
    .width = (int16_t)ST7789_WIDTH, 
    .height = (int16_t)ST7789_HEIGHT, 
    .din = (gpio_num_t)PIN_ST7789_DIN, 
    .clk = (gpio_num_t)PIN_ST7789_CLK, 
    .cs = (gpio_num_t)PIN_ST7789_CS, 
    .dc = (gpio_num_t)PIN_ST7789_DC, 
    .rst = (gpio_num_t)PIN_ST7789_RST, 
    .bl = (gpio_num_t)PIN_ST7789_BL, 
    .spi_clock_hz = ST7789_CLOCK_SPEED_MHZ * 1000000
});

// 压测
void stressTest() {
    const uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const int numColors = sizeof(colors) / sizeof(colors[0]);
    const int rectSize = 10;
    const int max_cycles = 50;

    int64_t total_start = esp_timer_get_time();  // 总开始时间

    for (int cycle = 0; cycle < max_cycles; cycle++) {
        int64_t cycle_start = esp_timer_get_time();  // 每次刷新开始时间

        for (int y = 0; y < screen.getHeight(); y += rectSize) {
            for (int x = 0; x < screen.getWidth(); x += rectSize) {
                int colorIndex = (x / rectSize + y / rectSize + cycle) % numColors;
                screen.drawRectangle(x, y, rectSize, rectSize, colors[colorIndex]);
            }
        }

        screen.flush();  // 刷新屏幕

        int64_t cycle_end = esp_timer_get_time();  // 每次刷新结束时间
        int64_t cycle_duration = cycle_end - cycle_start;

        // 打印单次刷新耗时和帧率
        printf("Cycle %d complete. Took %lld ms. FPS: %.2f\n",
               cycle, cycle_duration / 1000, 1000000.0 / cycle_duration);
    }

    int64_t total_end = esp_timer_get_time();  // 总结束时间
    int64_t total_duration = total_end - total_start;
    float avg_fps = (float)max_cycles * 1000000.0 / total_duration;

    // 打印总耗时和平均帧率
    printf("Total time: %lld ms. Average FPS: %.2f\n",
           total_duration / 1000, avg_fps);
}


int main() {
    printf("| Entered main().\n");
    // 初始化屏幕信息。
    screen.begin();

    screen.drawRectangle(0,   0, 40, 40, 0xF800); // 红
    screen.drawRectangle(100, 100, 40, 40, 0x07E0); // 绿
    screen.drawRectangle(200, 200, 40, 40, 0x001F); // 蓝

    screen.flush();

    // 压测
    // stressTest();

    return 0;
}

extern "C" void app_main(void) {
    main();
}