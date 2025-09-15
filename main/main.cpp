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
Luna::ST7789 screen(Luna::ST7789_Config{
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
void stressTestAll() {
    const uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const int numColors = sizeof(colors) / sizeof(colors[0]);
    const int rectSize = 10;
    const int max_cycles = 500;
    int64_t cycle_duration = 0.0;

    int64_t total_start = esp_timer_get_time();  // 总开始时间

    for (int cycle = 0; cycle < max_cycles; cycle++) {
        int64_t cycle_start = esp_timer_get_time();  // 每次刷新开始时间

        for (int y = 0; y < screen.getHeight(); y += rectSize) {
            for (int x = 0; x < screen.getWidth(); x += rectSize) {
                int colorIndex = (x / rectSize + y / rectSize + cycle) % numColors;
                screen.drawRectangle(x, y, rectSize, rectSize, colors[colorIndex]);
            }
        }
        screen.drawFrame(100, 80, 40, 100, 0xFFFF, 0x0000, 4);
        screen.moveCursor(88, 108);
        screen.printf("FPS: %.6f\nCycle: %d\nProc: %.4f%%", 
            1000000.0 / cycle_duration, 
            cycle, 
            100.0f * ((float)cycle / (float)max_cycles)
        );

        screen.flush();  // 刷新屏幕

        int64_t cycle_end = esp_timer_get_time();  // 每次刷新结束时间
        cycle_duration = cycle_end - cycle_start;

        // 打印单次刷新耗时和帧率
        printf("Cycle %d complete. Took %lld ms. FPS: %.6f\n",
               cycle, cycle_duration / 1000, 1000000.0 / cycle_duration);
    }

    int64_t total_end = esp_timer_get_time();  // 总结束时间
    int64_t total_duration = total_end - total_start;
    float avg_fps = (float)max_cycles * 1000000.0 / total_duration;

    // 打印总耗时和平均帧率
    printf("Total time: %lld ms. Average FPS: %.6f\n",
           total_duration / 1000, avg_fps);

    screen.fillScreen(0xFFFF);

    screen.drawFrame(30, 30, 40, 200, 0xFFFF, 0x0000, 4);
    screen.moveCursor(38, 38);
    screen.printf("Total time: %lld ms.\n Average FPS: %.6f\n",
        total_duration / 1000, avg_fps
    );

    screen.flush();  // 刷新屏幕
}


void stressTestDirty() {
    const uint16_t colors[] = {0xF800, 0x07E0, 0x001F, 0xFFE0, 0xF81F, 0x07FF, 0xFFFF};
    const int numColors = sizeof(colors) / sizeof(colors[0]);
    const int rectSize = 10;
    const int max_cycles = 500;
    int64_t cycle_duration = 0.0;

    int64_t total_start = esp_timer_get_time();  // 总开始时间

    for (int cycle = 0; cycle < max_cycles; cycle++) {
        int64_t cycle_start = esp_timer_get_time();  // 单次刷新开始时间

        // 随机选择屏幕区域绘制，模拟局部更新
        for (int i = 0; i < 50; i++) {  // 每次只更新50个小块
            int x = (rand() % (screen.getWidth() / rectSize)) * rectSize;
            int y = (rand() % (screen.getHeight() / rectSize)) * rectSize;
            int colorIndex = (x / rectSize + y / rectSize + cycle) % numColors;

            screen.drawRectangle(x, y, rectSize, rectSize, colors[colorIndex]);
        }

        // 显示动态帧信息
        screen.moveCursor(88, 108);
        screen.printf("FPS: %.2f\nCycle: %d\nProc: %.2f%%", 
            1000000.0 / (cycle_duration > 0 ? cycle_duration : 1), 
            cycle, 
            100.0f * ((float)cycle / (float)max_cycles)
        );

        // 仅刷新脏页
        screen.flushDirty();  

        int64_t cycle_end = esp_timer_get_time();  // 单次刷新结束
        cycle_duration = cycle_end - cycle_start;

        // 打印每次刷新耗时
        printf("Cycle %d complete. Took %lld ms. FPS: %.2f\n",
               cycle, cycle_duration / 1000, 1000000.0 / (cycle_duration > 0 ? cycle_duration : 1));
    }

    int64_t total_end = esp_timer_get_time();  // 总结束时间
    int64_t total_duration = total_end - total_start;
    float avg_fps = (float)max_cycles * 1000000.0 / total_duration;

    // 打印总耗时和平均帧率
    printf("Total time: %lld ms. Average FPS: %.2f\n",
           total_duration / 1000, avg_fps);

    // 显示最终统计
    screen.fillScreen(0xFFFF);
    screen.moveCursor(38, 38);
    screen.printf("Total time: %lld ms.\nAverage FPS: %.2f\n",
        total_duration / 1000, avg_fps
    );
    screen.flush();  // 刷新屏幕
}


int main() {
    printf("| Entered main().\n");
    // 初始化屏幕信息。
    screen.begin();
    screen.changeCursorColor(0x07E0);

    stressTestDirty();
    return 0;
}

extern "C" void app_main(void) {
    main();
}