#include <stdio.h>
#include <vector>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_task_wdt.h"


#include "st7789/st7789_basic.hpp"

// 喂狗函数，用于看门狗。
#define feed esp_task_wdt_reset

#define ST7789_WIDTH        240
#define ST7789_HEIGHT       240
#define PIN_ST7789_DIN      23
#define PIN_ST7789_CLK      18
#define PIN_ST7789_CS       5
#define PIN_ST7789_DC       4
#define PIN_ST7789_RST      16
#define PIN_ST7789_BL       17

Luna::ST7789_Basic *screen = new Luna::ST7789_Basic(
    ST7789_WIDTH, ST7789_HEIGHT, 
    PIN_ST7789_DIN, PIN_ST7789_CLK, PIN_ST7789_CS, 
    PIN_ST7789_DC, PIN_ST7789_RST, PIN_ST7789_BL
);

//  内存监视函数
static std::vector<uint32_t> memory_monitor_task() {
    // 获取当前空闲堆内存
    uint32_t free_memory = esp_get_free_heap_size();
    // 获取最小空闲堆内存
    uint32_t min_free_memory = esp_get_minimum_free_heap_size();
    return {free_memory, min_free_memory};
}

int main() {
    printf("| Entered main().\n");


    return 0;
}

extern "C" void app_main(void) {
    main();
}