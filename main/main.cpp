#include <stdio.h>
#include <driver/i2c_master.h>
#include <esp_task_wdt.h>

#include "ssd1306_luna/ssd1306_luna.h"
#define feed esp_task_wdt_reset

SSD1306* screen = nullptr;  // 暂时不初始化。

int main() {
    printf("| Entered main().\n");
    screen = new SSD1306();  // 在这里初始化。

    screen->draw_pixel(2, 2, 1);
    screen->draw_pixel(2, 3, 1);
    // screen.print_buffer();
    screen->flush();


    printf("| Program finished.\n");
    return 0;
}

extern "C" void app_main(void) {
    printf("| Hello world!.\n");

    main();  // 主函数。
}