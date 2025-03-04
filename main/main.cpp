#include <stdio.h>

#include <driver/i2c_master.h>
#include <driver/gpio.h>

#include "ssd1306_luna/ssd1306_luna.h"

SSD1306 screen;

int main() {
    printf("| Entered main().\n");
    screen.draw_pixel(2, 2, 1);
    screen.draw_pixel(2, 3, 1);
    screen.draw_pixel(3, 3, 1);

    screen.flush();
    //screen.print_buffer();
    return 0;    
}

extern "C" void app_main(void) {
    printf("Hello world!\n");
    main();
}