## 说明：
基于i2c_master.h开发，且整个库使用C++，而不是C。

## 协议支持情况：

| 支持协议  | 支持屏幕 | 环境     | 是否支持 |
| -------- | -------- | -------- | -------- |
| I2C_old  | SSD1306  | ESP-IDF  | ×        |
| I2C_new  | SSD1306  | ESP-IDF  | √        |
| SPI      | SSD1306  | ESP-IDF  | ×        |


## 使用例：
请先在使用之前，#include "ssd1306_luna/ssd1306_luna.h" <br>
然后在实例化后使用。<br>
个人的代码习惯是不适用app_main作为主函数，而是另定义一个main函数作为主函数。<br>

```
// C++  main.cpp
#include <stdio.h>
#include <driver/i2c_master.h>
#include "ssd1306_luna/ssd1306_luna.h"

SSD1306_SHAPES* screen = nullptr;  // 暂时不初始化。

int main() {
    printf("| Entered main().\n");
    screen = new SSD1306_SHAPES();  // 在这里初始化。
    screen->draw_rectangle(2, 2, 124, 60, true, false);
    screen->print_buffer();
    screen->flush();
    // 3秒后结束运行。
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    screen->clear_screen_buffer();
    screen->flush();

    printf("| Program finished.\n");
    return 0;
}

extern "C" void app_main(void) {
    printf("| Hello world!.\n");
    main();  // 主函数。
}
```

## 关于图形内容：
所有的图形内容都是基本内容，目前尚未完美实现。

## 注意：
(2025/3/9) 
ESP-IDF目前版本的新版驱动里，异步I2C驱动很有问题。所以目前仅限同步驱动。<br>
（等什么时候这玩意修好了我再做异步）

## 作者：
月と猫 - LunaNeko <br>
CC-BY-SA

最后更新日期：2025/5/27
