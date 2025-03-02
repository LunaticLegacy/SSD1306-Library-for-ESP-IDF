#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>

#include "u8g2/cppsrc/MUIU8g2.h"

// 定义通信用引脚
#define SCL_PORT 22
#define SDA_PORT 21

// I2C配置
#define I2C_MASTER_SCL_IO SCL_PORT
#define I2C_MASTER_SDA_IO SDA_PORT
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0

// SSD1306配置
#define SSD1306_WIDTH 128
#define SSD1306_HEIGHT 64


void setup_i2c() {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_MASTER_SD A_IO;
    conf.scl_io_num = I2C_MASTER_SCL_IO;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    conf.slave.maximum_speed = I2C_MASTER_FREQ_HZ

    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

extern "C" void app_main(void) {
    printf("Hello world!\n");

    // 初始化I2C    
    setup_i2c();

    // 初始化U8g2库并配置SSD1306
    U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);
    u8g2.begin();
    
    // 显示文本
    u8g2.setFont(u8g2_font_ncenB08_tr); // 设置字体
    u8g2.clearBuffer(); // 清除缓冲区
    u8g2.drawStr(10, 10, "Hello world!");
    u8g2.sendBuffer(); // 发送缓冲区到显示屏

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}