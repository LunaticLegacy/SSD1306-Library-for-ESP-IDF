#ifndef __ST7789_CMD_TABLE_H
#define __ST7789_CMD_TABLE_H

// ==================== 无参数指令（No-Parameter Commands） ====================

#define NOP         0x00    // 无操作（No Operation）
#define SWRESET     0x01    // 软件复位（Software Reset）
#define SLPIN       0x10    // 进入睡眠模式（Sleep In）
#define SLPOUT      0x11    // 退出睡眠模式（Sleep Out）
#define INVOFF      0x20    // 关闭显示反转（Display Inversion Off）
#define INVON       0x21    // 开启显示反转（Display Inversion On）
#define DISPOFF     0x28    // 关闭显示（Display Off）
#define DISPON      0x29    // 开启显示（Display On）
#define NORON       0x13    // 开启正常显示模式（Normal Display Mode On）
#define IDMON       0x39    // 开启空闲模式（Idle Mode On）
#define IDMOFF      0x38    // 关闭空闲模式（Idle Mode Off）
#define TEOFF       0x34    // 关闭撕裂效应信号（Tearing Effect Line Off）
#define TEON        0x35    // 开启撕裂效应信号（Tearing Effect Line On）

// ==================== 有参数指令（Parameter Commands） ====================
// 如下指令在输入后，会读取接下来输入的N个byte，作为参数。

// ---------- MADCTL (0x36): Memory Data Access Control ----------
#define MADCTL      0x36
/**
 * 参数：1字节
 * 控制显示数据访问顺序、颜色顺序等
 * 
 * 位定义：
 * Byte 0:
 *   Bit[0:1]: 保留（通常为0）
 *   Bit[2]: MH - 水平刷新顺序（0=左→右，1=右→左）
 *   Bit[3]: RGB/BGR顺序（0=RGB，1=BGR）
 *   Bit[4]: ML - 行列交换（0=正常，1=交换行列）
 *   Bit[5]: MV - 页行顺序（0=正常，1=反转）
 *   Bit[6]: MX - X方向地址顺序（0=左→右，1=右→左）
 *   Bit[7]: MY - YT方向地址顺序（0=上→下，1=下→上）
 */

// ---------- COLMOD (0x3A): Interface Pixel Format ----------
#define COLMOD      0x3A
/**
 * 参数：1字节
 * 设置接口像素格式（颜色深度）
 * 
 * 位定义：
 * Byte 0:
 *   Bit[0:2]: 颜色模式
 *       0b011: 12-bit/pixel (RGB444)
 *       0b101: 16-bit/pixel (RGB565)
 *       0b110: 18-bit/pixel (RGB666)
 *       0b111: 24-bit/pixel (RGB888)
 *   Bit[3]: 保留（通常为0）
 *   Bit[4:6]: 写入颜色模式时使用
 *       0b101 = 16-bit/pixel (RGB565)
 *       0b110 = 18-bit/pixel (RGB666)
 *   Bit[7]: 保留（通常为0）
 */
// 定义常用像素格式参数  0x0 101 0 110

// ---------- CASET (0x2A): Column Address Set ----------
#define CASET       0x2A
/**
 * 参数：4字节（2个uint16_t）
 * 设置列地址范围（起始列和结束列）
 * 
 * 参数顺序：
 *   Byte1: 起始列高8位
 *   Byte2: 起始列低8位
 *   Byte3: 结束列高8位
 *   Byte4: 结束列低8位
 */

// ---------- RASET (0x2B): Row Address Set ----------
#define RASET       0x2B
/**
 * 参数：4字节（2个uint16_t）
 * 设置行地址范围（起始行和结束行）
 * 
 * 参数顺序：
 *   Byte1: 起始行高8位
 *   Byte2: 起始行低8位
 *   Byte3: 结束行高8位
 *   Byte4: 结束行低8位
 */

// ---------- 2C: Memory Write ----------
#define RAMWR       0x2C
#define DRAW        RAMWR    // 别名：用于表示绘制操作
/**
 * 参数：可变长度（可支持N个uint8_t）
 * 向LCD显存内写入像素数据，显示区域由CASET(0x2A)及RASET(0x2B)指定矩形范围。
 * 示例：连续写入 RGB565 或 RGB666 像素数据
 * 
 * 含义：
 * - 发送此命令后，MCU可以连续写入像素数据到LCD显存。
 * - 每个像素的字节数取决于COLMOD预先设置（见COLMOD）：
 *     RGB565: 2 字节/像素
 *     RGB666: 3 字节/像素
 * - 数据顺序：
 *     - 默认按行扫描，从左上角开始
 *     - 每个像素的RGB数据连续（格式：BIP，Band Interleaved by Pixel）
 * - 刷新区域：
 *     - 由CASET（列地址）和RASET（行地址）指定
 *     - 写入开始时，列寄存器和页寄存器会重置到起始位置
 * - 命令执行期间，发送其他命令可以中断写入
 *     - 所以在实际使用该命令时，建议将其作为协程，分段写入
 * 
 * 行为说明：
 * - 将 MCU 数据写入显存，从而在屏幕上显示
 * - 可连续写整个屏幕，也可以分块写（例如分行或分列）
 * - MADCTL 设置会影响起始点和扫描方向
 * 
 * 适用模式：
 * - Normal Mode On / Idle Mode Off / Sleep Out → 可用
 * - Normal Mode On / Idle Mode On / Sleep Out → 可用
 * - Partial Mode On / Idle Mode Off / Sleep Out → 可用
 * - Partial Mode On / Idle Mode On / Sleep Out → 可用
 * - Sleep In → 可用
 * 
 */


// ---------- B2: Porch Setting ----------
#define PORCH_CTRL      0xB2
#define PORCTRL         PORCH_CTRL   // 别名：手册原名
/**
 * 参数：5字节
 * 控制前廊和后廊时间
 * 示例值: [0x0C, 0x0C, 0x00, 0x33, 0x33]
 * 
 * 含义：
 * Byte 0 (Normal mode back porch):
 *   Bit[0:6]   BPA[6:0] - 正常模式下后廊设置（最小0x01）
 * Byte 1 (Normal mode front porch):
 *   Bit[0:6]   FPA[6:0] - 正常模式下前廊设置（最小0x01）
 * Byte 2 (Porch separate enable):
 *   Bit[0]     PSEN - 独立模式开关
 *                 0 = 禁用独立前/后廊控制
 *                 1 = 启用独立前/后廊控制
 *   Bit[1:7]   保留
 * Byte 3 (Idle mode porches):
 *   Bit[0:3]   FPB[3:0] - 空闲模式前廊设置（最小0x01）
 *   Bit[4:7]   BPB[3:0] - 空闲模式后廊设置（最小0x01）
 * Byte 4 (Partial mode porches):
 *   Bit[0:3]   FPC[3:0] - 部分模式前廊设置（最小0x01）
 *   Bit[4:7]   BPC[3:0] - 部分模式后廊设置（最小0x01）
 *
 * 说明：
 * - 前/后廊决定每帧或每行显示前后空闲时间，保证液晶信号同步稳定。
 *     - 可理解为“前摇”和“后摇”。
 * - PSEN = 1 时，可对空闲模式/部分模式单独设置前后廊
 * - 最小值均为0x01，过小可能导致显示异常。
 */


// ---------- B7: Gate Control ----------
#define GATE_CTRL       0xB7
#define GCTRL           GATE_CTRL     // 别名：手册原名
/**
 * 参数：1字节
 * 控制门极驱动电压
 * 示例值: 0x35
 * 
 * 含义：
 * Byte 0:
 *   Bit[0:2]   低电压设置（VGLS）
 *   Bit[3]     保留
 *   Bit[4:6]   高电压设置（VGHS）
 *   Bit[7]     保留
 * 该参数和液晶显示原理相关，其值参考如下（来自手册）：
    VGHS[2:0] VGH (V) 
    00h 12.2 
    01h 12.54 
    02h 12.89 
    03h 13.26 
    04h 13.65 
    05h 14.06 
    06h 14.5 
    07h 14.97 
    
    VGLS[2:0] VGL (V) 
    00h -7.16 
    01h -7.67 
    02h -8.23 
    03h -8.87 
    04h -9.6 
    05h -10.43 
    06h -11.38 
    07h -12.5 
 */

// ---------- BB: VCOMS Setting ----------
#define VCOMS_CTRL  0xBB
/**
 * 参数：1字节
 * 设置VCOMS驱动电压
 * 示例值: 0x19
 * 含义：
 *   0x00 ~ 0xFF 对应不同VCOMS电压
 */

// ---------- C0: LCD Voltage Setting ----------
#define LCD_VCTRL  0xC0
/**
 * 参数：1字节
 * LCD驱动电压（VCI1、VCI2等）
 * 示例值: 0x2C
 * 含义：
 *   对应内部电压放大值或驱动电压调节
 */

// ---------- C2: VDV Setting ----------
#define VDV_CTRL   0xC2
/**
 * 参数：1字节
 * 设置VDV电压（Vcom驱动相关）
 * 示例值: 0x01
 * 含义：
 *   控制内部负电压或驱动信号幅度
 */

// ---------- C3: VRH Setting ----------
#define VRH_CTRL   0xC3
/**
 * 参数：1字节
 * 设置VRH电压（正电压调节）
 * 示例值: 0x12
 * 含义：
 *   控制液晶正向驱动电压
 */

// ---------- C4: Frame Rate Control ----------
#define FR_CTRL    0xC4
/**
 * 参数：1字节
 * 帧率控制
 * 示例值: 0x20
 * 含义：
 *   0x00 ~ 0xFF 对应不同刷新帧率（通常越大刷新越快）
 */

// ---------- C6: Power Control ----------
#define PWR_CTRL   0xC6
/**
 * 参数：1字节
 * 电源控制
 * 示例值: 0x0F
 * 含义：
 *   控制电源电流或电压开启序列
 */

// ---------- D0: Positive/Negative Voltage Setting ----------
#define PVNV_CTRL  0xD0
/**
 * 参数：2字节
 * 设置正负电压（Vop / Vneg）
 * 示例值: 0xA4, 0xA1
 * 含义：
 *   Byte0: 0xA4 - 正电压参数
 *   Byte1: 0xA1 - 负电压参数
 */


#endif // __ST7789_CMD_TABLE_H

/* Troubleshooting Notes (竖条 / 彩色噪点 常见原因与排查)
 * - 颜色/条带通常由以下问题引起：
 *   1) COLMOD 值错误：许多示例使用 0x55 表示 16-bit RGB565，若发送 0x05 或其它值，会导致 MCU 与屏幕对像素字节数/打包方式不一致，产生条带或色点。
 *   2) MADCTL 设置错误：MADCTL 中的 BGR/行列交换位会影响像素顺序与扫描方向，错误设置会导致颜色颠倒或条纹方向异常。
 *   3) SPI 模式 / 时序：模式 (CPOL/CPHA) 或传输速率过高会引起时序错误，表现为噪点或竖条。尝试降低 `clock_speed_hz` 或调整 `mode`。
 *   4) 半双工 / DMA 标志：在 ESP-IDF 中设置 `SPI_DEVICE_HALFDUPLEX`、`SPI_DEVICE_NO_DUMMY` 等标志若不匹配硬件/驱动，可能导致首字节/伪字节丢失或对齐错误。
 *   5) 数据字节顺序：对于 RGB565，高字节先或低字节先的顺序必须一致（你的 `drawPixel` 使用高字节先）；若 COLMOD 与驱动期望不同会出现色带。
 *   6) D/C 引脚切换时机错误：在发送命令/数据时，D/C 引脚必须可靠切换；若 D/C 未切换或延迟不足，会把数据当作命令或反之。
 *   7) 行/列窗口设置（CASET/RASET）错误：窗口设置错误导致写入偏移或重复，形成周期性条带。
 *
 * 调试步骤建议：
 * - 步骤1：把 `COLMOD` 参数改为 `0x55`（已在本头文件将 `COLOR_MODE_16BIT` 定义为 `0x55`），并在 `init_cmds` 中使用该常量。
 * - 步骤2：在 `begin()` 初始化序列中，暂时把 `MADCTL` 设置为 `MADCTL_MX | MADCTL_MY` 或 `0x00` 等几种值测试显示方向与颜色顺序差异。
 * - 步骤3：把 SPI 频率降低到 1 MHz 或 2 MHz 测试，确认是否为时序导致的噪点。
 * - 步骤4：移除或调整 `_devcfg.flags` 的 `SPI_DEVICE_NO_DUMMY` / `SPI_DEVICE_HALFDUPLEX`，尝试不使用这些标志或切换为双工测试。
 * - 步骤5：在写大块像素数据前，先写一个已知简单图案（例如把整个屏幕用单一颜色分块写入），并调试打印写入的数据首 8-16 字节以确认字节顺序。
 * - 步骤6：确认 `DC` 引脚在每次 `writeCommand` / `writeData` 调用前后正确设置，必要时在 `gpio_set_level(_dc, ...)` 之后短延时。
 * - 步骤7：检查 `setWindow()` 使用的坐标无误（避免越界或错位），并在写入像素时确保所写字节数与窗口大小相匹配。
 *
 * 若需要，我可以：
 * - 将 `st7789_basic.cpp` 中的 `init_cmds` 全部替换为使用宏常量（例如 `COLMOD, COLOR_MODE_16BIT` 已示范），并把 `MADCTL` 的初始值替换为可配置的宏；
 * - 在 `writeCommand`/`writeData` 中添加更严格的同步（例如在切换 D/C 后短暂延时）和错误检查；
 * - 提供一个最小测试序列（只设置 COLMOD/CASER/RASET/RAMWR 并填充单色）以便快速定位问题。
 */