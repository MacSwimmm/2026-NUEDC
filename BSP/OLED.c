#include "OLED.h"

#include <string.h>

#include "Board.h"
#include "oledfont.h"
#include "ti_msp_dl_config.h"

#define OLED_COMMAND 0U
#define OLED_DATA    1U

/* 显存按“列×页”组织，与 SSD1306 页寻址模式的发送顺序一致。 */
uint8_t OLED_GRAM[OLED_WIDTH][OLED_HEIGHT / 8U];

/* 以下四个小函数集中封装 GPIO 极性，避免软件 SPI 主流程散落寄存器操作。 */
static void SetRST(uint8_t high)
{
    if (high != 0U) DL_GPIO_setPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
    else DL_GPIO_clearPins(OLED_RST_PORT, OLED_RST_PIN_RST_PIN);
}

static void SetDC(uint8_t high)
{
    if (high != 0U) DL_GPIO_setPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
    else DL_GPIO_clearPins(OLED_DC_PORT, OLED_DC_PIN_DC_PIN);
}

static void SetClock(uint8_t high)
{
    if (high != 0U) DL_GPIO_setPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
    else DL_GPIO_clearPins(OLED_SCL_PORT, OLED_SCL_PIN_SCL_PIN);
}

static void SetData(uint8_t high)
{
    if (high != 0U) DL_GPIO_setPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
    else DL_GPIO_clearPins(OLED_SDA_PORT, OLED_SDA_PIN_SDA_PIN);
}

/**
 * @brief 通过 PA28/PA31 软件模拟四线 SPI 写一个字节。
 *
 * OLED 没有占用硬件 SPI 外设；DC 选择命令/数据，PB14 提供复位。若模块带 CS，
 * 按 README 将 CS 固定接低电平即可。
 */
static void WriteByte(uint8_t value, uint8_t is_data)
{
    uint8_t i;

    /* MSB 先发；时钟低电平期间准备数据，上升沿由 OLED 采样。 */
    SetDC(is_data);
    for (i = 0U; i < 8U; i++) {
        SetClock(0U);
        SetData((value & 0x80U) != 0U);
        SetClock(1U);
        value <<= 1;
    }
    SetDC(1U);
}

void OLED_Refresh(void)
{
    uint8_t page;
    uint8_t column;

    /* 每页重新设置页地址和列地址，确保整帧刷新不依赖控制器上次游标位置。 */
    for (page = 0U; page < 8U; page++) {
        WriteByte((uint8_t)(0xB0U + page), OLED_COMMAND);
        WriteByte(0x00U, OLED_COMMAND);
        WriteByte(0x10U, OLED_COMMAND);
        for (column = 0U; column < OLED_WIDTH; column++) {
            WriteByte(OLED_GRAM[column][page], OLED_DATA);
        }
    }
}

void OLED_ClearBuffer(void)
{
    memset(OLED_GRAM, 0, sizeof(OLED_GRAM));
}

void OLED_Clear(void)
{
    OLED_ClearBuffer();
    OLED_Refresh();
}

static void DrawPoint(uint8_t x, uint8_t y, uint8_t enabled)
{
    uint8_t page;
    uint8_t bit;

    /* 越界像素直接忽略，使上层文本绘制无需为最后一列增加特殊分支。 */
    if ((x >= OLED_WIDTH) || (y >= OLED_HEIGHT)) return;

    /* 与 SSD1306 的 C0 扫描方向配套，使应用层 y=0 始终表示屏幕顶端。 */
    page = (uint8_t)(7U - (y / 8U));
    bit = (uint8_t)(1U << (7U - (y % 8U)));
    if (enabled != 0U) OLED_GRAM[x][page] |= bit;
    else OLED_GRAM[x][page] &= (uint8_t)(~bit);
}

void OLED_ShowChar(uint8_t x, uint8_t y, char character)
{
    uint8_t glyph_index;
    uint8_t byte_index;
    uint8_t bit_index;
    uint8_t px = x;
    uint8_t py = y;

    if ((character < ' ') || (character > '~')) character = '?';
    glyph_index = (uint8_t)(character - ' ');

    /* 复用 6×12 ASCII 字库，每两个字节表示一列的 12 个像素。 */
    for (byte_index = 0U; byte_index < 12U; byte_index++) {
        uint8_t value = oled_asc2_1206[glyph_index][byte_index];
        for (bit_index = 0U; bit_index < 8U; bit_index++) {
            DrawPoint(px, py, (value & 0x80U) != 0U);
            value <<= 1;
            py++;
            if ((uint8_t)(py - y) >= 12U) {
                py = y;
                px++;
                break;
            }
        }
    }
}

void OLED_ShowString(uint8_t x, uint8_t y, const char *text)
{
    /* 当前字符占用 8 像素水平步长，其中有效字形宽度为 6 像素。 */
    while ((*text != '\0') && (x <= 120U)) {
        OLED_ShowChar(x, y, *text++);
        x = (uint8_t)(x + 8U);
    }
}

void OLED_ShowUnsigned(uint8_t x, uint8_t y, uint32_t value, uint8_t digits)
{
    char buffer[11];
    uint8_t i;

    /* uint32_t 最多 10 位十进制数字，限制长度避免写出本地缓冲。 */
    if (digits > 10U) digits = 10U;
    buffer[digits] = '\0';
    for (i = 0U; i < digits; i++) {
        buffer[digits - 1U - i] = (char)('0' + (value % 10U));
        value /= 10U;
    }
    OLED_ShowString(x, y, buffer);
}

void OLED_Init(void)
{
    /* 硬件复位后等待模块电源和内部电荷泵稳定，再发送初始化命令序列。 */
    SetRST(0U);
    Board_DelayMs(120U);
    SetRST(1U);

    WriteByte(0xAEU, OLED_COMMAND); /* 关闭显示，完成参数配置后再开启 */
    WriteByte(0xD5U, OLED_COMMAND);
    WriteByte(0x50U, OLED_COMMAND);
    WriteByte(0xA8U, OLED_COMMAND);
    WriteByte(0x3FU, OLED_COMMAND); /* 1/64 扫描 */
    WriteByte(0xD3U, OLED_COMMAND);
    WriteByte(0x00U, OLED_COMMAND);
    WriteByte(0x40U, OLED_COMMAND);
    WriteByte(0x8DU, OLED_COMMAND);
    WriteByte(0x14U, OLED_COMMAND); /* 开启内部电荷泵 */
    WriteByte(0x20U, OLED_COMMAND);
    WriteByte(0x02U, OLED_COMMAND); /* 页寻址模式 */
    WriteByte(0xA1U, OLED_COMMAND);
    WriteByte(0xC0U, OLED_COMMAND);
    WriteByte(0xDAU, OLED_COMMAND);
    WriteByte(0x12U, OLED_COMMAND);
    WriteByte(0x81U, OLED_COMMAND);
    WriteByte(0xEFU, OLED_COMMAND); /* 沿用参考工程已验证的 SSD1306 对比度 */
    WriteByte(0xD9U, OLED_COMMAND);
    WriteByte(0xF1U, OLED_COMMAND);
    WriteByte(0xDBU, OLED_COMMAND);
    WriteByte(0x30U, OLED_COMMAND);
    WriteByte(0xA4U, OLED_COMMAND);
    WriteByte(0xA6U, OLED_COMMAND); /* 正常显示，不反色 */
    WriteByte(0xAFU, OLED_COMMAND);
    OLED_Clear();
}
