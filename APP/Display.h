#ifndef G3507_BASE_DISPLAY_H
#define G3507_BASE_DISPLAY_H

/* OLED 应用层：组织页面内容，不直接操作 GPIO 或软件 SPI 时序。 */
void Display_Init(void);
void Display_NextPage(void);
void Display_Update(void);

#endif
