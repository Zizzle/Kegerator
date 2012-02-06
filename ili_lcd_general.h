#ifndef ILI_LCD_GENERAL_H_INCLUDED
#define ILI_LCD_GENERAL_H_INCLUDED

/*
 Compatible list:
 ili9320 ili9325 ili9328
 LG4531
*/

/* LCD color */
#define White            0xFFFF
#define Black            0x0000
#define Grey             0xF7DE
#define Blue             0x001F
#define Blue2            0x051F
#define Red              0xF800
#define Magenta          0xF81F
#define Green            0x07E0
#define Cyan             0x7FFF
#define Yellow           0xFFE0

/*---------------------- Graphic LCD size definitions ------------------------*/
#define LCD_WIDTH       240                 /* Screen Width (in pixels)           */
#define LCD_HEIGHT      320                 /* Screen Hight (in pixels)           */
#define BPP             16                  /* Bits per pixel                     */
#define BYPP            ((BPP+7)/8)         /* Bytes per pixel                    */

extern void lcd_Initializtion(void);
extern void lcd_SetCursor(unsigned int x,unsigned int y);
extern unsigned int lcd_getdeviceid(void);
void  lcd_PutChar(unsigned int  x,unsigned int y,unsigned char c,unsigned int  charColor,unsigned int bkColor);   
void lcd_PutString(unsigned int x, unsigned int y, unsigned char * s, unsigned int textColor, unsigned int bkColor);
void lcd_DrawHLine(int x1, int x2, int col, int y);
void lcd_DrawVLine(int y1, int y2, int col, int x);
void lcd_DrawRect(int x1, int y1, int x2, int y2, int col);
void lcd_DrawBMP(unsigned portCHAR *Pict);
//#define _ILI_REVERSE_DIRECTION_
//#define use_rt_gui


#if defined(use_rt_gui)
#include "rtthread.h"
#include <rtgui/rtgui.h>
#include <rtgui/driver.h>
#include <rtgui/rtgui_server.h>
#include <rtgui/rtgui_system.h>
extern void rt_hw_lcd_update(rtgui_rect_t *rect);
extern rt_uint8_t * rt_hw_lcd_get_framebuffer(void);
extern void rt_hw_lcd_set_pixel(rtgui_color_t *c, rt_base_t x, rt_base_t y);
extern void rt_hw_lcd_get_pixel(rtgui_color_t *c, rt_base_t x, rt_base_t y);
extern void rt_hw_lcd_draw_hline(rtgui_color_t *c, rt_base_t x1, rt_base_t x2, rt_base_t y);
extern void rt_hw_lcd_draw_vline(rtgui_color_t *c, rt_base_t x, rt_base_t y1, rt_base_t y2);
extern void rt_hw_lcd_draw_raw_hline(rt_uint8_t *pixels, rt_base_t x1, rt_base_t x2, rt_base_t y);
#endif

#endif // ILI_LCD_GENERAL_H_INCLUDED
