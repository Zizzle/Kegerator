///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2007, Matthew Pratt, All Rights Reserved.
//
// Authors: Matthew Pratt
//
// Date:  8 Jun 2012
//
///////////////////////////////////////////////////////////////////////////////
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "lcd.h"
#include "menu.h"
#include "adc.h"
#include "lcd.h"
#include "stm32f10x.h"
#include "settings.h"

unsigned char in_menu = 0;

#define MAX_PINTS 42

#define NR_KEGS 4
unsigned short keg_cal_before[NR_KEGS] = { 0, 0, 0, 0};
unsigned short keg_adc_channel[NR_KEGS] = { 11, 10, 12, 13};
unsigned short keg_ww[NR_KEGS];

static void back_handler(int initializing)
{
    lcd_clear(0);
    in_menu =0;
}

void save_calibrations()
{
	settings_save();
}

void keg1_cal_empty(int init)
{
    g_settings.keg_cal_empty[0] = read_adc(keg_adc_channel[0]);
    save_calibrations();
}
void keg1_cal_before(int init)
{
    keg_cal_before[0] = read_adc(keg_adc_channel[0]);
}
void keg1_cal_after(int init)
{
	if (keg_cal_before[0] == 0) return;
	g_settings.keg_cal_pint[0] = keg_cal_before[0] - read_adc(keg_adc_channel[0]);
    save_calibrations();
}

struct menu calibrate_keg1[] =
{
    {"Keg 1 calibrate empty",        NULL, keg1_cal_empty,  NULL},
    {"Keg 1 calibrate before pint",  NULL, keg1_cal_before, NULL},
    {"Keg 1 calibrate after pint",   NULL, keg1_cal_after,  NULL},
    {"Back",                         NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

void keg2_cal_empty(int init)
{
    g_settings.keg_cal_empty[1] = read_adc(keg_adc_channel[1]);
    save_calibrations();
}
void keg2_cal_before(int init)
{
    keg_cal_before[1] = read_adc(keg_adc_channel[1]);
}
void keg2_cal_after(int init)
{
	if (keg_cal_before[1] == 0) return;
	g_settings.keg_cal_pint[1] = keg_cal_before[1] - read_adc(keg_adc_channel[1]);
    save_calibrations();
}

struct menu calibrate_keg2[] =
{
    {"Keg 2 calibrate empty",   NULL, keg2_cal_empty, NULL},
    {"Keg 2 calibrate before pint",  NULL, keg2_cal_before, NULL},
    {"Keg 2 calibrate after pint",   NULL, keg2_cal_after,  NULL},
    {"Back",                    NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

void keg3_cal_empty(int init)
{
    g_settings.keg_cal_empty[2] = read_adc(keg_adc_channel[2]);
    save_calibrations();
}
void keg3_cal_before(int init)
{
    keg_cal_before[2] = read_adc(keg_adc_channel[2]);
}
void keg3_cal_after(int init)
{
	if (keg_cal_before[2] == 0) return;
	g_settings.keg_cal_pint[2] = keg_cal_before[2] - read_adc(keg_adc_channel[2]);
    save_calibrations();
}

struct menu calibrate_keg3[] =
{
    {"Keg 3 calibrate empty",   NULL, keg3_cal_empty, NULL},
    {"Keg 3 calibrate before pint",  NULL, keg3_cal_before, NULL},
    {"Keg 3 calibrate after pint",   NULL, keg3_cal_after,  NULL},
    {"Back",                    NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};


void keg4_cal_empty(int init)
{
    g_settings.keg_cal_empty[3] = read_adc(keg_adc_channel[3]);
    save_calibrations();
}
void keg4_cal_before(int init)
{
    keg_cal_before[3] = read_adc(keg_adc_channel[3]);
}
void keg4_cal_after(int init)
{
	if (keg_cal_before[3] == 0) return;
	g_settings.keg_cal_pint[3] = keg_cal_before[3] - read_adc(keg_adc_channel[3]);
    save_calibrations();
}

struct menu calibrate_keg4[] =
{
    {"Keg 4 calibrate empty",   NULL, keg4_cal_empty, NULL},
    {"Keg 4 calibrate before pint",  NULL, keg4_cal_before, NULL},
    {"Keg 4 calibrate after pint",   NULL, keg4_cal_after,  NULL},
    {"Back",                    NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

void target_temp_display(int init)
{
	if (init)
	{
		lcd_fill(200, 0, 120, 16, 0);
		lcd_printf(26, 0, 20, "target %d.%d", temp_get_target() / 100, temp_get_target() % 100);	
	}
	else
	{
		settings_save();
	}
}

void temp_plus(int down)
{
	if (down) temp_set_target(temp_get_target() + 50);
	target_temp_display(1);
}

void temp_minus(int down)
{
	if (down && temp_get_target() > 0) temp_set_target(temp_get_target() - 50);
	target_temp_display(1);
}

struct menu set_temp[] =
{
    {"Target + 0.5",   NULL, temp_plus, NULL},
    {"Target - 0.5",   NULL, temp_minus, NULL},
    {"Back",                    NULL, NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

struct menu main_menu[] =
{
    {"Calibrate Keg 1",   calibrate_keg1, NULL, NULL},
    {"Calibrate Keg 2",   calibrate_keg2, NULL, NULL},
    {"Calibrate Keg 3",   calibrate_keg3, NULL, NULL},
    {"Calibrate Keg 4",   calibrate_keg4, NULL, NULL},
    {"Set temperature",   set_temp,       target_temp_display, NULL},
    {"Back",              NULL, back_handler, NULL},
    {NULL, NULL, NULL, NULL}
};

void screen_touch(int xx, int yy)
{
    if (!in_menu)
    {
		in_menu = 1;
		menu_set_root(main_menu);
    }
    else
		menu_touch(xx, yy);
}

void display_keg(int yy, int index)
{
    unsigned adc = read_adc( keg_adc_channel[index]);

    int ww = 0;
    unsigned short empty = g_settings.keg_cal_empty[index];
    unsigned short full =  MAX_PINTS * g_settings.keg_cal_pint[index];

    if (adc > full)
    {
		ww = 320;
    }
    else if (adc > empty)
    {
		adc  -= empty;
		full -= empty;
	
		ww = (LCD_W * adc);
		ww /= full;
    }

#if 0
    int old = keg_ww[index];
    if (ww > old)
	lcd_fill(old, yy, ww - old, 50, 0x9999);
    else
	lcd_fill(ww, yy, old - ww, 50, 0x0);
#else
    lcd_fill(0,  yy, ww, 50, 0x9999);
    lcd_fill(ww, yy, 320-ww, 50, 0x0);    
#endif

    lcd_printf(5, 1+ yy / 16, 0, "%d pints left", 48 * ww / LCD_W);
    //lcd_printf(5,2 + yy / 16, 0, "%d", adc );

}

void vKegTask( void *pvParameters ) 
{
    printf("Keg start\r\n");
    adc_init();

    for( ;; )
    {
		if (!in_menu)
		{
			lcd_lock();
			display_off();
			vTaskDelay(10);

			display_keg(0,   0);
			display_keg(60,  1);
			display_keg(120, 2);
			display_keg(180, 3);
			lcd_printf(29,1, 8, "%d.%dC", ds1820_get_temp() / 100, ds1820_get_temp() % 100);
			display_on();
			lcd_release();
		}
		vTaskDelay( 100 );
    }
}
