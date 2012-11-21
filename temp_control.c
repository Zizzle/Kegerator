#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ds1820.h"
#include "settings.h"

#define MIN_DELAY 600

#define RELAY_PORT GPIOC      // E
#define RELAY_PIN GPIO_Pin_7 // 4

static 	int seconds_since_run;
static int running = 0;

void temp_set_target(int target)
{
	g_settings.target_temp = target;
}

int temp_get_target()
{
	return g_settings.target_temp;
}

static void run_freezer(int on)
{
	if (!on || (!running && seconds_since_run < MIN_DELAY))
	{
		GPIO_ResetBits(RELAY_PORT, RELAY_PIN);	
		running = 0;
		return;
	}

	running = 1;
	seconds_since_run = 0;
	GPIO_SetBits(RELAY_PORT, RELAY_PIN);	
}

void vTaskTempControl( void *pvParameters )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  RELAY_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(RELAY_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(RELAY_PORT, RELAY_PIN);

    for (;;)
    {
		run_freezer(ds1820_get_temp() > temp_get_target());
		seconds_since_run++;
        vTaskDelay(1000); // wait for conversion
    }    
}


