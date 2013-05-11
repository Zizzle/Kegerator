#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ds1820.h"
#include "settings.h"
#include "temp_control.h"

#define MIN_DELAY 600
#define HYSTERESIS 50

#define RELAY_PORT GPIOC      // E
#define RELAY_PIN GPIO_Pin_7 // 4

static int seconds_since_run;
static int run_time = 0;
static int state = 0;

void temp_set_target(int target)
{
	g_settings.target_temp = target;
}

int temp_get_target()
{
	return g_settings.target_temp;
}

int temp_get_state()
{
	return state;
}

static void turn_on()
{
	if (seconds_since_run < MIN_DELAY)
	{
		state = STATE_WAITING;
	}
	else
	{
		state = STATE_ON;
		seconds_since_run = 0;
		GPIO_SetBits(RELAY_PORT, RELAY_PIN);
	}
}

static void run_freezer(uint32_t current, uint32_t target)
{
	switch (state)
	{
		case STATE_OFF:
			GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
			if (current > target + HYSTERESIS)
			{
				turn_on();
			}
			else if (current > target)
			{
				state = STATE_HYSTERESIS;
			}
			break;

		case STATE_WAITING:
			if (current < target + HYSTERESIS)
			{
				state = STATE_OFF;
			}
			else
			{
				turn_on();
			}
			break;

		case STATE_HYSTERESIS:
			if (current <= target) state = STATE_OFF;
			if (current > target + HYSTERESIS)
			{
				turn_on();
			}
			break;

		case STATE_ON:
			run_time++;
			seconds_since_run = 0;
			GPIO_SetBits(RELAY_PORT, RELAY_PIN);
			if (current < target - HYSTERESIS)
			{
				state = STATE_OFF;
				GPIO_ResetBits(RELAY_PORT, RELAY_PIN);
			}
			break;
	}
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
		run_freezer(ds1820_get_temp(), temp_get_target());
		seconds_since_run++;
        vTaskDelay(1000); // wait for conversion
    }    
}


