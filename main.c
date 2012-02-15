/*
	FreeRTOS.org V5.0.0 - Copyright (C) 2003-2008 Richard Barry.

	This file is part of the FreeRTOS.org distribution.

	FreeRTOS.org is free software; you can redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation; either version 2 of the License, or
	(at your option) any later version.

	FreeRTOS.org is distributed in the hope that it will be useful,
	but WITHOUT ANY WARRANTY; without even the implied warranty of
	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
	GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with FreeRTOS.org; if not, write to the Free Software
	Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

	A special exception to the GPL can be applied should you wish to distribute
	a combined work that includes FreeRTOS.org, without being obliged to provide
	the source code for any proprietary components.  See the licensing section
	of http://www.FreeRTOS.org for full details of how and when the exception
	can be applied.

    ***************************************************************************
    ***************************************************************************
    *                                                                         *
    * SAVE TIME AND MONEY!  We can port FreeRTOS.org to your own hardware,    *
    * and even write all or part of your application on your behalf.          *
    * See http://www.OpenRTOS.com for details of the services we provide to   *
    * expedite your project.                                                  *
    *                                                                         *
    ***************************************************************************
    ***************************************************************************

	Please ensure to read the configuration and relevant port sections of the
	online documentation.

	http://www.FreeRTOS.org - Documentation, latest information, license and
	contact details.

	http://www.SafeRTOS.com - A version that is certified for use in safety
	critical systems.

	http://www.OpenRTOS.com - Commercial support, development, porting,
	licensing and training services.
*/


/*
 * Creates all the demo application tasks, then starts the scheduler.  The WEB
 * documentation provides more details of the standard demo application tasks.
 * In addition to the standard demo tasks, the following tasks and tests are
 * defined and/or created within this file:
 *
 * "Fast Interrupt Test" - A high frequency periodic interrupt is generated
 * using a free running timer to demonstrate the use of the
 * configKERNEL_INTERRUPT_PRIORITY configuration constant.  The interrupt
 * service routine measures the number of processor clocks that occur between
 * each interrupt - and in so doing measures the jitter in the interrupt timing.
 * The maximum measured jitter time is latched in the ulMaxJitter variable, and
 * displayed on the LCD by the 'Check' task as described below.  The
 * fast interrupt is configured and handled in the timertest.c source file.
 *
 * "LCD" task - the LCD task is a 'gatekeeper' task.  It is the only task that
 * is permitted to access the display directly.  Other tasks wishing to write a
 * message to the LCD send the message on a queue to the LCD task instead of
 * accessing the LCD themselves.  The LCD task just blocks on the queue waiting
 * for messages - waking and displaying the messages as they arrive.
 *
 * "Check" task -  This only executes every five seconds but has the highest
 * priority so is guaranteed to get processor time.  Its main function is to
 * check that all the standard demo tasks are still operational.  Should any
 * unexpected behaviour within a demo task be discovered the 'check' task will
 * write an error to the LCD (via the LCD task).  If all the demo tasks are
 * executing with their expected behaviour then the check task writes PASS
 * along with the max jitter time to the LCD (again via the LCD task), as
 * described above.
 */



/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
//#include <math.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"


#include "stm3210e_lcd.h"
#include "LCD_Message.h"
#include "console.h" 
#include "leds.h"
#include "touch.h"
#include "lcd.h"
#include "menu.h" 

/*-----------------------------------------------------------*/

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/*-----------------------------------------------------------*/

/**
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

unsigned long ulIdleCycleCount = 0UL;

struct menu foo2_menu[] =
{
    {"A",    NULL,     NULL, NULL}, 
    {"B",    NULL,     NULL, NULL},
    {"C",    NULL,     NULL, NULL},
    {"D",    NULL,     NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

struct menu foo_menu[] =
{
    {"Text1",    NULL,     NULL, NULL}, 
    {"Text2",    NULL,     NULL, NULL},
    {"Text3",    NULL,     NULL, NULL},
    {"Text4",    foo2_menu,     NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

struct menu bar_menu[] =
{
    {"Test1",    NULL,     NULL, NULL}, 
    {"Test2",    NULL,     NULL, NULL},
    {"Test3",    NULL,     NULL, NULL},
    {"Test4",    NULL,     NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

struct menu main_menu[] =
{
    {"Main1",       NULL,      NULL, NULL},
    {"Main2",       bar_menu,  NULL, NULL},
    {"Main3",       NULL,      NULL, NULL},
    {"Main4",       foo_menu,  NULL, NULL},
    {"Main5",       NULL,      NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

/*-----------------------------------------------------------*/

xTaskHandle xLCDTaskHandle, xTouchTaskHandle, xTerminalTaskHandle;


// Needed by file core_cm3.h
volatile int ITM_RxBuffer;

/*-----------------------------------------------------------*/

 
int main( void )
{
#ifdef DEBUG
    debug();
#endif
    
    prvSetupHardware();// set up peripherals etc 
    vLEDInit();   // set up the LED flash io and tasks
    
    xSerialPortInitMinimal( 9600, 255 );       
    
    
    /* Start the tasks defined within this file/specific to this demo. */
    
    
    xTaskCreate( vLCDTask, 
                 ( signed portCHAR * ) "LCD", 
                 mainLCD_TASK_STACK_SIZE, 
                 NULL, 
                 tskIDLE_PRIORITY+6, 
                 &xLCDTaskHandle );
    
    
    xTaskCreate( vTouchTask, 
                 ( signed portCHAR * ) "touch", 
                 configMINIMAL_STACK_SIZE +1500, 
                 NULL, 
                 tskIDLE_PRIORITY+2,
                 &xTouchTaskHandle );
    
    xTaskCreate( vTerminalMessagesTask, 
                 ( signed portCHAR * ) "term", 
                 configMINIMAL_STACK_SIZE + 1500, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 &xTerminalTaskHandle );
    
    menu_set_root(main_menu);
    
    /* Start the scheduler. */
    vTaskStartScheduler();
    
    /* Will only get here if there was insufficient memory to create the idle
       task. */
    return 0;
}
/*-----------------------------------------------------------*/




/*-----------------------------------------------------------*/



/*-----------------------------------------------------------*/


/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
    /* Start with the clocks in their expected state. */
    RCC_DeInit();
    
    /* Enable HSE (high speed external clock). */
    RCC_HSEConfig( RCC_HSE_ON );
    
    /* Wait till HSE is ready. */
    while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
    {
    }
    
    /* 2 wait states required on the flash. */
    *( ( unsigned portLONG * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = 8MHz * 9 = 72 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div1, RCC_PLLMul_9 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | 
                                RCC_APB2Periph_GPIOB |
                                RCC_APB2Periph_GPIOC | 
                                RCC_APB2Periph_GPIOD | 
                                RCC_APB2Periph_GPIOE | 
                                RCC_APB2Periph_GPIOF | 
                                RCC_APB2Periph_AFIO, 
                                ENABLE );
        
	/* SPI2 Periph clock enable */
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_SPI2, ENABLE );


	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
        
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
                
}

/*-----------------------------------------------------------*/
void vApplicationStackOverflowHook( xTaskHandle *pxTask, signed portCHAR *pcTaskName )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( GPIOC, &GPIO_InitStructure );
    GPIO_WriteBit( GPIOC, GPIO_Pin_7, 1 );
    
    for( ;; );
}

/*-----------------------------------------------------------*/

void vApplicationIdleHook( void ) 
{
    ulIdleCycleCount++;
}


#ifdef DEBUG

/* Keep the linker happy. */
void assert_failed( unsigned portCHAR* pcFile, unsigned portLONG ulLine )
{

	for( ;; )
	{
	}
}

#endif

