/*

/* Standard includes. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"

/*app includes. */
//#include "stm3210e_lcd.h"
#include "LCD_Message.h"
#include "console.h" 
#include "leds.h"
#include "touch.h"
#include "lcd.h"
#include "menu.h" 
#include "speaker.h"
#include "timer.h"
#include "crane.h"
#include "ds1820.h"
#include "serial.h"
/*-----------------------------------------------------------*/

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/*-----------------------------------------------------------*/

/**
 * Configure the hardware for the App.
 */
static void prvSetupHardware( void );

/**
 * Configure the menu structures for the App.
 */

struct menu diag_menu[] =
{
    {"DS1820 Setup",    NULL, ds1820_search_applet, ds1820_search_key}, 
    {"DS1820 temps",    NULL, ds1820_display_temps, NULL},
    {"Diag3",    NULL,     NULL, NULL},
    {"Diag4",    NULL,     NULL, NULL},
    {"Diag5",    NULL,     NULL, NULL},
    {"Diag6",    NULL,     NULL, NULL},
    {NULL, NULL, NULL, NULL}
};

/*
struct menu main_menu[] =
{
    {"Manual Control",  manual_menu,   NULL, NULL},
    {"Diagnostics",     diag_menu,     NULL, NULL},
    {"Main3",       NULL,    NULL, NULL},
    {"Main4",       NULL,    NULL, NULL},
    {"Main5",       NULL,    NULL, NULL},
    {"Main6",       NULL,    NULL, NULL},
    {NULL, NULL, NULL, NULL}
};
*/
/*-----------------------------------------------------------*/

xTaskHandle xLCDTaskHandle, 
    xTouchTaskHandle, 
    xTerminalTaskHandle , 
    xBeepTaskHandle, 
    xTimerSetupHandle,
    xDS1820Handle;


// Needed by file core_cm3.h
volatile int ITM_RxBuffer;

/*-----------------------------------------------------------*/

/**
 * Main.
 */ 
int main( void )
{
#ifdef DEBUG
    debug();
#endif

//__libc_init_array();
    
GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOD, &GPIO_InitStructure );


    prvSetupHardware();// set up peripherals etc 
    USARTInit(USART_PARAMS1);

    SCB->SHCSR |= SCB_SHCSR_BUSFAULTENA;


    lcd_init();          

    speaker_init();

    vCraneInit();

        
    vLEDInit();
        
// SPI_FLASH_Init(); cant use this ATM because of conflict with
    // tft Pins

      
 

    xTaskCreate( vTouchTask, 
                 ( signed portCHAR * ) "touch", 
                 configMINIMAL_STACK_SIZE +1000, 
                 NULL, 
                 tskIDLE_PRIORITY+2,
                 &xTouchTaskHandle );
    
/*
    xTaskCreate( vTerminalMessagesTask, 
                 ( signed portCHAR * ) "term", 
                 configMINIMAL_STACK_SIZE + 1500, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 &xTerminalTaskHandle );

    xTaskCreate( vBeepTask, 
                 ( signed portCHAR * ) "beep", 
                 configMINIMAL_STACK_SIZE, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 &xBeepTaskHandle );

   
    xTaskCreate( vTaskDS1820Convert, 
                 ( signed portCHAR * ) "DS1820", 
                 configMINIMAL_STACK_SIZE + 500, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 &xDS1820Handle );
*/
       
    /* Start the scheduler. */
    vTaskStartScheduler();
    
    printf("FAIL\r\n");

    /* Will only get here if there was insufficient memory to create the idle
       task. */
    return 0;
}


/*-----------------------------------------------------------*/
// HARDWARE SETUP
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
        RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );
        
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
                
}

/*-----------------------------------------------------------*/
// STACK OVERFLOW HOOK -  TURNS ON LED
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




/* Keep the linker happy. */
void assert_failed( unsigned portCHAR* pcFile, unsigned portLONG ulLine )
{
    printf("FAILED %s %d\r\n", pcFile, ulLine);

	for( ;; )
	{
	}
}

