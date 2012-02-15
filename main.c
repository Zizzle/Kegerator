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
#include "stm3210e_lcd.h"
#include "LCD_Message.h"
#include "console.h" 
#include "leds.h"
#include "touch.h"
#include "lcd.h"
#include "menu.h" 
#include "speaker.h"
/*-----------------------------------------------------------*/

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/*-----------------------------------------------------------*/

/**
 * Configure the hardware for the App.
 */
static void prvSetupHardware( void );

unsigned long ulIdleCycleCount = 0UL;


/**
 * Configure the menu structures for the App.
 */
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

xTaskHandle xLCDTaskHandle, 
    xTouchTaskHandle, 
    xTerminalTaskHandle , 
    xBeepTaskHandle;


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
    
    prvSetupHardware();// set up peripherals etc 

    vLEDInit();   // set up the LED flash io and tasks
    
    xSerialPortInitMinimal( 9600, 255 );       
    
    speaker_init();
    
    /* Start the tasks defined within this file/specific to this demo. */
    
    //LCD Task starts at high priority, then drops
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

    xTaskCreate( vBeepTask, 
                 ( signed portCHAR * ) "beep", 
                 configMINIMAL_STACK_SIZE, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 &xBeepTaskHandle );
    
    menu_set_root(main_menu);
    
    /* Start the scheduler. */
    vTaskStartScheduler();
    
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

