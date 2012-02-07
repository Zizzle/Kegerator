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
/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Library includes. */
#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"

/* Demo app includes. */
#include "stm3210e_lcd.h"
#include "LCD_Message.h"

#include "integer.h"

//#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "flash.h"
#include "comtest2.h"
#include "leds.h"
/*-----------------------------------------------------------*/

/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY		( ( portTickType ) 5000 / portTICK_RATE_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )

//#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

#define mainCREATOR_TASK_PRIORITY       ( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainCOM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainINTEGER_TASK_PRIORITY       ( tskIDLE_PRIORITY )

/* The maximum number of message that can be waiting for display at any one
time. */
#define mainLCD_QUEUE_SIZE					( 3 )
#define mainTP_QUEUE_SIZE					( 100 )
#define mainMESSAGE_QUEUE_SIZE                                  ( 30 )
/* The check task uses the sprintf function so requires a little more stack. */
#define mainCHECK_TASK_STACK_SIZE	( configMINIMAL_STACK_SIZE + 1000 )
#define mainLCD_TASK_STACK_SIZE		( configMINIMAL_STACK_SIZE + 1000 )

/* Dimensions the buffer into which the jitter time is written. */
#define mainMAX_MSG_LEN						25

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK ( ( unsigned portLONG ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )

/* Constants related to the LCD. */
#define mainMAX_LINE		( 240 )
#define mainROW_INCREMENT	( 24 )
#define mainMAX_COLUMN		( 20 )
#define mainCOLUMN_START	( 319 )
#define mainCOLUMN_INCREMENT 	( 16 )

/* Baud rate used by the comtest tasks. */
#define mainCOM_TEST_BAUD_RATE		( 115200 )

/* The LED used by the comtest tasks. See the comtest.c file for more
information. */
#define mainCOM_TEST_LED	( 3 )

/*-----------------------------------------------------------*/

/**
 * Configure the hardware for the demo.
 */
static void prvSetupHardware( void );

/**
 * The LCD is written two by more than one task so is controlled by a
 * 'gatekeeper' task.  This is the only task that is actually permitted to
 * access the LCD directly.  Other tasks wanting to display a message send
 * the message to the gatekeeper.
 */
static void vLCDTask( void *pvParameters );

/**
 * Checks the status of all the demo tasks then prints a message to the
 * display.  The message will be either PASS - and include in brackets the
 * maximum measured jitter time (as described at the to of the file), or a
 * message that describes which of the standard demo tasks an error has been
 * discovered in.
 *
 * Messages are not written directly to the terminal, but passed to vLCDTask
 * via a queue.
 */
static void vCheckTask( void *pvParameters );

/**
 * Configures the timers and interrupts for the fast interrupt test as
 * described at the top of this file.
 */
extern void vSetupTimerTest( void );

/**
 * External dependence needed by printf implementation. Write a character to standard out.
 *
 * @param c Specifies the character to be written.
 * @return Returns the character written. No error conditions are managed.
 */
//int putChar( int ch );
unsigned long ulIdleCycleCount = 0UL;
static unsigned int uiXPos = 0;
static unsigned int uiYPos = 0;

void vTerminalMessagesTask( void *pvParameters );
void vStackOverflowCheckTask( void *pvParameters );
void vTouchTask( void *pvParameters );
/*-----------------------------------------------------------*/

/* The queue used to send messages to the LCD task. */
xQueueHandle xLCDQueue, xTPQueue, xMessageQueue;

char messageBuf[0xFF];
// Needed by file core_cm3.h
volatile int ITM_RxBuffer;

typedef struct 
{
    unsigned int uiX;
    unsigned int uiY;
} TP_PosData;

TP_PosData TP_PD;

/*-----------------------------------------------------------*/

 
int main( void )
{
#ifdef DEBUG
    debug();
#endif
    

    xTPQueue = xQueueCreate( mainTP_QUEUE_SIZE, sizeof( TP_PosData ) );
    xMessageQueue = xQueueCreate( mainMESSAGE_QUEUE_SIZE, sizeof( messageBuf ) );
    prvSetupHardware(); 
   
    
    /* Create the queue used by the LCD task.  Messages for display on the LCD
       are received via this queue. */
    //xLCDQueue = xQueueCreate( mainLCD_QUEUE_SIZE, sizeof( TP_PosData ) );

    xSerialPortInitMinimal( 9600, 255 );       
    
        
    //vStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
   

	/* Start the tasks defined within this file/specific to this demo. */

   
    xTaskCreate( vLCDTask, 
                 ( signed portCHAR * ) "LCD", 
                 mainLCD_TASK_STACK_SIZE, 
                 NULL, 
                 tskIDLE_PRIORITY+6, 
                 NULL );
    
    /* xTaskCreate( vStackOverflowCheckTask, 
                 ( signed portCHAR * ) "Stack Over1", 
                 configMINIMAL_STACK_SIZE + 500, 
                 NULL, 
                 tskIDLE_PRIORITY + 1,
                 NULL );

    */
    xTaskCreate( vTouchTask, 
                 ( signed portCHAR * ) "touch", 
                 configMINIMAL_STACK_SIZE + 1000, 
                 NULL, 
                 tskIDLE_PRIORITY+2,
                 NULL );
    /*
     xTaskCreate( vTerminalMessagesTask, 
                 ( signed portCHAR * ) "terminal", 
                 configMINIMAL_STACK_SIZE + 1000, 
                 NULL, 
                 tskIDLE_PRIORITY,
                 NULL );
    */           
	/* The suicide tasks must be created last as they need to know how many
	tasks were running prior to their creation in order to ascertain whether
	or not the correct/expected number of tasks are running at any given time. */
//        vCreateSuicidalTasks( mainCREATOR_TASK_PRIORITY );

	/* Configure the timers used by the fast interrupt timer test. */
	vSetupTimerTest();

	/* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task. */
	return 0;
}
/*-----------------------------------------------------------*/


void vLCDTask( void *pvParameters )
{
   
    int ii = 0;
    int cols[] = {Red, Grey, Green, Blue, Magenta, Green, Cyan};
    portTickType xLastExecutionTime = xTaskGetTickCount();
    TP_PosData rxStruct;
    portBASE_TYPE xStatus;
//    char txBuf[50];

    /* Initialise the LCD and display a startup message. */
    vTaskSuspendAll();
    lcd_Initializtion();        
    xTaskResumeAll();
    
    vTaskPrioritySet(NULL, tskIDLE_PRIORITY+2);
    
    for( ;; )
    {
        if ( uxQueueMessagesWaiting ( xTPQueue ) > 1)
        {
            xStatus = xQueueReceive( xTPQueue, &rxStruct, 0);
            printf("LCD Task is running\r\n");
            // lcd_PutString(30, 100, "This is a TEST\0", Black, cols[ii]);
        }
        else         
            taskYIELD();
        

        lcd_DrawHLine(rxStruct.uiX, rxStruct.uiX, Black, rxStruct.uiY);
        
        // sprintf(txBuf, "LCD HWM = %d\r\n", uxTaskGetStackHighWaterMark(NULL));     
        //xQueueSendToBack(xMessageQueue, &txBuf, 0);
        
        //lcd_DrawCircle(50, 200,1 50);
        // lcd_PutString(10, 200, buf, White, Black);
        ii++;
        if (ii >= 6) ii = 0;       
      

   }   
   
}

void vTerminalMessagesTask( void *pvParameters )
{
    char rxBuf[256];
    portBASE_TYPE xStatus;
    //printf("asdfads");
    //vTaskDelay(500);
    for(;;)
    {
        //if ( uxQueueMessagesWaiting ( xMessageQueue ) )
        //    xStatus = xQueueReceive( xMessageQueue, rxBuf, 0);
        //if(rxBuf[0])
        {
            //   vTaskSuspendAll();
           
            // printf(rxBuf);
            //  xTaskResumeAll();
        }
        taskYIELD();
    }
}
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

        // vLEDInit();

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



void vStackOverflowCheckTask( void *pvParameters ) 
{
    size_t heap,heap1;
    
    portTickType xLastExecutionTime = xTaskGetTickCount();
    for (;;)
    {
        
        vTaskDelayUntil(&xLastExecutionTime, 1000/portTICK_RATE_MS );
        //heap1 =  xPortGetFreeHeapSize();
        heap =  xPortGetFreeHeapSize();

        printf("Check HWM = %d\r\n", uxTaskGetStackHighWaterMark(NULL));
        printf("HeapRemaining = %d\r\n", heap);

        //printf("HeapRemaining = %d\r\n", heap1);
        taskYIELD();
         
    }
}

void vTouchTask( void *pvParameters ) 
{
    //vTaskSuspendAll();
    Touch_Initializtion();
    //xTaskResumeAll();
    portTickType xLastExecutionTime = xTaskGetTickCount();
    portTickType xTicksToWait = 100/portTICK_RATE_MS;
    //char txBuf[50];
    unsigned int x = 0, y = 0;
    TP_PD.uiX = 0;
    TP_PD.uiY = 0;
        
        
    portBASE_TYPE xStatus;
    for (;;)
    {
        // printf("Touchchecked %d, %d\r\n", TP_PD.uiX, TP_PD.uiY);        
        //printf("Touch HWM = %d\r\n", uxTaskGetStackHighWaterMark(NULL));  
        //vTaskDelayUntil(&xLastExecutionTime, 50/portTICK_RATE_MS );
        
        x = Touch_MeasurementX();
        y = Touch_MeasurementY();
        TP_PD.uiX = x;
        TP_PD.uiY = y;
        
        
        if ((x|y)!=0)
        {
            xStatus = xQueueSendToBack( xTPQueue, &TP_PD, 0);  
            //sprintf(txBuf, "Tuuch HWM = %d\r\n", uxTaskGetStackHighWaterMark(NULL));
            //xStatus = xQueueSendToBack( xMessageQueue, &txBuf, 0);  
      
            printf("Touchchecked %d, %d\r\n", TP_PD.uiX, TP_PD.uiY);
        }

        taskYIELD();
    }
}

   


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


