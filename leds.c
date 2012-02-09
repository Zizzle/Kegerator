//-------------------------------------------------------------------------
// Author: Brad Goold
// Date:  7 Feb 2012
// Email Address: W0085400@umail.usq.edu.au
// 
// Purpose:
// Pre:
// Post:
// RCS $Date$
// RCS $Revision$
// RCS $Source$
// RCS $Log$
//-------------------------------------------------------------------------


//-------------------------------------------------------------------------
/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

/* Library includes. */
#include "stm32f10x.h"
#include "leds.h"
#include <stdio.h>

static unsigned portSHORT usOutputValue = 0;

void vStartupLEDTask ( void *pvParameters );
void vLEDFlashTask( void *pvParameters );
 
/*-----------------------------------------------------------*/

void vLEDInit( void )
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    //D1 = PC6, D2 = PC7 , D3 = PD13, D4 = PD6
    GPIO_InitStructure.GPIO_Pin =  D1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( D1PORT, &GPIO_InitStructure );
    
    GPIO_InitStructure.GPIO_Pin =  D2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( D2PORT, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin =  D3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( D3PORT, &GPIO_InitStructure );

    GPIO_InitStructure.GPIO_Pin =  D4;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init( D3PORT, &GPIO_InitStructure );
    
    xTaskCreate( vStartupLEDTask, 
                 ( signed portCHAR * ) "LED",
                 configMINIMAL_STACK_SIZE + 100,
                 NULL,
                 tskIDLE_PRIORITY+5,
                 NULL);
    
    
}

void vStartupLEDTask ( void *pvParameters ) {
    vLEDSet(D1PORT, D1, 1);
    vLEDSet(D2PORT, D2, 1);
    vLEDSet(D3PORT, D3, 1);
    vLEDSet(D4PORT, D4, 1);
    
    vTaskDelay(500/portTICK_RATE_MS);
    
    vLEDSet(D1PORT, D1, 0);
    vLEDSet(D2PORT, D2, 0);
    vLEDSet(D3PORT, D3, 0);
    vLEDSet(D4PORT, D4, 0);
    
    xTaskCreate( vLEDFlashTask, 
                 ( signed portCHAR * ) "LED Flash",
                 configMINIMAL_STACK_SIZE + 400,
                 NULL,
                 tskIDLE_PRIORITY+2,
                 NULL);
    // printf("LEDStartup HWM = %d1\r\n", uxTaskGetStackHighWaterMark(NULL));
    
        
    vTaskDelete(NULL);
    
    for (;;)
    {
        //Should never get here
        vTaskPrioritySet(NULL, tskIDLE_PRIORITY);
        // printf("LcdStartup Still Running\r\n");
    }
}

void vLEDFlashTask( void *pvParameters )
{
    portTickType xLastExecutionTime = xTaskGetTickCount();
    for (;;)
    {
        vTaskDelayUntil( &xLastExecutionTime, 200/portTICK_RATE_MS ); 
        vLEDToggle( D4PORT, D4 );
        //    printf("LedFlash HWM = %d\r\n", uxTaskGetStackHighWaterMark(NULL));
        taskYIELD();
    }
    
}

//-------------------------------------------------------------------------

void vLEDSet( GPIO_TypeDef *GPIO_PORT , uint16_t GPIO_Pin, unsigned portBASE_TYPE uxValue)
{
    vTaskSuspendAll();
    
    if (uxValue)
        GPIO_WriteBit( GPIO_PORT, GPIO_Pin, 1 );       
    
    else 
        GPIO_WriteBit( GPIO_PORT, GPIO_Pin, 0 );
    
    
    xTaskResumeAll();
}

void vLEDToggle( GPIO_TypeDef *GPIO_PORT , uint16_t GPIO_Pin)
{
    uint16_t uxVal = GPIO_ReadOutputDataBit(GPIO_PORT,GPIO_Pin);
   
    if (uxVal) 
        vLEDSet(GPIO_PORT,GPIO_Pin, 0);
    else
        vLEDSet(GPIO_PORT,GPIO_Pin, 1);
   
}
