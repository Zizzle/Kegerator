//-------------------------------------------------------------------------
// Author: Brad Goold
// Date: 14 Feb 2012
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
// Included Libraries
//-------------------------------------------------------------------------
#include <stdio.h> 
//#include <math.h> 
#include "FreeRTOS.h" 
#include "queue.h"
#include "console.h" 
#include "semphr.h"

//-------------------------------------------------------------------------
xQueueHandle xConsoleQueue;
char messageBuf[0xFF];
xSemaphoreHandle xMutex;

//-------------------------------------------------------------------------

void vTerminalMessagesTask( void *pvParameters )
{
    extern unsigned portSHORT usMaxJitter;    
    char rxBuf[256];
    char tasks[256];
    float DS1820Temp = 0.0;
    xConsoleQueue = xQueueCreate( mainMESSAGE_QUEUE_SIZE, sizeof( messageBuf ) );
    xMutex = xSemaphoreCreateMutex();
    portBASE_TYPE xStatus;
    portTickType xLastExecutionTime = xTaskGetTickCount();    
    unsigned int heap = 0;
    static portTickType xLastIdleCount = 0;
    vTaskDelay(3000); //wait for settling time
    
    for(;;)
    {
       
        heap =  xPortGetFreeHeapSize();
        vTaskList(tasks); // put here outside of the Suspended
                          // Scheduler. 
        xStatus = xQueueReceive( xConsoleQueue, &rxBuf, 1000);
       
//        printf("%.2f\r\n", DS1820Temp); 
        
        if ( xStatus == pdTRUE){
            xSemaphoreTake(xMutex, portMAX_DELAY);     
            printf("\r\nIncoming Message! ----\r\n");
            printf(rxBuf);
            //        printf("last ex time = %u\r\n", xTaskGetTickCount());
            printf("Message Complete! ----\r\n\r\n");
            xSemaphoreGive(xMutex);
        }
       
      
       

    }
    taskYIELD();
}
