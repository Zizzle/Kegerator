//#include "stm32f10x_lib.h"
//#include "ili9320.h"
#include <stdio.h>

#include <stdint.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"

#include "queue.h"

#include "touch.h"
#include "task.h"
#include "lcd.h"
#include "menu.h"
#include "console.h"
#include "speaker.h"
#include "timer.h"

#define CH_X  0xd0//0x90
#define CH_Y  0x90//0xd0

extern xQueueHandle xTPQueue;

void SPI_CS(u8 a)
{
  // PD6 -> TS_nC
  if (a)
    GPIO_SetBits(GPIOB,GPIO_Pin_7);
  else
    GPIO_ResetBits(GPIOB,GPIO_Pin_7);
}

void SPI_DIN(u8 a)
{
  // PD7 -> TS_DIN
  if (a)
    GPIO_SetBits(GPIOA,GPIO_Pin_7);
  else
    GPIO_ResetBits(GPIOA,GPIO_Pin_7);
}

void SPI_CLK(u8 a)
{
  // PD5 -> TS_CLK
  if (a)
    GPIO_SetBits(GPIOA,GPIO_Pin_5);
  else
    GPIO_ResetBits(GPIOA,GPIO_Pin_5);
}

u8 SPI_DOUT(void)
{
  // PD10 -> TS_DOUT
  return GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6);
}

void SPI_delay(u16 i)
{
    u16 k;
    for (k=0;k<i;k++);

}

void Touch_Start(void)
{
	SPI_CLK(0);
	SPI_CS(1);
	SPI_DIN(1);
	SPI_CLK(1);
	SPI_CS(0);
      
}

void Touch_Write(u8 d)
{
    u8 buf, i ;
    
    SPI_CLK(0);
    for( i = 0; i < 8; i++ )
    {
        buf = (d >> (7-i)) & 0x1 ;
        SPI_DIN(buf);
        SPI_CLK(0);
        SPI_CLK(1);
        SPI_CLK(0);
    }
}

u16  Touch_Read(void)
{
    u16 buf ;
    u8 i ;
    
    buf=0;
    for( i = 0; i < 12; i++ )
    {
        buf = buf << 1 ;
        SPI_CLK(1);
        SPI_CLK(0);			
        if ( SPI_DOUT() )	
        {
            buf = buf + 1 ;
        }
    }
    return( buf ) ;
}

u8  Touch_Busy(void)
{
    // PC16 -> TS_BUSY
    return GPIO_ReadInputDataBit(GPIOC,GPIO_Pin_13);
}

u8  Touch_PenIRQ(void)
{
    // PB6 -> TS_nPENIRQ
    return GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_6);
}

void Touch_Initializtion()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
  /*****************************
  **    Ó²ŒþÁ¬œÓËµÃ÷          **
  ** STM32         TSC2046    **
  ** PC6    <----> nPENIRQ    ** i
  ** PC13    <----> BUSY      ** i
  ** PA5    <----> DCLK       ** o
  ** PA7    <----> DIN        ** o
  ** PA6    <----> DOUT       ** i
  ** PB7    <----> nCS        ** o
  ******************************/
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13; //Busy Pin
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //Dout
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //IRQ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7; //CLK, Din
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //nCS
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    printf("Touch Hardware Initialised!\r\n");

}

u16 _AD2X(int adx)
{
    u16 sx=0;
    int r = adx - 280;
    r *= 239;
    sx=r / (3740 - 280);
    if (sx<=0 || sx>240)
        return 0;
    return sx;
}

u16 _AD2Y(int ady)
{
    u16 sy=0;
    int r = ady - 230;
    r *= 319;
    sy=r/(3720 - 230);
    if (sy<=0 || sy>320)
        return 0;
    return sy;
}

uint16_t  Touch_MeasurementX(void)
{
    u8 i;
    u16 p=0;
    for (i=0;i<8;i++)
    {
        p+=Touch_GetPhyX();
        SPI_delay(1000);
    }
    p>>=3;
    p = (p-380)/14;
    if (p < 240)
        return ( 240 - p );
    else return 0;
}

uint16_t  Touch_MeasurementY(void)
{
    u8 i;
    u16 p=0;
    for (i=0;i<8;i++)
    {
        p+=Touch_GetPhyY();
        //printf("in y\r\n");
        SPI_delay(1000);
    }
    p>>=3;
    p = (((p-210)*2)/23);
    if (p < 320)
        return ( 320 - p );
    else return 0;
}

u16  Touch_GetPhyX(void)
{
    if (Touch_PenIRQ()) return 0;

    Touch_Start();
    Touch_Write(0x00);
    Touch_Write(CH_X);

    while(!Touch_Busy());
    return (Touch_Read());
}

u16  Touch_GetPhyY(void)
{
    if (Touch_PenIRQ()) return 0;

    Touch_Start();
    Touch_Write(0x00);
    Touch_Write(CH_Y);

    while(!Touch_Busy());
    return (Touch_Read());
}
/*
u16 Dx(u16 xx)
{
  return (_AD2X(xx));
}
u16 Dy(u16 yy)
{
  return (_AD2Y(yy));
}
*/
// Set duty cycle when using TIM3 as a PWM output.

void vTouchTask( void *pvParameters ) 
{
    Touch_Initializtion();
    portTickType xLastExecutionTime = xTaskGetTickCount();
    portTickType xTicksToWait = 1000/portTICK_RATE_MS;
    portBASE_TYPE xStatus;
    extern portBASE_TYPE usMaxJitter;
    unsigned int x = 0, y = 0, beep = TOUCH_BEEP; // current x,y value
    static unsigned int lx = 0, ly = 0, xx = 0x0000; //place to store the last
                                        //value of x and y for sending
                                        //to the menu

    portBASE_TYPE sense, changed;
    static portBASE_TYPE last;
    uint32_t id = 0;
    unsigned char b1 = 0, b2 = 0;
    
 
    for (;;)
    {
        //check TP every 50 Milliseconds
        vTaskDelayUntil(&xLastExecutionTime, 50/portTICK_RATE_MS );
        
        //measure x,y
        x = Touch_MeasurementX(); 
        y = Touch_MeasurementY();
        
        //do we have a touch?
        sense = 0;
        if ((x|y)!=0) // if we have a touch
            sense  = 1;
        changed = sense ^ last;
        


        // do we have negative edge of touch
        if ((changed==1) && (sense==0))
        {
            menu_key(lx, ly);    
            xQueueSend(xBeepQueue, &beep, 0);
            // SPI_FLASH_Init();
            // SPI_FLASH_WriteEnable();
            // b1 = SPI_FLASH_SendByte(0xA3);
            // b2 = SPI_FLASH_SendByte(0xA3);
            //SPI_FLASH_StartReadSequence(0x0);
            //id =  SPI_FLASH_ReadID();
            //SPI_FLASH_BufferRead(buf1, 0x00, 50);
            x = 30;
            SetTIM3Duty(xx+=1);
            printf("%x\r\n",TIM3->CCR3 );
            // sprintf(buf, "%u %u jittervar\r\n", usMaxJitter, TIM2->CNT );
//xStatus = xQueueSendToBack( xConsoleQueue, &buf, xTicksToWait );   
        }
        //save static variables
        last = sense; 
        lx = x;
        ly = y;
    }
}

   
portBASE_TYPE touchIsInWindow(uint16_t x, uint16_t y, uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2)
{
    //   char buf[256] = "in check\r\n";
    // xQueueSendToBack(xConsoleQueue, &buf, 0); //send message
    if (((x < x2) && (x > x1)) && ((y < y2) && (y > y1)))
        return pdTRUE;
    else return pdFALSE;
}
