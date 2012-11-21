//#include "stm32f10x_lib.h"
//#include "ili9320.h"
#include <stdio.h>

#include <stdint.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "touch.h"
#include "task.h"
#include "speaker.h"
#include "keg_display.h"
#include "spi.h"

#define CH_X  0xd0//0x90
#define CH_Y  0x90//0xd0

extern xQueueHandle xTPQueue;

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
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; //IRQ
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
   
    printf("Touch Hardware Initialised!\r\n");
}

void SPI_delay(u16 i)
{
    u16 k;
    for (k=0;k<i;k++);

}

void Touch_Write(u8 d)
{
	spi1_tx(d);
}

u16  Touch_Read(void)
{
	u16 val = spi1_rx() << 8 | spi1_rx();
	return val >> 3;
}

u16  Touch_GetPhyX(void)
{
	u16 val;
    if (Touch_PenIRQ()) return 0;
	if (!spi_select(SPI_DEVICE_TOUCH))	return 0;
    Touch_Write(0x00);
    Touch_Write(CH_X);
    while(!Touch_Busy());
    val = Touch_Read();
	spi_release();
	return val;
}

u16  Touch_GetPhyY(void)
{
	u16 val;
    if (Touch_PenIRQ()) return 0;
	if (!spi_select(SPI_DEVICE_TOUCH))	return 0;
    Touch_Write(0x00);
    Touch_Write(CH_Y);
    while(!Touch_Busy());
    val = Touch_Read();
	spi_release();
	return val;
}

int16_t  Touch_MeasurementY(void)
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
    else return -1;
}

int16_t  Touch_MeasurementX(void)
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
        return ( p );
    else return -1;
}

void vTouchTask( void *pvParameters ) 
{
    printf("Touch start\r\n");

    Touch_Initializtion();
    unsigned int x = 0, y = 0, beep = TOUCH_BEEP; // current x,y value
 
//    lcd_clear(0);

    unsigned char valid = 0;
    for( ;; )
    {
		int x,y;

        //measure x,y
        x = Touch_MeasurementX(); 
        y = Touch_MeasurementY();

//		printf("x %d y %d\r\n", x, y);

		if (x >=0 && x < 320 && y >= 0 && y < 240)
		{
			if (!valid)
			screen_touch(x, y);
			valid = 1;
		}
		else if (valid)
		{
			screen_touch(-1, -1);
			valid = 0;
		}

		vTaskDelay( 10 );
    }
}
