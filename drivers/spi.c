
#include "stm32f10x.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"


static xSemaphoreHandle xSpiMutex;
static enum SpiDevice spi_device = SPI_DEVICE_NONE;

void spi1_init()
{
	SPI_InitTypeDef  SPI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB|RCC_APB2Periph_SPI1|RCC_APB2Periph_ADC1|RCC_APB2Periph_USART1, ENABLE);

	/* Configure CS for flash */  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	/* Configure CS for touch screen */  
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7; //nCS
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	/* Configure SPI1 pins: SCK and MOSI with default alternate function (not remapped) push-pull */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* Configure MISO as Input with internal pull-up */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
 	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
  	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
  	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
  	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
  	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
  	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
  	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
  	SPI_InitStructure.SPI_CRCPolynomial = 7;
  	SPI_Init(SPI1, &SPI_InitStructure);
	
	SPI_CalculateCRC(SPI1, DISABLE);
	SPI_Cmd(SPI1, ENABLE);
}

int spi1_tx(int tx)
{
	SPI_I2S_SendData(SPI1,tx);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI1);
}
int spi1_rx()
{
	SPI_I2S_SendData(SPI1,0xFF);
	while(SPI_I2S_GetFlagStatus(SPI1,SPI_I2S_FLAG_RXNE)==RESET);
	return SPI_I2S_ReceiveData(SPI1);
}

void spi_init()
{
    xSpiMutex = xSemaphoreCreateMutex();

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);

	printf("SPI INIT\r\n");
	spi1_init();
}

int spi_select(enum SpiDevice device)
{
    if( xSemaphoreTake( xSpiMutex, SPI_DEFAULT_LOCK_WAIT ) != pdTRUE )
    {	
		return 0;
    }
    spi_device = device;
    switch (device)
    {
		case SPI_DEVICE_TOUCH:
			GPIO_ResetBits(GPIOB, GPIO_Pin_7);
			break;

		case SPI_DEVICE_FLASH:
			GPIO_ResetBits(GPIOA, GPIO_Pin_4);
			break;

		default: break;
    }

    return 1;
}

void spi_release()
{
    xSemaphoreGive(xSpiMutex);
    spi_device = SPI_DEVICE_NONE;

	GPIO_SetBits(GPIOA, GPIO_Pin_4);
	GPIO_SetBits(GPIOB, GPIO_Pin_7);
}

