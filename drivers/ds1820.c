////////////////////////////////////////////////////////////////////////////
// Brad Goold 2012 
// DS1820 driver for STM32F103VE using FreeRTOS
////////////////////////////////////////////////////////////////////////////

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ds1820.h"
#include "queue.h"
#include "lcd.h"

static uint32_t g_temperature;

// ROM COMMANDS
#define SKIP_ROM	0xCC

// FUNCTION COMMANDS
#define CONVERT_TEMP     0x44
#define READ_SCRATCHPAD  0xBE

// ERROR DEFINES
#define BUS_ERROR      0xFE
#define PRESENCE_ERROR 0xFD
#define NO_ERROR       0x00

// BUS COMMANDS
//#define DQ_IN()  {DS1820_PORT->CRL&=0xFF0FFFFF;DS1820_PORT->CRH |= 0x00400000;}
//#define DQ_OUT() {DS1820_PORT->CRL&=0xFF0FFFFF;DS1820_PORT->CRH |= 0x00300000;}
#define DQ_SET()   GPIO_SetBits(DS1820_PORT, DS1820_PIN)
#define DQ_RESET() GPIO_ResetBits(DS1820_PORT, DS1820_PIN)
#define DQ_READ()  GPIO_ReadInputDataBit(DS1820_PORT, DS1820_PIN)

// STATIC FUNCTIONS
static void ds1820_convert(void);
static void ds1820_init(void);
static unsigned char ds1820_reset(void);
static void ds1820_write_bit(uint8_t bit);
static uint8_t ds1820_read_bit(void);
static void ds1820_write_byte(uint8_t byte);
static uint8_t ds1820_read_byte(void);
static void ds1820_convert(void);
static void delay_us(uint16_t count); 


void DQ_OUT()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}

void DQ_IN()
{
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
}


////////////////////////////////////////////////////////////////////////////
// Static Functions 
////////////////////////////////////////////////////////////////////////////

static void delay_us(uint16_t count){ 
    
    uint16_t TIMCounter = count;
    TIM_Cmd(TIM2, ENABLE);
    TIM_SetCounter(TIM2, TIMCounter);
    while (TIMCounter)
    {
        TIMCounter = TIM_GetCounter(TIM2);
    }
    TIM_Cmd(TIM2, DISABLE);
}
////////////////////////////////////////////////////////////////////////////
static void ds1820_init(void)
{
    // intialise timer 2 for counting 
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    TIM_TimeBaseStructure.TIM_Period = 1;                
    TIM_TimeBaseStructure.TIM_Prescaler = 72;       //72MHz->1MHz
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;   
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Down;  
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    GPIO_InitTypeDef GPIO_InitStructure;
    
    // PC10-DQ
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOE, &GPIO_InitStructure);
    //printf("ioMode = %x\r\n", GPIOC->CRH);


}
////////////////////////////////////////////////////////////////////////////

static unsigned char ds1820_reset(void)
{
    portENTER_CRITICAL();
    DQ_OUT();
    DQ_SET();
    DQ_RESET();
    delay_us(500);
    DQ_IN();
    
    delay_us(60);
    if (DQ_READ()!= 0)
    {
        portEXIT_CRITICAL();
        return PRESENCE_ERROR;
       
    }
    portEXIT_CRITICAL();

    delay_us(240); 
    return NO_ERROR;
}
////////////////////////////////////////////////////////////////////////////

static void ds1820_write_bit(uint8_t bit){

    DQ_OUT();
    DQ_SET(); // make sure bus is high
    DQ_RESET(); // pull low for 2us
    delay_us(2);
    if (bit){
        DQ_IN();
    }
    else DQ_RESET();
    delay_us(60);
    DQ_IN(); // return bus
}
////////////////////////////////////////////////////////////////////////////

static uint8_t ds1820_read_bit(void){
    uint8_t bit;
    delay_us(1);
    DQ_OUT();
    DQ_SET(); // make sure bus is high
    DQ_RESET(); // pull low for 2us
    delay_us(1);
    DQ_IN(); // give bus back to DS1820;
    delay_us(5); 
    bit = DQ_READ();
    delay_us(56);
    //DQ_OUT();
    //DQ_SET();
    if (bit) 
        return 1;
    else return 0; 
}
////////////////////////////////////////////////////////////////////////////

static void ds1820_write_byte(uint8_t byte){

    delay_us(100);
    portENTER_CRITICAL();
    int ii;
    for (ii = 0; ii < 8; ii++) {
        if (byte&0x01){
            ds1820_write_bit(1);
            // printf("1 written\r\n");
        }
        else {
            ds1820_write_bit(0);
            //printf("0 written\r\n");
        }
        byte = byte>>1;
    } 
    //delay_us(100);
   portEXIT_CRITICAL();
}
////////////////////////////////////////////////////////////////////////////

static uint8_t ds1820_read_byte(void){
    int ii;
    uint8_t bit, byte = 0;
    portENTER_CRITICAL();
    delay_us(1);
    for (ii = 0; ii < 8; ii++) {
        byte = byte >> 1;
        bit = ds1820_read_bit();
        if (bit==0) {
            byte = byte & 0x7F;
        }
        else byte = byte | 0x80;
    }
    portEXIT_CRITICAL();
    return byte;
}
////////////////////////////////////////////////////////////////////////////

static void ds1820_convert(void){
    portENTER_CRITICAL();
    ds1820_reset();
    ds1820_write_byte(SKIP_ROM); 
    ds1820_write_byte(CONVERT_TEMP);
    ds1820_reset();
    portEXIT_CRITICAL();
    DQ_IN();
       
}
////////////////////////////////////////////////////////////////////////////

uint32_t ds1820_get_temp()
{
	return g_temperature;
}

uint32_t ds1820_one_device_get_temp(void){
    int ii;
    uint8_t sp[9]; //scratchpad
    int16_t ds1820_temperature = 0;

    ds1820_reset();
    ds1820_write_byte(SKIP_ROM); 
    ds1820_write_byte(READ_SCRATCHPAD);
    for (ii = 0; ii < 9; ii++){
        sp[ii] = ds1820_read_byte();
    }

    ds1820_reset();
    ds1820_temperature = sp[1] & 0x0f;
    ds1820_temperature <<= 8;
    ds1820_temperature |= sp[0];
    unsigned char remain = sp[6];
    ds1820_temperature >>= 1;
    ds1820_temperature = (ds1820_temperature * 100) -  25  + (100 * 16 - remain * 100) / (16);
    return ds1820_temperature; 
}
////////////////////////////////////////////////////////////////////////////
// Interfacing Function
////////////////////////////////////////////////////////////////////////////
void vTaskDS1820Convert( void *pvParameters )
{
    // initialise the bus
    ds1820_init();
    if (ds1820_reset() ==PRESENCE_ERROR)
    {
//		lcd_printf(10, 10, 10, "No DS1820");
    }
	else
	{
//		lcd_printf(10, 10, 10, "Found DS1820");
	}

    for (;;)
    {
        ds1820_convert();
        vTaskDelay(2000); // wait for conversion
		g_temperature= ds1820_one_device_get_temp();
    }    
}
////////////////////////////////////////////////////////////////////////////

