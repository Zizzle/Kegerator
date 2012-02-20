////////////////////////////////////////////////////////////////////////////
// Brad Goold 2012 
// DS1820 driver for STM32F103VE using FreeRTOS
////////////////////////////////////////////////////////////////////////////

#include "FreeRTOS.h"
#include "task.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "ds1820.h"
#include "queue.h"
#include "console.h"

// ROM COMMANDS
#define MATCH_ROM	0x55
#define	SEARCH_ROM	0xF0
#define SKIP_ROM	0xCC
#define READ_ROM        0x33
#define ALARM_SEARCH    0xEC

// FUNCTION COMMANDS
#define CONVERT_TEMP     0x44
#define COPY_SCRATCHPAD  0x48
#define WRITE_SCRATCHPAD 0x4E
#define READ_SCRATCHPAD  0xBE
#define RECALL_EEPROM    0xB8
#define READ_PS          0xB4

#define MAX_SENSORS      10

// ERROR DEFINES
#define BUS_ERROR      0xFE
#define PRESENCE_ERROR 0xFD
#define NO_ERROR       0x00

// BUS COMMANDS
#define DQ_IN()  {DS1820_PORT->CRH&=0xFFFFF0FF;DS1820_PORT->CRH |= 0x00000400;}
#define DQ_OUT() {DS1820_PORT->CRH&=0xFFFFF0FF;DS1820_PORT->CRH |= 0x00000300;}
#define DQ_SET()   GPIO_SetBits(DS1820_PORT, DS1820_PIN)
#define DQ_RESET() GPIO_ResetBits(DS1820_PORT, DS1820_PIN)
#define DQ_READ()  GPIO_ReadInputDataBit(DS1820_PORT, DS1820_PIN)

uint8_t sp[9]; //scratchpad
uint8_t sp1[9]; //scratchpad
int16_t ds1820_temperature = 0;
//int16_t ds1820_temperature1 = 0;
uint8_t rom[8];

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


#define HERMS_TEMP_SENSOR "\x10\x9c\xa4\x1e\x02\x08\x00\xf"
#define MASH_TEMP_SENSOR "\x10\xe3\x9b\x1e\x02\x08\x00\x58"



void vTaskDS1820Conversion( void *pvParameters ){
    char buf[30];
    int ii = 0;
    float temp; 
    ds1820_init();
    if (ds1820_reset() ==PRESENCE_ERROR)
    {
        sprintf(buf, "NO SENSOR DETECTED\r\n");
        xQueueSendToBack(xConsoleQueue, &buf, 0);
        vTaskDelete(NULL); // if this task fails... delete it
    }
    for (;;)
    {
        //    ds1820_search();
        ds1820_convert();
        vTaskDelay(1000);

        temp = ds1820_read_device(MASH_TEMP_SENSOR);
//        ds1820_get_temp();
        //for (ii = 8; ii >= 0; ii--)
        //    printf("%x ", sp1[ii]);
        //printf("\r\n");
        //printf("%.2f\r\n",  ((float)ds1820_temperature1)/100 );
        //printf("ROM CODE = ");
        //for (ii = 7; ii >= 0; ii--){
        //    printf("%x ", rom[ii]);
        // }
        //printf("\r\n");
        sprintf(buf, "Temperature = %.2fDeg-C\r\n", temp);
        xQueueSendToBack(xConsoleQueue, &buf, 0);
        taskYIELD();
    }
    
}

void ds1820_init(void) {
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
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    //printf("ioMode = %x\r\n", GPIOC->CRH);


}




unsigned char ds1820_reset(void)
{
    portENTER_CRITICAL();
    DQ_OUT();
    DQ_SET();
    DQ_RESET();
    delay_us(500);
    DQ_IN();
    
    delay_us(30);
    if (DQ_READ()!= 0)
    {
        return PRESENCE_ERROR;
    }
    portEXIT_CRITICAL();
    DQ_OUT();
    DQ_SET();
    delay_us(200); 
    
    return NO_ERROR;

}


void ds1820_write_bit(uint8_t bit){

    DQ_OUT();
    DQ_SET(); // make sure bus is high
    portENTER_CRITICAL();
    DQ_RESET(); // pull low for 2us
    delay_us(2);
    if (bit){
        DQ_IN();
    }
    else DQ_RESET();
    delay_us(60);
    portEXIT_CRITICAL();
    DQ_IN(); // return bus
}

uint8_t ds1820_read_bit(){
    uint8_t bit;
    delay_us(1);
    DQ_OUT();
    DQ_SET(); // make sure bus is high
    portENTER_CRITICAL();
    DQ_RESET(); // pull low for 2us
    delay_us(1);
    DQ_IN(); // give bus back to DS1820;
    delay_us(5); 
    bit = DQ_READ();
    delay_us(56);
    //DQ_OUT();
    //DQ_SET();
    portEXIT_CRITICAL();
    if (bit) 
        return 1;
    else return 0; 
}

void ds1820_write_byte(uint8_t byte){

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

uint8_t ds1820_read_byte(void){
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

void ds1820_convert(void){
    ds1820_reset();
    ds1820_write_byte(SKIP_ROM); 
    ds1820_write_byte(CONVERT_TEMP);
    DQ_IN();
    while(DQ_READ()==0){
        printf("waiting for conversion\r\n");
        delay_us(500);
    }
}
 
uint8_t ds1820_one_device_get_temp(void){
    int ii;

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

    //ds1820_fahrenheit = 3200 + (ds1820_temperature * 9) / 5;
    
}

uint8_t ds1820_search(){
    uint8_t b1=0, b2=0,b3= 0, b4 = 0, a=0, ii, jj;
    // ds1820_reset();
    ds1820_reset();
    ds1820_write_byte(READ_ROM); 
    portENTER_CRITICAL();
    for (ii = 0; ii < 8; ii++){
        rom[ii] = ds1820_read_byte();
    }
    portEXIT_CRITICAL();
    ds1820_reset();
}



float ds1820_read_device(uint8_t * rom_code){
    float retval;
    uint16_t ds1820_temperature1 = 10000;
    uint8_t ii; 
    ds1820_reset();
    ds1820_write_byte(MATCH_ROM);
    portENTER_CRITICAL();
    for (ii = 0; ii < 8; ii++)
    {
        ds1820_write_byte(rom_code[ii]);
    }
    ds1820_write_byte(READ_SCRATCHPAD);
    for (ii = 0; ii < 9; ii++){
        sp1[ii] = ds1820_read_byte();
    }
    portEXIT_CRITICAL();
    ds1820_reset();
    ds1820_temperature1 = sp1[1] & 0x0f;
    ds1820_temperature1 <<= 8;
    ds1820_temperature1 |= sp1[0];
    unsigned char remain = sp1[6];
    ds1820_temperature1 >>= 1;
    ds1820_temperature1 = (ds1820_temperature1 * 100) -  25  + (100 * 16 - remain * 100) / (16);
    retval = ((float)ds1820_temperature1/100);
    return retval;
}
