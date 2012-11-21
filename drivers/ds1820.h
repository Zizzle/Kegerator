///////////////////////////////////////////////////////////////////////////////
//Brad Goold 2012
//
//
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DS1820_H
#define DS1820_H

#define DS1820_PORT GPIOE
#define DS1820_PIN  GPIO_Pin_5

#define BUS_ERROR      0xFE
#define PRESENCE_ERROR 0xFD
#define NO_ERROR       0x00

// DS1820 Sensor defines: Use these macro's when calling ds1820_get_temp()
#define HLT 0
#define MASH 1
#define CABINET 2
#define AMBIENT 3
#define SPARE 4

//Scheduled task for FreeRTOS
void          vTaskDS1820Convert( void *pvParameters ); //task to

//get temp of a particular sensor
uint32_t         ds1820_get_temp();

#endif
