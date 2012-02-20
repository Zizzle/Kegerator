///////////////////////////////////////////////////////////////////////////////
//Brad Goold 2012
//
//
//
///////////////////////////////////////////////////////////////////////////////

#ifndef DS1820_H
#define DS1820_H

#define DS1820_PORT GPIOC
#define DS1820_PIN  GPIO_Pin_10


#define BUS_ERROR      0xFE
#define PRESENCE_ERROR 0xFD
#define NO_ERROR       0x00



void          ds1820_init(void);
unsigned char ds1820_reset(void);
void          vTaskDS1820Conversion( void *pvParameters );
void          ds1820_convert(void);
uint8_t       ds1820_one_device_get_temp(void);
uint8_t       ds1820_search(void);
uint8_t         ds1820_get_temp(uint8_t * rom_code);
float         ds1820_read_device(uint8_t * rom_code);
#endif
