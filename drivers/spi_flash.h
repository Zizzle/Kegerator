#ifndef SPI_FLASH_H
#define SPI_FLASH_H

int flash_read(uint32_t address, uint8_t *buffer, uint32_t len);
int flash_write(uint32_t address, uint8_t *buffer, uint32_t len);

#endif

