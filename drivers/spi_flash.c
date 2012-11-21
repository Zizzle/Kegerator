#include "stm32f10x.h"
#include "spi.h"

#define FLASH_READ_BYTES 0x03
#define FLASH_READ_STATUS 0x05
#define FLASH_WRITE_ENABLE 0x06
#define FLASH_SECTOR_ERASE 0xD8
#define FLASH_PROGRAM_PAGE 0x2

#define FLASH_WRITE_IN_PROGESS 1

int flash_read(uint32_t address, uint8_t *buffer, uint32_t len)
{
	if (!spi_select(SPI_DEVICE_FLASH))
		return -1;

	printf("SPI RX\r\n");
	spi1_tx(FLASH_READ_BYTES);
	flash_send_address(address);
	for (int ii = 0; ii < len; ii++)
	{
		buffer[ii] = spi1_rx();
		printf("Got %x\r\n", buffer[ii]);	
	}

	spi_release();
}

void flash_read_id()
{
	if (!spi_select(SPI_DEVICE_FLASH))
		return -1;
	spi1_tx(0x9f);
	printf("ID %x\r\n", spi1_rx() );	
	printf("ID %x\r\n", spi1_rx() );	
	printf("ID %x\r\n", spi1_rx() );	
	printf("SR %x\r\n", flash_read_status_register());	
	spi_release();
}

int flash_read_status_register()
{
	spi1_tx(FLASH_READ_STATUS);
	return spi1_rx();
}

int flash_wait_for_write_complete()
{
	if (!spi_select(SPI_DEVICE_FLASH))	return -1;
	int status;
	do
	{
		status = flash_read_status_register();
	} while (status & FLASH_WRITE_IN_PROGESS);

	printf("SR %x\r\n", flash_read_status_register());	
	spi_release();
}

int flash_write_enable()
{
	if (!spi_select(SPI_DEVICE_FLASH))	return -1;
	spi1_tx(FLASH_WRITE_ENABLE);
	spi_release();
	
	if (!spi_select(SPI_DEVICE_FLASH))	return -1;
	printf("WE SR %x\r\n", flash_read_status_register());	
	spi_release();
}

void flash_send_address(uint32_t address)
{
	spi1_tx((address >> 16) & 0xFF);
	spi1_tx((address >> 8 ) & 0xFF);
	spi1_tx((address >> 0 ) & 0xFF);
}

int flash_write(uint32_t address, uint8_t *buffer, uint32_t len)
{
	flash_write_enable();

	if (!spi_select(SPI_DEVICE_FLASH))	return -1;
	spi1_tx(FLASH_SECTOR_ERASE);
	flash_send_address(address);
	spi_release();

	flash_wait_for_write_complete();

	flash_write_enable();

	if (!spi_select(SPI_DEVICE_FLASH))	return -1;
	spi1_tx(FLASH_PROGRAM_PAGE);
	flash_send_address(address);
	for (int ii = 0; ii < len; ii++)
	{
		spi1_tx(buffer[ii]);
	}	
	spi_release();

	flash_wait_for_write_complete();
}
