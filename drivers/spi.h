#ifndef SPI_H
#define SPI_H

// the number of ticks spi_select will block waiting to get the mutex
#define SPI_DEFAULT_LOCK_WAIT 10000

enum SpiDevice
{
    SPI_DEVICE_NONE, // use for detecting read/write without locks
    SPI_DEVICE_TOUCH,
    SPI_DEVICE_FLASH
};

// Take a mutex and Chip select the device.
// Returns 0 on failure, 1 on success
int spi_select(enum SpiDevice device);

void spi_release();


#endif

