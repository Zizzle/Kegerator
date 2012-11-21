#include "stm32f10x.h"
#include "spi_flash.h"
#include "settings.h"

struct settings g_settings;

void settings_load()
{

	flash_read(0, &g_settings, sizeof(g_settings));

	if (g_settings.target_temp > 1000)
		g_settings.target_temp = 600; // failsafe
}

void settings_save()
{
	flash_write(0, &g_settings, sizeof(g_settings));
}
