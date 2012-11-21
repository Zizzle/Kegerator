#ifndef SETTINGS_H
#define SETTINGS_H

struct settings 
{
	uint16_t target_temp;
	uint16_t keg_cal_empty[8];
	uint16_t keg_cal_pint[8];
};

extern struct settings g_settings;

void settings_load();
void settings_save();

#endif

