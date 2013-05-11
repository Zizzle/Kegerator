#ifndef TEMP_CONTROL_H
#define TEMP_CONTROL_H

void temp_set_target(int target);
int temp_get_target();
void vTaskTempControl( void *pvParameters );

#define STATE_OFF 0
#define STATE_WAITING 1
#define STATE_HYSTERESIS 2
#define STATE_ON 3

int temp_get_state();

#endif
