#ifndef TEMP_CONTROL_H
#define TEMP_CONTROL_H

void temp_set_target(int target);
int temp_get_target();
void vTaskTempControl( void *pvParameters );

#endif
