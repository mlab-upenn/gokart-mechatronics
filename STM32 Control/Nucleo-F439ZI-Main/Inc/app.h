#ifndef _APP_H
#define _APP_H

#include "spektrum_nucleo.h"

typedef struct app_state {

	float acc_percent;
	float steer_percent;

	int control_mode;
	int gokart_status;

	spektrum_nucleo_state_t rc_receiver;
	spektrum_state_t rc_receiver_state;

} app_state_t;

void app_run(app_state_t *app);

extern app_state_t *main_app;
extern UART_HandleTypeDef huart2;

#define SPEKTRUM_UART &huart2

#endif
