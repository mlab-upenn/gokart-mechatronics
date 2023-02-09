#ifndef _APP_H
#define _APP_H

#include "main.h"
#include "spektrum_nucleo.h"

#define MAIN cyan_bold("F4 main:  ")

typedef struct app_state {

	float acc_percent;
	float steering_angle;

	int shut_power;

	spektrum_nucleo_state_t rc_receiver;
	spektrum_state_t rc_receiver_state;

	__IOM int num_rc_readings;

} app_state_t;

extern app_state_t *main_app;

void app_run(app_state_t *app);

#define SPEKTRUM_UART (&huart2)
extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim1; // speed sensor pulse width measurement

#endif // _APP_H
