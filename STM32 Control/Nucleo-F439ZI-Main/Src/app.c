#include <stdio.h>
#include <string.h>

#include "app.h"
#include "terminal.h"
#include "utils.h"
#include "endianness.h"

float GOKART_STEER_MAX = 50.0;

int SPEKTRUM_THROTTLE_MIN = 330;
int SPEKTRUM_THROTTLE_MAX = 1701;
int SPEKTRUM_THROTTLE_NEUTRAL = 994;

int SPEKTRUM_STEER_MIN = 411;
int SPEKTRUM_STEER_MAX = 1762;
int SPEKTRUM_STEER_NEUTRAL = 1072;

#define JOY_STEERING_INDEX 1
#define JOY_THROTTLE_INDEX 0
#define JOY_EMK_INDEX 4
#define JOY_MODE_INDEX 5

static void joy_steer_to_steer(app_state_t *app){
	int joy_steer = app->rc_receiver_state.channels[JOY_STEERING_INDEX].servo_position;

	if (joy_steer > SPEKTRUM_STEER_NEUTRAL){
		float steering_percent = ((float) joy_steer - (float)SPEKTRUM_STEER_NEUTRAL) / ((float)SPEKTRUM_STEER_MAX - (float)SPEKTRUM_STEER_NEUTRAL);
		app->steering_angle = steering_percent * GOKART_STEER_MAX + GOKART_STEER_MAX;
	} else{
		float steering_percent = ((float) SPEKTRUM_STEER_NEUTRAL - (float)joy_steer) / ((float)SPEKTRUM_STEER_NEUTRAL - (float)SPEKTRUM_STEER_MIN);
		app->steering_angle = -steering_percent * GOKART_STEER_MAX + GOKART_STEER_MAX;
	}
}

static void joy_throttle_to_throttle(app_state_t *app){
	int joy_throttle = app->rc_receiver_state.channels[JOY_THROTTLE_INDEX].servo_position;
	float acc_percent = (float)(joy_throttle - SPEKTRUM_THROTTLE_NEUTRAL) / (float)(SPEKTRUM_THROTTLE_MAX - SPEKTRUM_THROTTLE_NEUTRAL);

	app->acc_percent = acc_percent;

	if (acc_percent < -0.6){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
	} else if (acc_percent > -0.6 && acc_percent > -0.2){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	} else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	}
}

static void convert_channels_to_commands(app_state_t *app) {
	joy_steer_to_steer(app);
	joy_throttle_to_throttle(app);
}

static void handle_spektrum_msg(const spektrum_internal_msg_t *msg, void *context) {

	app_state_t *app = (app_state_t *) context;

	spektrum_msg_to_state(msg, &app->rc_receiver_state, (long) HAL_GetTick());

	if (app->num_rc_readings < 5) {
		app->num_rc_readings++;
		return;
	}

	// convert to commands
	convert_channels_to_commands(app);
}

void app_debug_loop(app_state_t *app) {

}

void app_run(app_state_t *app) {

	// STARTUP
	printf(red_bold("------------------------------") nl);
	printf(MAIN "waiting..." nl);
	debug_sizeof();

	// add a delay to ensure all external peripherals correctly start up
	// and start providing correct data
	HAL_Delay(100);

	printf(MAIN "starting..." nl);

	app->steering_angle = GOKART_STEER_MAX;

	spektrum_nucleo_state_t *rc_receiver = &app->rc_receiver;

	spektrum_nucleo_init(rc_receiver);
	spektrum_nucleo_start_receiving(SPEKTRUM_UART);

	rc_receiver->msg_handler = handle_spektrum_msg;
	rc_receiver->msg_handler_context = app;
}
