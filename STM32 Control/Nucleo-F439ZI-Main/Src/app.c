#include "stdio.h"
#include "app.h"

float SPEKTRUM_THROTTLE_MIN = 324.0;
float SPEKTRUM_THROTTLE_MAX = 1706.0;
float SPEKTRUM_THROTTLE_NEUTRAL = 1015.0;

float SPEKTRUM_STEER_MIN = 404.0;
float SPEKTRUM_STEER_MAX = 1761.0;
float SPEKTRUM_STEER_NEUTRAL = 1065.0;

#define JOY_STEERING_INDEX 1
#define JOY_THROTTLE_INDEX 0
#define JOY_EMK_INDEX 4
#define JOY_MODE_INDEX 5

static void joy_steer_to_steer(app_state_t *app){
	int steer_val = app->rc_receiver_state.channels[JOY_STEERING_INDEX].servo_position;

	if (steer_val > SPEKTRUM_STEER_NEUTRAL){
		app->steer_percent = (steer_val - SPEKTRUM_STEER_NEUTRAL) / (SPEKTRUM_STEER_MAX - SPEKTRUM_STEER_NEUTRAL);
	} else{
		app->steer_percent = -(SPEKTRUM_STEER_NEUTRAL - steer_val) / (SPEKTRUM_STEER_NEUTRAL - SPEKTRUM_STEER_MIN);
	}
}

static void joy_acc_to_acc(app_state_t *app){
	int acc_val = app->rc_receiver_state.channels[JOY_THROTTLE_INDEX].servo_position;
	float acc_percent = (acc_val - SPEKTRUM_THROTTLE_NEUTRAL) / (SPEKTRUM_THROTTLE_MAX - SPEKTRUM_THROTTLE_NEUTRAL);

	app->acc_percent = acc_percent;

	if (acc_percent < 0.0){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
	} else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	}
}

static void joy_control_to_control(app_state_t *app){
	int control_val = app->rc_receiver_state.channels[5].servo_position;

	if (control_val == 1706){
		app->control_mode = 0;
	} else if (control_val == 1024){
		app->control_mode = 1;
	} else if (control_val == 342){
		app->control_mode = 2;
	}

	int deadmanswitch = app->rc_receiver_state.channels[4].servo_position;

	if (deadmanswitch == 342){
		app->gokart_status = 0;
	} else if (deadmanswitch == 1706){
		app->gokart_status = 1;
	}
}

static void convert_channels_to_commands(app_state_t *app) {
	joy_steer_to_steer(app);
	joy_acc_to_acc(app);
	joy_control_to_control(app);
}

static void handle_spektrum_msg(const spektrum_internal_msg_t *msg, void *context) {
	app_state_t *app = (app_state_t *) context;
	spektrum_msg_to_state(msg, &app->rc_receiver_state, (long) HAL_GetTick());
	convert_channels_to_commands(app);
}

void app_run(app_state_t *app) {
	app->steer_percent = 0.0;
	app->acc_percent = 0.0;
	app->control_mode = 0;
	app->gokart_status = 0;

	spektrum_nucleo_state_t *rc_receiver = &app->rc_receiver;

	spektrum_nucleo_init(rc_receiver);
	spektrum_nucleo_start_receiving(SPEKTRUM_UART);

	rc_receiver->msg_handler = handle_spektrum_msg;
	rc_receiver->msg_handler_context = app;
}
