#include <string.h>
#include "spektrum_nucleo.h"

void spektrum_nucleo_init(spektrum_nucleo_state_t *state) {
	memset(state, 0, sizeof(spektrum_nucleo_state_t));
}

void spektrum_nucleo_reset_receiver(spektrum_nucleo_state_t *state, bool full) {
	state->num_lost_bytes += state->buffer_size;
	state->buffer_size = 0;
	if (full) {
		state->receiver_state = UNINITIALIZED;
	}
}

static void spektrum_nucleo_process_packet(spektrum_nucleo_state_t *state) {
	// check packet structure first
	uint8_t system = state->packet.system;
	if (
		system != SPEKTRUM_SYSTEM_22MS_1024_DSM2
		&& system != SPEKTRUM_SYSTEM_11MS_2048_DSM2
		&& system != SPEKTRUM_SYSTEM_22MS_2048_DSMS
		&& system != SPEKTRUM_SYSTEM_11MS_2048_DSMX
		) {
		state->num_invalid_packets++;
		state->buffer_size = 0;
		return;
	}

	// parse it
	spektrum_packet_to_msg(&state->packet, &state->msg);

	state->msg_handler(&state->msg, state->msg_handler_context);

	// reset buffer
	state->buffer_size = 0;
}

void spektrum_nucleo_handle_overrun(spektrum_nucleo_state_t *state) {
	state->num_overruns++;
	spektrum_nucleo_reset_receiver(state, true);
}

void spektrum_nucleo_handle_byte(spektrum_nucleo_state_t *state, uint8_t data) {

	if (state->receiver_state == EXPECTING_DELAY) {
		state->num_lost_bytes++;
		return;
	}

	if (state->receiver_state == UNINITIALIZED) {
		state->receiver_state = RECEIVING_PACKET;
	}

	uint8_t *buffer = (uint8_t *) &state->packet;

	buffer[state->buffer_size++] = data;

	if (state->buffer_size == sizeof(spektrum_internal_packet_t)) {
		spektrum_nucleo_process_packet(state);
		state->receiver_state = EXPECTING_DELAY;
	}

}

void spektrum_nucleo_handle_idle(spektrum_nucleo_state_t *state, uint32_t ts) {
	state->uart_idle_it_last_ts = ts;
	state->uart_idle_it_count += 1u;

	if (state->receiver_state == RECEIVING_PACKET) {
		state->num_lost_packets += 1u;
		spektrum_nucleo_reset_receiver(state, false);
		return;
	}

	state->receiver_state = RECEIVING_PACKET;

}

void spektrum_nucleo_reset_stats(spektrum_nucleo_state_t *state) {
	state->num_lost_bytes = 0u;
	state->num_lost_packets = 0u;
	state->num_invalid_packets = 0u;
	state->num_overruns = 0u;
}

void spektrum_nucleo_start_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void spektrum_nucleo_stop_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
}

void spektrum_nucleo_irq_handler(spektrum_nucleo_state_t *state, UART_HandleTypeDef *huart) {

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
		__HAL_UART_CLEAR_OREFLAG(huart);
		spektrum_nucleo_handle_overrun(state);
		return;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET) {
		uint8_t data = (uint8_t) (huart->Instance->DR & (uint8_t) 0x00FF);
		spektrum_nucleo_handle_byte(state, data);
		return;
	}

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == SET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);
		spektrum_nucleo_handle_idle(state, HAL_GetTick());
		return;
	}
};
