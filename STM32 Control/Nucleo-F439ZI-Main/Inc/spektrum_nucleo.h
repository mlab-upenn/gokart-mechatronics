#ifndef _SPEKTRUM_NUCLEO_H
#define _SPEKTRUM_NUCLEO_H

#include "stm32f4xx_hal.h"
#include "spektrum.h"

typedef enum {
	UNINITIALIZED = 0,
	RECEIVING_PACKET = 1,
	EXPECTING_DELAY = 2,
} spektrum_nucleo_receiver_state;

typedef struct spektrum_nucleo_state {

	__IOM spektrum_nucleo_receiver_state receiver_state;

	spektrum_internal_packet_t packet;
	__IOM uint32_t buffer_size;
	spektrum_internal_msg_t msg;

	__IOM uint32_t uart_idle_it_prev_ts;
	__IOM uint32_t uart_idle_it_count;
	__IOM uint32_t uart_idle_it_last_ts;

	// stats
	__IOM uint32_t num_lost_bytes;
	__IOM uint32_t num_lost_packets;
	__IOM uint32_t num_invalid_packets;
	__IOM uint32_t num_overruns;

	// message handler
	spektrum_msg_handler msg_handler;
	void *msg_handler_context;

} spektrum_nucleo_state_t;

void spektrum_nucleo_init(spektrum_nucleo_state_t *state);

void spektrum_nucleo_reset_receiver(spektrum_nucleo_state_t *state, bool full);

void spektrum_nucleo_handle_overrun(spektrum_nucleo_state_t *state);

void spektrum_nucleo_handle_byte(spektrum_nucleo_state_t *state, uint8_t data);

void spektrum_nucleo_handle_idle(spektrum_nucleo_state_t *state, uint32_t ts);

void spektrum_nucleo_reset_stats(spektrum_nucleo_state_t *state);

void spektrum_nucleo_start_receiving(UART_HandleTypeDef *huart);

void spektrum_nucleo_irq_handler(spektrum_nucleo_state_t *state, UART_HandleTypeDef *huart);

// void spektrum_nucleo_process_msg(spektrum_nucleo_state_t *state);

#endif // _SPEKTRUM_NUCLEO_H
