#include "endianness.h"
#include "spektrum.h"
#include "string.h"

char *channel_names[SPEKTRUM_NUM_STD_CHANNELS] = {
	"Throttle",
	"Aileron",
	"Elevator",
	"Rudder",
	"Gear",
	"Aux_1",
	"Aux_2",
	"Aux_3",
	"Aux_4",
	"Aux_5",
	"Aux_6",
	"Aux_7",
};

void spektrum_packet_to_msg(spektrum_internal_packet_t *packet, spektrum_internal_msg_t *msg) {
	msg->fades = packet->fades;
	msg->system = packet->system;
	for (int i = 0; i < 7; ++i) {
		uint16_t value = ntohs(packet->servo[i]);
		spektrum_channel_data_t *servo = &msg->data[i];
		if (packet->system == SPEKTRUM_SYSTEM_22MS_1024_DSM2) {
			servo->servo_phase = false;
			servo->channel_id = (SPEKTRUM_MASK_1024_CHANNEL_ID & value) >> 10u;
			servo->servo_position = SPEKTRUM_MASK_1024_SERVO_POS & value;
		} else {
			// else 2048
			servo->servo_phase = 0x8000u & value;
			servo->channel_id = (SPEKTRUM_MASK_2048_CHANNEL_ID & value) >> 11u;
			servo->servo_position = SPEKTRUM_MASK_2048_SERVO_POS & value;
		}
	}
}

void spektrum_init(spektrum_packet_parser_state_t *state) {
	memset(state, 0, sizeof(spektrum_packet_parser_state_t));
}

void spektrum_process_data(
	const uint8_t *data, int size,
	spektrum_packet_parser_state_t *state,
	spektrum_msg_handler handler,
	void *context
) {

	uint8_t *buffer = (uint8_t *) &state->packet;
	uint8_t *system = &state->packet.system;

	const uint8_t *data_end_ptr = data + size;
	const uint8_t *byte_ptr = data;

	while (byte_ptr < data_end_ptr) {

		uint8_t byte = *byte_ptr++;

		// save byte to buffer and then increase buffer index
		buffer[state->buffer_size++] = byte;

		// once we have system field value we validate it
		if (state->buffer_size == 2) {
			if (
				*system != SPEKTRUM_SYSTEM_22MS_1024_DSM2
				&& *system != SPEKTRUM_SYSTEM_11MS_2048_DSM2
				&& *system != SPEKTRUM_SYSTEM_22MS_2048_DSMS
				&& *system != SPEKTRUM_SYSTEM_11MS_2048_DSMX
				) {
				// shift buffer by one byte
				//   this effectively means that we drop only the fades value
				//   and consider the system value as fades value
				//   and get the new system value in the next iteration
				buffer[0] = buffer[1];
				state->buffer_size = 1;
				state->dropped_bytes++;
			}
			continue;
		}

		// packet is complete
		if (state->buffer_size == SPEKTRUM_INTERNAL_PACKET_SIZE) {

			// convert packet to message
			spektrum_packet_to_msg(&state->packet, &state->msg);

			// call the handler
			handler(&state->msg, context);

			// reset buffer
			state->buffer_size = 0;

			// do receive more than one packet
			continue;
		}

	}

}

void spektrum_msg_to_state(const spektrum_internal_msg_t *msg, spektrum_state_t *state, long current_time) {

	state->fades = msg->fades;
	state->system = msg->system;

	if (state->last_msg_ts > 0) {
		long delay = current_time - state->last_msg_ts;
		if (delay > state->max_msg_delay) {
			state->max_msg_delay = delay;
		}
	}

	state->connected = true;
	state->last_msg_ts = current_time;

	for (int i = 0; i < SPEKTRUM_NUM_MSG_CHANNELS; ++i) {
		const spektrum_channel_data_t *channel = &msg->data[i];
		if (channel->channel_id == SPEKTRUM_CHANNEL_LAST) {
			// skip the special channel that is used as the end of the data section of the packet
			continue;
		}
		if (channel->channel_id < 0 || channel->channel_id >= SPEKTRUM_NUM_STD_CHANNELS) {
			state->num_unexpected_channels++;
			continue;
		}
		state->channels[channel->channel_id].servo_phase = channel->servo_phase;
		state->channels[channel->channel_id].servo_position = channel->servo_position;
	}

}

void spektrum_reset_state_stats(spektrum_state_t *state) {
	state->connected = false;
	state->num_unexpected_channels = 0u;
	state->last_msg_ts = 0u;
	state->max_msg_delay = 0u;
}

