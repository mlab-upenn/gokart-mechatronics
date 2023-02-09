#ifndef _SPEKTRUM_H
#define _SPEKTRUM_H

#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SPEKTRUM_NUM_MSG_CHANNELS 7

typedef struct spektrum_internal_packet {
	uint8_t fades;
	uint8_t system;
	// All data fields are big-endian, that is,
	// the MSB is transmitted before the LSB.
	// Bit 0 is the lsb, bit 15 is the msb.
	uint16_t servo[SPEKTRUM_NUM_MSG_CHANNELS];
	// TODO:
	//   It seems that the last channel data are always 0xFFFF.
	//   Maybe we can use that to improve packets detection algorithm.
	//   Currently, we rely only on the system field (whose value is not unique within the packet).
}  __attribute__((__packed__)) spektrum_internal_packet_t;

#ifdef static_assert
static_assert(
	sizeof(spektrum_internal_packet_t) == 16,
	"sizeof(spektrum_internal_packet_t) is not 16 bytes"
);
#endif // static_assert

#define SPEKTRUM_INTERNAL_PACKET_SIZE sizeof(spektrum_internal_packet_t)

#define SPEKTRUM_SYSTEM_22MS_1024_DSM2 0x01
#define SPEKTRUM_SYSTEM_11MS_2048_DSM2 0x12
#define SPEKTRUM_SYSTEM_22MS_2048_DSMS 0xa2
#define SPEKTRUM_SYSTEM_11MS_2048_DSMX 0xb2

#define SPEKTRUM_CHANNEL_Throttle 0
#define SPEKTRUM_CHANNEL_Aileron 1
#define SPEKTRUM_CHANNEL_Elevator 2
#define SPEKTRUM_CHANNEL_Rudder 3
#define SPEKTRUM_CHANNEL_Gear 4
#define SPEKTRUM_CHANNEL_Aux_1 5
#define SPEKTRUM_CHANNEL_Aux_2 6
#define SPEKTRUM_CHANNEL_Aux_3 7
#define SPEKTRUM_CHANNEL_Aux_4 8
#define SPEKTRUM_CHANNEL_Aux_5 9
#define SPEKTRUM_CHANNEL_Aux_6 10
#define SPEKTRUM_CHANNEL_Aux_7 11
#define SPEKTRUM_NUM_STD_CHANNELS 12
#define SPEKTRUM_CHANNEL_LAST 15

// an example packet (2048)
// 0x01 0xb2 0x81 0x6c 0x3c 0x00 0x1b 0xff 0x44 0x00 0x4c 0x00 0x5c 0x00 0xff 0xff

// Servo Field 1024 Mode
//   This format is used only by DSM2/22ms mode.
//   All other modes use 2048 data.

//   Bits 15-10   Channel ID (6 bits, 0-63, 64 possible values)
//   Bits  9- 0   Servo Position (10 bits, 0-1023, 1024 possible values)
//
#define SPEKTRUM_MASK_1024_CHANNEL_ID 0xFC00u
#define SPEKTRUM_MASK_1024_SERVO_POS  0x03FFu

// Servo Field 2048 Mode
//   This format is used by all protocols except DSM2/22ms mode.
//   Bits    15   Servo Phase (1 bits, 0-1, 2 possible values)
//   Bits 14-11   Channel ID (4 bits, 0-15, 16 possible values)
//   Bits 10- 0   Servo Position (11 bits, 0-2047, 2048 possible values)
#define SPEKTRUM_MASK_2048_CHANNEL_ID 0x7800u
#define SPEKTRUM_MASK_2048_SERVO_POS  0x07FFu

typedef struct spektrum_channel_state {
	bool servo_phase; // only applies to 2048 modes, for 1024 modes it set to false
	int servo_position;
} spektrum_servo_state_t;

typedef struct spektrum_channel_data {
	bool servo_phase; // only applies to 2048 modes, for 1024 modes it set to false
	int channel_id;
	int servo_position;
} spektrum_channel_data_t;

typedef struct spektrum_internal_msg {
	uint8_t fades;
	uint8_t system;
	spektrum_channel_data_t data[SPEKTRUM_NUM_MSG_CHANNELS];
} spektrum_internal_msg_t;

typedef struct spektrum_packet_parser_state {
	spektrum_internal_packet_t packet;
	int buffer_size;
	spektrum_internal_msg_t msg;
	int dropped_bytes;
} spektrum_packet_parser_state_t;

extern char *channel_names[SPEKTRUM_NUM_STD_CHANNELS];

typedef struct spektrum_state {

	uint8_t fades;
	uint8_t system;
	spektrum_servo_state_t channels[SPEKTRUM_NUM_STD_CHANNELS];

	bool connected;
	long num_unexpected_channels;
	long last_msg_ts;
	long max_msg_delay;

} spektrum_state_t;

typedef void (*spektrum_msg_handler)(const spektrum_internal_msg_t *packet, void *context);

void spektrum_packet_to_msg(spektrum_internal_packet_t *packet, spektrum_internal_msg_t *msg);

void spektrum_init(spektrum_packet_parser_state_t *state);

void spektrum_process_data(
	const uint8_t *data, int size,
	spektrum_packet_parser_state_t *state,
	spektrum_msg_handler handler,
	void *context
);

void spektrum_msg_to_state(const spektrum_internal_msg_t *packet, spektrum_state_t *state, long current_time);

void spektrum_reset_state_stats(spektrum_state_t *state);

#ifdef __cplusplus
}
#endif

#endif // _SPEKTRUM_H
