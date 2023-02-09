#include <stdio.h>
#include <string.h>
#include <endianness.h>
#include "vesc.h"
#include "vesc_crc.h"
#include "utils.h"

#include "vesc_datatypes.h"

// see https://github.com/vedderb/bldc/blob/master/datatypes.h
// see https://github.com/RollingGecko/VescUartControl/blob/master/VescUart.cpp
// see http://vedder.se/2015/10/communicating-with-the-vesc-using-uart/
// see https://forums.parallax.com/discussion/171979/vesc-uart-control-with-spin

int vesc_get_payload_offset(const int payload_length) {
	if (payload_length <= 256) {
		return 2;
	} else {
		return 3;
	}
}

int vesc_encode_packet(uint8_t packet[256], int payload_length) {

	uint8_t *payload;
	uint8_t *footer;

	int size = payload_length;

	if (payload_length <= 256) {
		packet[0] = 2;
		packet[1] = (uint8_t) payload_length;
		size += 5; // 2 + 3
		payload = &packet[2];
		footer = &packet[2 + payload_length];
	} else {
		packet[0] = 3;
		packet[1] = (uint8_t) (payload_length >> 8);
		packet[2] = (uint8_t) (payload_length & 0xFF);
		size += 6; // 3 + 3
		payload = &packet[3];
		footer = &packet[3 + payload_length];
	}

	uint16_t checksum = crc16(payload, payload_length);

	*(footer++) = (uint8_t) (checksum >> 8);
	*(footer++) = (uint8_t) (checksum & 0xFF);
	*(footer++) = 3;

	return size;
}

int vesc_set_current(uint8_t packet[256], float current) {

	int payload_length = 5;
	uint8_t *payload = &packet[vesc_get_payload_offset(5)];

	payload[0] = COMM_SET_CURRENT;
	int32_t *current_field = (int32_t *) &payload[1];

	*current_field = htonl((int32_t) (current * 1000));

	return vesc_encode_packet(packet, payload_length);

}

int vesc_set_rpm(uint8_t packet[256], int32_t rpm) {

	int payload_length = 5;
	uint8_t *payload = &packet[vesc_get_payload_offset(5)];

	payload[0] = COMM_SET_RPM;
	int32_t *rmp_field = (int32_t *) &payload[1];

	*rmp_field = htonl(rpm);

	return vesc_encode_packet(packet, payload_length);

}
