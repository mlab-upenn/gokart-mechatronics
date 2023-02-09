#ifndef _VESC_H
#define _VESC_H

#include <stdint.h>

int vesc_get_payload_offset(const int payload_length);

int vesc_encode_packet(uint8_t packet[256], int payload_length);

int vesc_set_current(uint8_t packet[256], float current);

int vesc_set_rpm(uint8_t packet[256], int32_t rpm);

#endif // _VESC_H
