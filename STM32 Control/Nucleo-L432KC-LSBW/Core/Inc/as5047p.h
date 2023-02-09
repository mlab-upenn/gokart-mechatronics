#ifndef _AS5047P_H
#define _AS5047P_H

#include "stm32l4xx_hal.h"

typedef struct as5047p_spi_comm_stats {
	uint32_t num_parity_errors;
	uint32_t num_overruns;
	uint32_t num_unexpected_frames_occurred;
} as5047p_spi_comm_stats_t;

int spi_as5047p_blocking_read(double *angle, as5047p_spi_comm_stats_t *stats);

#endif // _AS5047P_H
