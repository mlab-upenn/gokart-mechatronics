#include <stdio.h>
#include <stdbool.h>
#include <math.h>

//#include "app.h"
#include "as5047p.h"
#include "terminal.h"
#include "utils.h"

extern SPI_HandleTypeDef hspi1;

#define SPI1_NSS_MANUAL_Pin GPIO_PIN_4
#define SPI1_NSS_MANUAL_GPIO_Port GPIOA

/**
 * Checks if the given value (16 bits) has even parity,
 * i.e., the number of 1s (in the binary representation) is even.
 * @see https://stackoverflow.com/a/21618038
 * @param x data (16 bits)
 * @return true if if the given value (16 bits) has even parity
 */
static bool is_parity_even_16(uint16_t x) {
	x ^= x >> 8;
	x ^= x >> 4;
	x ^= x >> 2;
	x ^= x >> 1;
	return (~x) & 1;
}

int spi_as5047p_blocking_read(double *angle, as5047p_spi_comm_stats_t *stats) {

	// CSn signal (SS) (chip select)
	// logical zero means active
	// uint32_t t1 = htim1.Instance->CNT;
	HAL_GPIO_WritePin(SPI1_NSS_MANUAL_GPIO_Port, SPI1_NSS_MANUAL_Pin, GPIO_PIN_RESET);

	// note that there needs to be at least 50 ns delay between CSn falling edge and first clock rising edge
	// but no need to add explict delay here (implicit delay caused by our MCU/bus speed adds ~ 1000 ns)

	__HAL_SPI_ENABLE(&hspi1);

	// wait for 16 bits
	while (SPI_CHECK_FLAG(hspi1.Instance->SR, SPI_FLAG_RXNE) != SET);

	bool overrun = SPI_CHECK_FLAG(hspi1.Instance->SR, SPI_FLAG_OVR) == SET;

	__HAL_SPI_DISABLE(&hspi1);

	// do the correct disable procedure for receive only mode (see the Reference Manual)
	while (SPI_CHECK_FLAG(hspi1.Instance->SR, SPI_FLAG_BSY) != RESET);
	int num_frames = 0;
	uint16_t frame;
	while (__HAL_SPI_GET_FLAG(&hspi1, SPI_FLAG_FRLVL) != SPI_FRLVL_EMPTY) {
		frame = (uint16_t) hspi1.Instance->DR;
		num_frames++;
	}

	// note that there needs to be at least 50 ns delay between CSn falling edge and first clock rising edge
	// but no need to add explict delay here

	// deselect device (logical high means inactive)
	HAL_GPIO_WritePin(SPI1_NSS_MANUAL_GPIO_Port, SPI1_NSS_MANUAL_Pin, GPIO_PIN_SET);
	// uint32_t t2 = htim1.Instance->CNT;
	// uint32_t diff = t2 - t1;
	// = ~ 9 us according to the TIM1 with 1 us resolution
	// = ~ 7.8 us is the time diff between CSn falling and rising edge (setting SPI1_NSS_MANUAL_Pin RESET and SET)

	// printf("diff = %lu" nl, diff);

	// save stats (for debugging)

	if (overrun) {
		stats->num_overruns++;
	}

	if (num_frames != 1) {
		stats->num_unexpected_frames_occurred++;
		if (num_frames == 0) {
			return 1;
		}
	}

	// check parity
	bool parity_ok = is_parity_even_16(frame);

	if (!parity_ok) {
		stats->num_parity_errors++;
		return 2;
	}

	// extract angle raw value (0 - 0x3FFF)
	uint16_t raw_value = 0x3FFF & frame;

	// convert to degrees (0 - 360)
	*angle = ((double) raw_value * 360) / 0x3FFF;

	return 0;

}
