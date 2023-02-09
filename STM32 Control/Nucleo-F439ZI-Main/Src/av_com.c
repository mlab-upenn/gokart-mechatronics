#include "av_com.h"

uint8_t data_frame[29];
int current_uart_pos = 0;

void send_vehicle_status(UART_HandleTypeDef *huart, uint8_t message[], int size){
	HAL_UART_Transmit(huart, message, size, 14);// Sending in normal mode
}

void av_com_start_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

void av_com_stop_receiving(UART_HandleTypeDef *huart) {
	__HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
	__HAL_UART_DISABLE_IT(huart, UART_IT_IDLE);
}

void av_com_irq_handler(UART_HandleTypeDef *huart) {
	// If data overrun, clear the overrun flag
	// Clear the data reception array, and reset to start

	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE)) {
		current_uart_pos = 0;
		memset(data_frame, '\0', sizeof(data_frame));

		__HAL_UART_CLEAR_OREFLAG(huart);
		return;
	}

	// receive the data and reset to start upon completion
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET) {
		uint8_t data = (uint8_t) (huart->Instance->DR & (uint8_t) 0x00FF);
		data_frame[current_uart_pos] = data;
		current_uart_pos ++;

		if (current_uart_pos == 29){
			current_uart_pos = 0;
		}

		return;
	}

	// If dataline idle, clear the idle flag
	// If we are somehow in the middle of receiving an array
	// clear the data and reset to start so we don't get bit shift in the next cycle
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) == SET) {
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		if(current_uart_pos != 0){
			current_uart_pos = 0;
			memset(data_frame, '\0', sizeof(data_frame));
		}
		return;
	}
};
