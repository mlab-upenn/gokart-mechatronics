#include "stm32f4xx_hal.h"

extern uint8_t drive_msg[25];
extern uint8_t current_pos;

void uart_serial_start(UART_HandleTypeDef *huart);

void uart_serial_stop(UART_HandleTypeDef *huart);

void uart_serial_irq_handler(UART_HandleTypeDef *huart);
