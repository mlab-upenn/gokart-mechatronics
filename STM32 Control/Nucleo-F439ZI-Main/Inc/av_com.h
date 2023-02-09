#include "main.h"

extern uint8_t data_frame[29];
extern int current_uart_pos;

void send_vehicle_status(UART_HandleTypeDef *huart, uint8_t message[], int size);

void av_com_start_receiving(UART_HandleTypeDef *huart);

void av_com_stop_receiving(UART_HandleTypeDef *huart);

void av_com_irq_handler(UART_HandleTypeDef *huart);
