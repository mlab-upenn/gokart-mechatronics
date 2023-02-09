#pragma once
#include "cmsis_os.h"

#define FIFO_RX_SIZE            1024
#define FIFO_TX_SIZE            1024

void USART1Init    (void);
BOOL USART1Rx      (uint8_t* data); // returns FALSE is no data is available
BOOL USART1Tx      (uint8_t  data); // returns FALSE if tx fifo is full
BOOL USART1TxStr   (char*    str);  // returns FALSE if tx fifo is full
BOOL USART1TxFull  (void);
BOOL USART1RxLine(char* str, uint32_t length);
BOOL USART1RxDataWaitTimed(uint32_t timeout); // returns FALSE if timed out
void USART1RxDataWait     (void);            // pends until data is available
void AppUSART1_IRQHandler(void);
