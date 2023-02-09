#include "cmsis_os.h"
#include "main.h"
#include <stdio.h>
#include <string.h>

#include "dma.h"
#include "usart.h"
#include "appUSART.h"
#include "fifo.h"

/*------------------------------------------------------------------------------
  Overview
   
  This driver utilizes an interrupt driven transmit and a DMA/interrupt driven 
  receive.  
 
  TX:  
    Data is inserted into tx fifo, the tx isr pulls data from the tx fifo and sends.

  RX:
    Received data is stored via a DMA transaction to the DMA rx memory.  Note 
    that the rx DMA prevents loss of data as long as the data is removed from
    the rx DMA fifo (via USART1Getchar()) within a reasonable amount of time.
    Max rx service latency must be size of DMA rx memory / (baud/10).  
      Example:
        FIFO_RX_SIZE / (115200/10) = 100/(115200/10) = 8.68ms  
------------------------------------------------------------------------------*/

static FIFO__S          fifoTx;
uint8_t          	    Fifo_Rx[FIFO_RX_SIZE];
static uint8_t          Fifo_Tx[FIFO_TX_SIZE];
static uint32_t         tail = 0;

// local functions
static BOOL USART1RxDataAvailable(void);

static osMessageQId USARTRXEventQueue;
static uint8_t USART1RXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
static osStaticMessageQDef_t USART1RXEventQueueControlBlock;

osMessageQStaticDef(USART1RXEventQueue, 1, uint8_t, USART1RXEventQueueBuffer, &USART1RXEventQueueControlBlock);

static osMutexId USARTMutex;
static osStaticMutexDef_t USART1MutexControlBlock;

osMutexStaticDef(USART1Mutex, &USART1MutexControlBlock);


/*------------------------------------------------------------------------------
  USART1Init()
------------------------------------------------------------------------------*/
void USART1Init(void)
{
  static  BOOL              initializedFlag   = FALSE;

  if(initializedFlag) { return;                 }
  else                { initializedFlag = TRUE; }

  USARTRXEventQueue = osMessageCreate(osMessageQ(USART1RXEventQueue), NULL);

  USARTMutex = osMutexCreate(osMutex(USART1Mutex));
  memset(Fifo_Rx,0,sizeof(Fifo_Rx));
  memset(Fifo_Tx,0,sizeof(Fifo_Tx));
  FifoInit(&fifoTx,Fifo_Tx,FIFO_TX_SIZE);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Fifo_Rx, FIFO_RX_SIZE);
}

void RTOS_SignalIfRxDataAvailable(void)
{
  if(USART1RxDataAvailable())
  { 
    osMessagePut(USARTRXEventQueue, 0, 0);
  }
}

#define RTOS_Use()    osMutexWait(USARTMutex, osWaitForever)
#define RTOS_Unuse()  osMutexRelease(USARTMutex)

BOOL USART1RxDataWaitTimed(uint32_t timeout)
{
  osEvent evt;
  evt = osMessageGet(USARTRXEventQueue, timeout);
  return (evt.status == osEventMessage);
}

void USART1RxDataWait(void)
{
  osMessageGet(USARTRXEventQueue, osWaitForever);
}

 
/*------------------------------------------------------------------------------
  USART1RxDataAvailable()
------------------------------------------------------------------------------*/
static BOOL USART1RxDataAvailable(void)
{
  // returns FALSE if there is no new data in DMA memory 
  // (notes: circular DMA mode; NDTR decrements with every byte received, then rolls over)
  return (tail != (sizeof(Fifo_Rx) - APP_DMA_GetDataLength(DMA1, 0x00000004U )));
}
 
/*------------------------------------------------------------------------------
  USART1Rx()
------------------------------------------------------------------------------*/
BOOL USART1Rx(uint8_t* data)
{
  BOOL dataAvailableFlag = FALSE;
 
  RTOS_Use();
 
  if(USART1RxDataAvailable())
  {
    *data = Fifo_Rx[tail];
    tail = (tail < (sizeof(Fifo_Rx) - 1)) ? tail+1 : 0;
    dataAvailableFlag = TRUE;
  }
 
  RTOS_Unuse();
 
  return dataAvailableFlag;
}
 
/*------------------------------------------------------------------------------
  USART1Tx()
 
  Returns TRUE for success and FALSE if tx fifo is full.
------------------------------------------------------------------------------*/
BOOL USART1Tx(uint8_t data)
{
  BOOL successFlag = FALSE;
   
  RTOS_Use();
   
  // try to put data into tx fifo
  if(FifoPut(&fifoTx, data))
  {
    // generate a tx interrupt (tx isr will pull data from fifo and send)
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_TXE);

    successFlag = TRUE;
  }
 
  RTOS_Unuse();
 
  return successFlag;
}
 
 
/*------------------------------------------------------------------------------
  USART1TxStr()
 
  Returns TRUE for success and FALSE if fifo is full.
------------------------------------------------------------------------------*/
BOOL USART1TxStr(char* str)
{
  if(str == 0) return TRUE;
 
  while(*str != 0)
  {
    if(!USART1Tx((uint8_t)*str)) { return FALSE; }
    str++;
  }
 
  return TRUE;
}
 
void AppUSART1_IRQHandler(void)
{
  uint8_t data;
 
  //
  // TX
  //
  // check if interrupt was caused by tx data reg empty
  if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TXE) && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TXE))
  {   
    // check for data available in tx fifo
    // note - TXE interrupt is cleared when data is sent
    if (FifoGet(&fifoTx, &data))
    {
      USART1->TDR = (uint16_t)(data & 0x1FFUL);
    }
    // otherwise, manually disable interrupt 
    else
    {      
      __HAL_UART_DISABLE_IT(&huart1, UART_IT_TXE);
    }
  }
  
  if(__HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_TC) && __HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC))
  {
    /* Clear TC flag */
	 __HAL_UART_CLEAR_FLAG(&huart1, UART_CLEAR_TCF);
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  RTOS_SignalIfRxDataAvailable();
}
