#include "cmsis_os.h"
#include "main.h"
#include "i2c.h"
#include "appI2C.h"
#include <string.h>
 
static osMessageQId I2CRXEventQueue;
static uint8_t I2C1RXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
static osStaticMessageQDef_t I2C1RXEventQueueControlBlock;
osMessageQStaticDef(I2C1RXEventQueue, 1, uint8_t, I2C1RXEventQueueBuffer, &I2C1RXEventQueueControlBlock);

static osMessageQId I2CTXEventQueue;
static uint8_t I2C1TXEventQueueBuffer[ 1 * sizeof( uint8_t ) ];
static osStaticMessageQDef_t I2C1TXEventQueueControlBlock;
osMessageQStaticDef(I2C1TXEventQueue, 1, uint8_t, I2C1TXEventQueueBuffer, &I2C1TXEventQueueControlBlock);

static osMutexId I2CMutex;
static osStaticMutexDef_t I2C1MutexControlBlock;
osMutexStaticDef(I2C1Mutex, &I2C1MutexControlBlock);

static uint8_t I2C1TxBuf[I2C1_TX_BUF_SIZE];

void I2C1Init(void)
{
  I2CRXEventQueue = osMessageCreate(osMessageQ(I2C1RXEventQueue), NULL);
  I2CTXEventQueue = osMessageCreate(osMessageQ(I2C1TXEventQueue), NULL);
  I2CMutex = osMutexCreate(osMutex(I2C1Mutex));
}


uint8_t I2C1RxDataWaitTimed(uint32_t timeout) 
{ 
  osEvent evt;
  evt = osMessageGet(I2CRXEventQueue, timeout);
  return (evt.status == osEventMessage); 
}

uint8_t I2C1TxDataWaitTimed(uint32_t timeout) 
{ 
  osEvent evt;
  evt = osMessageGet(I2CTXEventQueue, timeout);
  return (evt.status == osEventMessage); 
}

void I2C1TxDataWait(void)            
{         
  osMessageGet(I2CTXEventQueue, osWaitForever);
}  

void I2C1RxDataWait(void)            
{         
  osMessageGet(I2CRXEventQueue, osWaitForever);
}

void I2C1CompleteCallback(uint8_t RX)
{
  if(RX == 1)
    osMessagePut(I2CRXEventQueue, 0, 0);
  else
    osMessagePut(I2CTXEventQueue, 0, 0);
}

void I2C1Tx(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size)
{
  osMutexWait(I2CMutex, osWaitForever);
  I2C1TxBuf[0]=reg;
  memcpy(&I2C1TxBuf[1],data,size);
  HAL_I2C_Master_Transmit_IT(&hi2c1, addr, I2C1TxBuf, size+1);     
  I2C1TxDataWait();  
  osMutexRelease(I2CMutex);
}

void I2C1Rx(uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size)
{
  osMutexWait(I2CMutex, osWaitForever);
  HAL_I2C_Master_Transmit_IT(&hi2c1, addr, &reg, 1);
  I2C1TxDataWait();
  HAL_I2C_Master_Receive_IT(&hi2c1, addr,  data, size);
  I2C1RxDataWait();
  osMutexRelease(I2CMutex);
}  

