#pragma once

//
// I2C1
//

#define I2C1_TX_BUF_SIZE 100
void I2C1Init(void);
void I2C1Rx               (uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size);
void I2C1Tx               (uint8_t addr, uint8_t reg, uint8_t* data, uint16_t size);
void I2C1CompleteCallback (uint8_t);
uint8_t I2C1RxDataWaitTimed(uint32_t timeout); // returns FALSE if timed out
void I2C1RxDataWait     (void);             // pends until data is available

