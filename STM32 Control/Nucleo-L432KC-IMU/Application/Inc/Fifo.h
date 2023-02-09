#pragma once

typedef struct
{
  uint8_t*  buf;
  uint32_t  bufSize;
  uint32_t  head;
  uint32_t  tail;
} FIFO__S;
 
// initializes fifo
void FifoInit(FIFO__S* fifo, uint8_t* buf, uint32_t bufSize);
 
// returns TRUE if successful
BOOL FifoPut(FIFO__S* fifo, uint8_t data);
 
// returns TRUE if successful
BOOL FifoGet(FIFO__S* fifo, uint8_t* data);
