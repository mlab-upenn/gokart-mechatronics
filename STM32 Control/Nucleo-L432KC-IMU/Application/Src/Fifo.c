#include "main.h"
#include "Fifo.h"
 
// initializes fifo
void FifoInit(FIFO__S* fifo, uint8_t* buf, uint32_t bufSize)
{
  fifo->buf     = buf;
  fifo->bufSize = bufSize;
  fifo->head    = 0;   
  fifo->tail    = 0;   
}
 
// returns TRUE if successful
BOOL FifoPut(FIFO__S* fifo, uint8_t data)
{
   BOOL     fifoPutSuccessfulFlag=FALSE;
   uint32_t headTemp             =0;
 
   // verify fifo
   if(fifo!=0)
   {
     // check if fifo is full
      headTemp=fifo->head;
      headTemp=(headTemp<(fifo->bufSize-1))?(headTemp+1):0;
      if(headTemp!=fifo->tail)
      {
         fifo->buf[fifo->head]=data;      // store data in fifo
         fifo->head           =headTemp;  // adjust head
         fifoPutSuccessfulFlag=TRUE;      // indicate fifo insert successful
      }
   }
        
   return fifoPutSuccessfulFlag;
}
 
// returns TRUE if successful
BOOL FifoGet(FIFO__S* fifo, uint8_t* data)
{
   BOOL dataAvailableFlag=FALSE;
    
   // verify fifo
   if(fifo!=0)
   {
      // check if fifo has data
      if(fifo->head!=fifo->tail)
      {
         *data=fifo->buf[fifo->tail];                             // store data
         fifo->tail=(fifo->tail<(fifo->bufSize-1))?(fifo->tail+1):0;  // adjust tail
         dataAvailableFlag=TRUE;                                  // indicate data available
      }        
   }
    
   return dataAvailableFlag;
}
