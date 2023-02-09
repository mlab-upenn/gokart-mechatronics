#include "main.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include "appUSART.h"
#include "appLSM9DS1.h"
#include "console.h"

static char lineBuffer[1024];
static char outBuffer[1024];

static uint8_t lineIndex = 0;
static uint8_t streamActiveFlag = 0;

static float acceleration_mg[3];
static float angular_rate_mdps[3];
static float magnetic_field_mgauss[3];

static void processLine(void);
static void consoleTimerCallback(void const *arg);
static osTimerId consoleTimer;
static osStaticTimerDef_t consoleTimerCB;

osTimerStaticDef(ConsoleTimer, consoleTimerCallback, &consoleTimerCB);

static void consoleTimerCallback(void const *arg)
{
  (void) arg;

  GetIMUReading(acceleration_mg, angular_rate_mdps, magnetic_field_mgauss);

  sprintf((char *)outBuffer,
          "IMU - [mg]:%4.2f\t%4.2f\t%4.2f\t[mdps]:%4.2f\t%4.2f\t%4.2f\r\n",
          acceleration_mg[0], acceleration_mg[1], acceleration_mg[2],
          angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
  USART1TxStr((char *)outBuffer);

  sprintf(outBuffer, "MAG - [mG]:%4.2f\t%4.2f\t%4.2f\r\n",
          magnetic_field_mgauss[0], magnetic_field_mgauss[1],
          magnetic_field_mgauss[2]);
  USART1TxStr(outBuffer);

}

static void processLine(void)
{
  switch(lineBuffer[0])
  {
    case 'B':
    	if(streamActiveFlag==0)
    	{
    	  streamActiveFlag=1;
    	  USART1TxStr("Beginning Streaming\r\n");
    	  osTimerStart(consoleTimer, 100);
    	}
	    break;
    case 'E':
    	if(streamActiveFlag==1)
    	{
      	  streamActiveFlag=0;
    	  osTimerStop(consoleTimer);
    	  USART1TxStr("Ending Streaming\r\n");
    	}
    	break;
    case '?':
    	if(streamActiveFlag==0)
    	{
          USART1TxStr("B - Begin streaming\r\nE - End streaming\r\n? - This menu\r\n");
    	}
    default:
    	break;
  }
}

void console(void)
{
  uint8_t data;

  consoleTimer = osTimerCreate(osTimer(ConsoleTimer), osTimerPeriodic, NULL);
  IMUInit();
  for(;;)
  {
	USART1RxDataWait();
    while(USART1Rx(&data))
    {
      if((data=='\n')||(data=='\r'))
      {
    	lineBuffer[lineIndex]='\0';
        processLine();
        lineIndex=0;
      }
      else
      {
        lineBuffer[lineIndex]=data;
        lineIndex++;
      }
    }
    osDelay(1);
  }
}
