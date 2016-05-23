/*
The mode switcher changes between two modes. One that includes an integrator and
one that doesn't. At the power on, the integrator is turned on.
*/
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "system.h"
#include "mode_switcher.h"
#include "queue.h"
#include "debug.h"
#include "param.h"
#include "ledseq.h"


static bool isInit;
extern QueueHandle_t xQueue1;
uint16_t no_integ=0;
uint16_t* pnt_integ;


static void integrator_led (void)   // turns the left blue led on or off
{                                   // depending on the mode
  if (no_integ)
    ledSet(LED_BLUE_L,0);
  else
    ledSet(LED_BLUE_L,1);
}


static void mode_switcherTask(void* param)
{
  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(1)); // 500Hz

    xQueueSend( xQueue1,( void * ) &pnt_integ, ( TickType_t ) 1000 );
    integrator_led ();
  }
}

void mode_switcherInit(void)
{
  if(isInit)
    return;

  // Call dependency inits
  pnt_integ = &no_integ;
  // Create task
  xTaskCreate(mode_switcherTask, MODE_SW_TASK_NAME,
              MODE_SW_TASK_STACKSIZE, NULL, MODE_SW_TASK_PRI, NULL);


  isInit = true;
}

bool mode_switcherTest(void)
{
  return true;
}

PARAM_GROUP_START(mode)
PARAM_ADD(PARAM_UINT16, integ_off, &no_integ)
PARAM_GROUP_STOP(mode)
