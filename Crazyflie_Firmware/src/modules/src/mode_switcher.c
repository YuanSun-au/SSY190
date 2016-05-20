/*
This mode switcher should decide if the mode is FLY (controller on)
or STOP (controller off)
*/
#include "FreeRTOS.h"
#include "task.h"
#include "config.h"
#include "system.h"
#include "mode_switcher.h"
#include "queue.h"
#include "debug.h"
#include "param.h"

//#define Nstates 6

static bool isInit;
//static int status=0;
//float ref_mode[Nstates] = {0,0,0,0,0,0,0,0};
extern QueueHandle_t xQueue1;
//float* float_ref_pnt;
//extern QueueHandle_t xQueue2;
uint16_t no_integ=0;
uint16_t* pnt_integ;

//static int toggle(int var){
//  return var?0:1;
//}

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
    //xQueueSend( xQueue2,( void * ) &pnt, ( TickType_t ) 1000 );
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
PARAM_ADD(PARAM_UINT16, integrator, &integ)
PARAM_GROUP_STOP(mode)
