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

static bool isInit;
static int status=0;
int FREQ;
static QueueHandle_t *xQueue1;
static QueueHandle_t *xQueue2;
int* pnt;

static int toggle(int var){
  return var?0:1;
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

    // Get reference (from where?)

    // cretate a smooth trajectory for the reference (maybe a derivative?)

    // Tell controller we have a new reference (if we have? do we need to?)

    // Testing
    DEBUG_PRINT("--MODE_SW\n");
    FREQ = status?10:5;
    status = toggle(status);
    pnt = &FREQ;
    xQueueSend( xQueue1,( void * ) &pnt, ( TickType_t ) 1000 );
    xQueueSend( xQueue2,( void * ) &pnt, ( TickType_t ) 1000 );
      }
    }

void mode_switcherInit(QueueHandle_t *q1, QueueHandle_t *q2)
{
  if(isInit)
    return;

  // Call dependency inits
 xQueue1 = q1;
 xQueue1 = xQueueCreate( 1, sizeof( int* ) );
 xQueue2 = q2;
 xQueue2 = xQueueCreate( 1, sizeof( int* ) );
  // Create task
  xTaskCreate(mode_switcherTask, MODE_SW_TASK_NAME,
              MODE_SW_TASK_STACKSIZE, NULL, MODE_SW_TASK_PRI, NULL);


  isInit = true;
}
bool mode_switcherTest(void)
{
  return true;
}
