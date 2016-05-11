/*
This mode switcher should decide if the mode is FLY (controller on)
or STOP (controller off)
*/
#include "FreeRTOS.h"
#include "task.h"

static bool isInit;

void mode_switcherInit(void)
{
  if(isInit)
    return;

  // Call dependency inits

  // Create task
  xTaskCreate(mode_switcherTask, MODE_SW_TASK_NAME,
              MODE_SW_TASK_STACKSIZE, NULL, MODE_SW_TASK_PRI, NULL);


  isInit = true;
  int status = 0;
}

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
    vTaskDelayUntil(&lastWakeTime, F2T(0.5)); // 500Hz
    xSemaphoreTake( xSemaphore ); // Take the semaphore (block all other)

    // Get reference (from where?)

    // cretate a smooth trajectory for the reference (maybe a derivative?)

    // Tell controller we have a new reference (if we have? do we need to?)

    // Testing
    FREQ = status?10:5;
    status = toggle(status);
    xSemaphoreGive(xSemaphore); // release the sem.
      }
    }
