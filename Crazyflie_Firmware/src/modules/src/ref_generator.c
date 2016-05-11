/*
The reference generator should give a smooth trajectory for the QUAD
given the wanted reference and the current position
IN: reference from user,current state
OUT: reference to controller
*/
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static bool isInit;

void ref_generatorInit(void)
{
  if(isInit)
    return;

  // Call dependency inits

  // Create task
  xTaskCreate(controllerTask, CONTROLLER_TASK_NAME,
              CONTROLLER_TASK_STACKSIZE, NULL, CONTROLLER_TASK_PRI, NULL);


  isInit = true;
}


static void ref_generatorTask(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // delay until new ref

    // Get reference (from where?)

    // cretate a smooth trajectory for the reference (maybe a derivative?)

    // Tell controller we have a new reference (if we have? do we need to?)

      }
    }
