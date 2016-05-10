/*
The controller should calculate the control signal from the reference
and the current state.
IN: current state, reference
OUT: motor power
*/
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

static bool isInit;

void controllerInit(void)
{
  if(isInit)
    return;

  // Call dependency inits

  // Create task
  xTaskCreate(controllerTask, CONTROLLER_TASK_NAME,
              CONTROLLER_TASK_STACKSIZE, NULL, CONTROLLER_TASK_PRI, NULL);


  isInit = true;
}



static void controllerTask(void* param)
{
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR); // What is this?

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // delay until new ref or state estimation

    // Get error
    e=ref-state;

    // Calculate input (T,tx,ty,tz)
    inputs=K*e;

    // Translate from (T,tx,ty,tz) to motorPowerMi
    //check the book at page 80
    motorPowerM1 = limitThrust();
    motorPowerM2 = limitThrust();
    motorPowerM3 = limitThrust();
    motorPowerM4 = limitThrust();

      }
    }
