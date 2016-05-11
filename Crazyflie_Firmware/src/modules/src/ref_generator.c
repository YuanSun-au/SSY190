/*
The reference generator should give a smooth trajectory for the QUAD
given the wanted reference and the current position
IN: reference from user,current state
OUT: reference to controller
*/
// From C
#include <math.h>
// From FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
// From Crazyflie
#include "commander.h"
#include "ledseq.h"
#include "config.h"
#include "system.h"
#include "ref_generator.h"

static bool isInit;

// euler angles from sensorFusion
static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
// euler angles from user
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;

static int toggle(int var){
  return var?0:1;
}

static void ref_generatorTask(void* param)
{
  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR); // OMG WHAT IS THIS

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  int ledstatus = 0; // for the ledToggle

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(FREQ)); // delay until next
   xSemaphoreTake( xSemaphore, ( TickType_t ) 10 ); // Take the semaphore (block all other)

    // Get reference (from where?) (from commander)
    commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired); // sets the desired to what the user want

    // cretate a smooth trajectory for the reference (maybe a derivative?)

    // Tell controller we have a new reference (if we have? do we need to?)

    // For this week we just toggle some leds
    ledSet(LED_RED_R,ledstatus);
    ledstatus = toggle(ledstatus);
    xSemaphoreGive(xSemaphore); // release the sem.

      }
    }

void ref_generatorInit(void)
{
  if(isInit)
    return;

  // Call dependency inits

  // Create task
  xTaskCreate(ref_generatorTask, REF_GEN_TASK_NAME,
              REF_GEN_TASK_STACKSIZE, NULL, REF_GEN_TASK_PRI, NULL);


  isInit = true;
}
