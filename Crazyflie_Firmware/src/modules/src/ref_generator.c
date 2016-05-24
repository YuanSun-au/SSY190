/*
The reference generator receives the input from the commander and passes on the
relevant values to the controller
IN: reference from user,current state
OUT: reference to controller
*/

// From C
#include <math.h>
// From FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
// From Crazyflie
#include "commander.h"
#include "config.h"
#include "system.h"
#include "ref_generator.h"
#include "debug.h"
// multiplicator on the received thrust
#define THRUST_M 0.808715820312500

static bool isInit;

static void ref_generatorTask(void* param)
{
  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(10)); // delay until next

  }
}


static float ref_generatorSetZThrust (uint16_t thrust)
{
  if (thrust > 0) {
    return (float) thrust*THRUST_M;
  }
  else {
    return 0;
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

bool ref_generatorTest(void)
{
  return true;
}

float ref_generatorExtIn (float* xRef)
{
  float rollRef, pitchRef, yawRef;
  uint16_t thrust;

  commanderGetRPY (&rollRef, &pitchRef, &yawRef);
  commanderGetThrust (&thrust);

  xRef[0]=rollRef*1.3;  //ref_generatorSetAngle(rollRef);
  xRef[1]=pitchRef*1.3; //ref_generatorSetAngle(pitchRef);

  return ref_generatorSetZThrust(thrust);
}
