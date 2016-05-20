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
#include "queue.h"
// From Crazyflie
#include "commander.h"
#include "led.h"
#include "config.h"
#include "system.h"
#include "ref_generator.h"
#include "log.h"
#include "debug.h"

#define ANGLE 20;
//#define Z_SPEED 10;

static bool isInit;
extern QueueHandle_t xQueue2;
int* FREQ;


// euler angles from sensorFusion
//static float eulerRollActual;
//static float eulerPitchActual;
//static float eulerYawActual;
// euler angles from user
//static float eulerRollDesired;
//static float eulerPitchDesired;
//static float eulerYawDesired;

static void ref_generatorTask(void* param)
{
//  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR); // OMG WHAT IS THIS

  //Wait for the system to be fully started
  systemWaitStart();

//  lastWakeTime = xTaskGetTickCount ();

  //int ledstatus = 0; // for the ledToggle

  while(1)
  {
    //vTaskDelayUntil(&lastWakeTime, F2T(*FREQ)); // delay until next
    xQueueReceive( xQueue2, &( FREQ ), portMAX_DELAY ); // if/else needed?
    // Get reference (from where?) (from commander)
    //commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired); // sets the desired to what the user want

    // cretate a smooth trajectory for the reference (maybe a derivative?)

    // Tell controller we have a new reference (if we have? do we need to?)

    // For this week we just toggle some leds
    //DEBUG_PRINT("ref_gen got message: %d\n",*FREQ);
    //ledSet(CHG_LED,ledstatus);
    //ledstatus = toggle(ledstatus);

      }
}

// static float ref_generatorSetAngle (float value)
// {
// // returns an angle of +1 degree or -1 degree, if the commander input is positive, respectively negative
//   if (value < -10) {
//     return -ANGLE;
//   }
//   else if (value > 10) {
//     return ANGLE;
//   }
//   else {
//     return 0;
//   }
// }

// static float ref_generatorSetZ (uint16_t thrust)
// {
// // returns a speed in Z of +0.05 or -0.05 if the commander input is positive, repectively negative
//   if (thrust < -5) {
//     return -Z_SPEED;
//   }
//   else if (thrust > 5) {
//     return Z_SPEED;
//   }
//   else {
//     return 0;
//   }
// }


static float ref_generatorSetZThrust (uint16_t thrust)
{
  if (thrust > 0) {
    return (float) thrust*0.8;
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

  xRef[0]=rollRef;  //ref_generatorSetAngle(rollRef);
  xRef[1]=pitchRef; //ref_generatorSetAngle(pitchRef);
  //xRef[2]=90; // yaw always zero  --> might still be changed if desired
  //xRef[3]=0; // we might not need these since they always are 0
  //xRef[4]=0;
  //xRef[5]=0;
  //xRef[6]=0;//ref_generatorSetZ(thrust);
  //xRef[7]=0;//ref_generatorSetZ(thrust);

  return ref_generatorSetZThrust(thrust);
}

/* Loggable variables */
LOG_GROUP_START(ref)
LOG_ADD(LOG_INT8, ref_freq, &FREQ)
LOG_GROUP_STOP(ref)
