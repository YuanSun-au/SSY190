/*
The controller should calculate the control signal from the reference
and the current state.
IN: current state, reference
OUT: motor power
*/
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "ledseq.h"
#include "queue.h"

#include "config.h"
#include "system.h"
#include "controller.h"
#include "debug.h"

#DEFINE Nstates 12;
#DEFINE Ninputs 4;
static float[Ninputs][Nstates] K =
{
{0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,6.4690507959,0.0000000000,0.0000000000,0.6466351359},
{0.0161735793,0.0000000000,0.0000000000,0.0035197525,0.0000000000,0.0000000000,0.0000000000,0.0036164069,0.0000000000,0.0000000000,0.0003455295,0.0000000000},
{0.0000000000,0.0166489318,0.0000000000,0.0000000000,0.0036232004,0.0000000000,-0.0037226955,0.0000000000,0.0000000000,-0.0003556848,0.0000000000,0.0000000000},
{0.0000000000,0.0000000000,0.0054216565,0.0000000000,0.0000000000,0.0054433432,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000},
};

static bool isInit;
extern QueueHandle_t xQueue1;
int* REF;

static int toggle(int var){
  return var?0:1;
}

static float[] ctrlCalc(float[Nstates] ref,float[Nstates] states)
// Calculates control signal given states and state references
{
  float[Ninputs] u_k;
  for(int i=0;i<Ninputs;i++)
  {
    for(int j=0;j<Nstates;j++)
    {
      u_k[i]=u_k[i]+K[i][j]*(ref[j]-states[j]);
    }
  }
  return u_k;
}
static float[] Torque2Thrust(float[Ninputs] inputs)
{
  float[Ninputs] thrusts;
  //predefine the Transformation matrix
  return thrusts;
}

static void controllerTask(void* param)
{
  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR); // What is this?

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  int ledstatus = 0; // for the ledToggle

  while(1)
  {
    //vTaskDelayUntil(&lastWakeTime, F2T(*FREQ)); // delay until new ref or state estimation
    if(xQueueReceive( xQueue1, &( REF ),( TickType_t ) 1000 ))
      { // if/else needed?
    // Get ref and sensor
    // ref comes in a queue from ref_generator
    // todo: make REF into a vector


    // Calculate input (T,tx,ty,tz)
    float[Ninputs] inputs = ctrlCalc(u,x); // Do not redefine...

    // Translate from (T,tx,ty,tz) to motorPowerMi
    //check the book at page 80 (remember to rotate to the X-formation)
    float[Ninputs] ThrustVector = Torque2Thrust(inputs);
    motorPowerM1 = limitThrust(ThrustVector[0]);
    motorPowerM2 = limitThrust(ThrustVector[1]);
    motorPowerM3 = limitThrust(ThrustVector[2]);
    motorPowerM4 = limitThrust(ThrustVector[3]);

    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);


    // For this week we just toggle some leds
      DEBUG_PRINT("--controller got %d\n",*FREQ);
      ledSet(CHG_LED,ledstatus);
      ledstatus = toggle(ledstatus);
    }
  }
}


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

bool controllerTest(void)
{
  return true;
}
