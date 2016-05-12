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

static bool isInit;
static QueueHandle_t* xQueue1;
static int* FREQ;

static int toggle(int var){
  return var?0:1;
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
    vTaskDelayUntil(&lastWakeTime, F2T(*FREQ)); // delay until new ref or state estimation
    xQueueReceive( xQueue1, &( FREQ ), ( TickType_t ) 1000 ); // if/else needed?
    // Get error
    //e=ref-state;

    // Calculate input (T,tx,ty,tz)
    //inputs=K*e;

    // Translate from (T,tx,ty,tz) to motorPowerMi

    //check the book at page 80
    /*motorPowerM1 = limitThrust();
    motorPowerM2 = limitThrust();
    motorPowerM3 = limitThrust();
    motorPowerM4 = limitThrust();

    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);
    motorsSetRatio(MOTOR_M1, motorPowerM1);*/


    // For this week we just toggle some leds
    ledSet(ERR_LED1,ledstatus);
    ledSet(ERR_LED2,ledstatus);
    ledstatus = toggle(ledstatus);
      }
}


void controllerInit(QueueHandle_t *q1)
{
  if(isInit)
    return;

  // Call dependency inits

  xQueue1 = q1;
  // Create task
  xTaskCreate(controllerTask, CONTROLLER_TASK_NAME,
              CONTROLLER_TASK_STACKSIZE, NULL, CONTROLLER_TASK_PRI, NULL);

  isInit = true;
}

bool controllerTest(void)
{
  return true;
}
