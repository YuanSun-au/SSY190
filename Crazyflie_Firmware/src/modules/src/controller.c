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
#include "motors.h"
#include "sensfusion6.h"
#include "imu.h"
#include "num.h"
#include "attitude_controller.h"
#include "position_estimator.h"
#include "log.h"

#include "pm.h"
#include "commander.h"

#define Nstates 12
#define Ninputs 4


static float K[Ninputs][Nstates] =
{
{0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,6.4690507959,0.0000000000,0.0000000000,0.6466351359},
{0.0161735793,0.0000000000,0.0000000000,0.0035197525,0.0000000000,0.0000000000,0.0000000000,0.0036164069,0.0000000000,0.0000000000,0.0003455295,0.0000000000},
{0.0000000000,0.0166489318,0.0000000000,0.0000000000,0.0036232004,0.0000000000,-0.0037226955,0.0000000000,0.0000000000,-0.0003556848,0.0000000000,0.0000000000},
{0.0000000000,0.0000000000,0.0054216565,0.0000000000,0.0000000000,0.0054433432,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000,0.0000000000},
};

static float b,d,k; // insert values here...
estimate_t pos;
float speedZ;
static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg

static bool isInit;
extern QueueHandle_t xQueue1;
int* REF;

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)
static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

float u[Ninputs];
float x[Nstates] = {0,0,0,0,0,0,0,0,0,0,0,0};
float ref[Nstates] = {0,0,0,0,0,0,0,0,0,0,0,0};
static float u_k[Ninputs];
static float thrusts[Ninputs];

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

static void ctrlCalc(float ref[Nstates],float states[Nstates])
// Calculates control signal given states and state references
// must return a pointer
{
  int i;
  for(i=0;i<Ninputs;i++)
  {
    int j;
    for(j=0;j<Nstates;j++)
    {
      //u_k[i]=u_k[i]+K[i][j]*(ref[j]-states[j]);
      u_k[i]=K[i][j]*(ref[j]-states[j]);
    }
  }
}

static void Torque2Thrust(float inputs[Ninputs])
// must return a pointer
{
  //predefine b,d,k
thrusts[0] = -1/(4*b)*inputs[0] -1.4142/(4*b*d)*inputs[1] + 1.4142/(4*b*d)*inputs[2] + 1/(4*k)*inputs[3];
thrusts[1] = -1/(4*b)*inputs[0] -1.4142/(4*b*d)*inputs[1] -1.4142/(4*b*d)*inputs[2] -1/(4*k)*inputs[3];
thrusts[2] = -1/(4*b)*inputs[0] + 1.4142/(4*b*d)*inputs[1] -1.4142/(4*b*d)*inputs[2] + 1/(4*k)*inputs[3];
thrusts[3] = -1/(4*b)*inputs[0] + 1.4142/(4*b*d)*inputs[1] + 1.4142/(4*b*d)*inputs[2] -1/(4*k)*inputs[3];
}

static void controllerTask(void* param)
{
  uint32_t lastWakeTime;

//  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR); // What is this?

  //Wait for the system to be fully started
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    imu9Read(&gyro, &acc, &mag);

    //if(xQueueReceive( xQueue1, &( REF ),( TickType_t ) 1000 ))
    if( imu6IsCalibrated() )
    { // if/else needed?
      // Get ref and sensor
      // ref comes in a queue from ref_generator
      // todo: make REF into a vector

      sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, 1/IMU_UPDATE_FREQ);
      sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
      positionUpdateVelocity(sensfusion6GetAccZWithoutGravity(acc.x,acc.y,acc.z), 1/IMU_UPDATE_FREQ);
      positionEstimate(&pos, (float)(0), 1/IMU_UPDATE_FREQ);
      velocityEstimateZ(&x[5]); //x6 = dotZ?
      x[2]=pos.position.z;
      x[6]=eulerRollActual;
      x[7]=eulerPitchActual;
      x[8]=eulerYawActual;
      x[9]=gyro.x;
      x[10]=gyro.y;
      x[11]=gyro.z;

      // Calculate input (T,tx,ty,tz)
      ctrlCalc(ref, x); // Do not redefine...

      // Translate from (T,tx,ty,tz) to motorPowerMi
      Torque2Thrust(u_k);

      motorPowerM1 = limitThrust((uint32_t)thrusts[0]);
      motorPowerM2 = limitThrust((uint32_t)thrusts[1]);
      motorPowerM3 = limitThrust((uint32_t)thrusts[2]);
      motorPowerM4 = limitThrust((uint32_t)thrusts[3]);

      motorsSetRatio(MOTOR_M1, motorPowerM1);
      motorsSetRatio(MOTOR_M2, motorPowerM2);
      motorsSetRatio(MOTOR_M3, motorPowerM3);
      motorsSetRatio(MOTOR_M4, motorPowerM4);

      // DEBUG
    //  DEBUG_PRINT("Controller debug");

    }
  }
}


void controllerInit(void)
{
  if(isInit)
    return;

  // Call dependency inits
  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();

  // Create task
  xTaskCreate(controllerTask, CONTROLLER_TASK_NAME,
              CONTROLLER_TASK_STACKSIZE, NULL, CONTROLLER_TASK_PRI, NULL);

  isInit = true;
}

bool controllerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();
  return pass;
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
//LOG_ADD(LOG_UINT16, thrust, &ThrustVector[0])
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &acc.x)
LOG_ADD(LOG_FLOAT, y, &acc.y)
LOG_ADD(LOG_FLOAT, z, &acc.z)
LOG_GROUP_STOP(acc)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &gyro.x)
LOG_ADD(LOG_FLOAT, y, &gyro.y)
LOG_ADD(LOG_FLOAT, z, &gyro.z)
LOG_GROUP_STOP(gyro)

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &mag.x)
LOG_ADD(LOG_FLOAT, y, &mag.y)
LOG_ADD(LOG_FLOAT, z, &mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPowerM4)
LOG_ADD(LOG_INT32, m1, &motorPowerM1)
LOG_ADD(LOG_INT32, m2, &motorPowerM2)
LOG_ADD(LOG_INT32, m3, &motorPowerM3)
LOG_GROUP_STOP(motor)
