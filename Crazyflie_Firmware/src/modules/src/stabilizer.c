/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "config.h"
#include "system.h"
#include "pm.h"
#include "stabilizer.h"
#include "commander.h"
#include "attitude_controller.h"
#include "sensfusion6.h"
#include "imu.h"
#include "motors.h"
#include "log.h"
#include "pid.h"
#include "param.h"
#include "sitaw.h"
#ifdef PLATFORM_CF1
  #include "ms5611.h"
#else
  #include "lps25h.h"
#endif
#include "num.h"
#include "position_estimator.h"
#include "position_controller.h"
#include "altitude_hold.h"


/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define ATTITUDE_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define ALTHOLD_UPDATE_RATE_DIVIDER  5
#define ALTHOLD_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ALTHOLD_UPDATE_RATE_DIVIDER))   // 100hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float eulerRollActual;   // Measured roll angle in deg
static float eulerPitchActual;  // Measured pitch angle in deg
static float eulerYawActual;    // Measured yaw angle in deg

uint16_t actuatorThrust;  // Actuator output for thrust base

uint32_t motorPowerM1;  // Motor 1 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM2;  // Motor 2 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM3;  // Motor 3 power output (16bit value used: 0 - 65535)
uint32_t motorPowerM4;  // Motor 4 power output (16bit value used: 0 - 65535)

static bool isInit;


static uint16_t limitThrust(int32_t value);

static void stabilizerTask(void* param)
{
  uint32_t attitudeCounter = 0;
  uint32_t lastWakeTime;

  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();

  lastWakeTime = xTaskGetTickCount ();

  while(1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(IMU_UPDATE_FREQ)); // 500Hz

    // Magnetometer not yet used more then for logging.
    imu9Read(&gyro, &acc, &mag);

    if (imu6IsCalibrated())
    {
      // 250HZ
      if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
      {
        sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z, ATTITUDE_UPDATE_DT);
        sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);
        //Calulate p, q, r, u, v, w, x, y, z using integrals and derivatives?
        
        //Kalman implementation
        float y_k[12]={eulerRollActual,eulerPitchActual,eulerYawActual,p,q,r,u,v,w,x,y,z}; 			
        float y_hat_k[12]={0,0,0,0,0,0,0,0,0,0,0,0};		//Plant output initialize to 0 (how many elements?)
        float x_hat_k[12]; 		//Last estimated states
        float A[12][12]=		//A matrix NOT discretized!!!
        {
		{1,0,0,0.002,0,0,0,0,0,0,0,0},
		{0,1,0,0,0.002,0,0,0,0,0,0,0},
		{0,0,1,0,0,0.002,0,0,0,0,0,0},
		{0,0,0,1,0,0,0,0,0,0,0,0},
		{0,0,0,0,1,0,0,0,0,0,0,0},
		{0,0,0,0,0,1,0,0,0,0,0,0},
		{0,-0.01962,0,0,-0.00001962,0,1,0,0,0,0,0},
		{0.01962,0,0,0.00001962,0,0,0,1,0,0,0,0},
		{0,0,0,0,0,0,0,0,1,0,0,0},
		{0,-0.00001962,0,0,-0.00000001308,0,0.002,0,0,1,0,0},
		{0.00001962,0,0,0.00000001308,0,0,0,0.002,0,0,1,0},
		{0,0,0,0,0,0,0,0,0.002,0,0,1},
        };
        float B[12][4] = 	//B matrix NOT discretized!!!
        {{0,0.1434,0,0},
         {0,0,0.1393,0},
		 {0,0,0,0.09204},
		 {0,143.4,0,0},
		 {0,0,69638,0},
		 {0,0,0,46019},
		 {0,0,0,0},
		 {0,0,0,0},
		 {37.047,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
        };
        float C[12][12] = 	//C matrix NOT discretized!!! (depends on number of meassured states)
        {{1,0,0,0,0,0,0,0,0,0,0,0},
         {0,1,0,0,0,0,0,0,0,0,0,0},
		 {0,0,1,0,0,0,0,0,0,0,0,0},
		 {0,0,0,1,0,0,0,0,0,0,0,0},
		 {0,0,0,0,1,0,0,0,0,0,0,0},
		 {0,0,0,0,0,1,0,0,0,0,0,0},
		 {0,0,0,0,0,0,1,0,0,0,0,0},
		 {0,0,0,0,0,0,0,1,0,0,0,0},
		 {0,0,0,0,0,0,0,0,1,0,0,0},
		 {0,0,0,0,0,0,0,0,0,1,0,0},
		 {0,0,0,0,0,0,0,0,0,0,1,0},
		 {0,0,0,0,0,0,0,0,0,0,0,1},
        };
        float D[12][4] = ... 	//D matrix NOT discretized!!! (depends on number of meassured states)
		{{0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		 {0,0,0,0},
		};
        float u_k[4];			//Last input
		float x_hat_new[12]; 	// Updated state, initialize to zero
		float K[4][12];
		float u_k[4]; //Input initialize with zeros?
		float r_k[12]; //Reference
		//Meassurment update
		for(int i=0;i<12;i++)
        {
        	for(int j=0;j<12;j++)
        	{
        		y_hat_k[i]=y_hat_k[i]+C[i][j]*x_hat_k[j];   
          	}
        	for(int j=0;j<4;j++)
			{
				y_hat_k[i]=y_hat_k[i]+D[i][j]*u_k[j];
			}
        }
        //State prediction
        for(int i=0;i<12;i++)
		{
			for(int j=0;j<12;j++)
			{
				x_hat_new[i]=x_hat_new[i]+(A[i][j]-B[i][1]*K[1][j]-B[i][2]*K[2][j]-B[i][3]*K[3][j]-B[i][4]*K[4][j])*x_hat_k[j];   //State update
			
			}
			for(int j=0;j<12;j++)
			{
				x_hat_new[i]=x_hat_new[i]+L[i][j]*(y_k[j]-y_hat_k[j]);
			}
		}
        //LQR implementation
        for(int i=0;i<4;i++)
        {
        	for(int j=0;j<12;j++)
        	{
        		u_k[i]=u_k[i]+K[i][j]*(r_k[j]-x_hat_new[j]);

        	}
        }

        // Set motors depending on the euler angles
//        motorPowerM1 = limitThrust(fabs(32000*eulerYawActual/180.0));
//        motorPowerM2 = limitThrust(fabs(32000*eulerPitchActual/180.0));
//        motorPowerM3 = limitThrust(fabs(32000*eulerRollActual/180.0));
//        motorPowerM4 = limitThrust(fabs(32000*eulerYawActual/180.0));

        motorsSetRatio(MOTOR_M1, motorPowerM1);
        motorsSetRatio(MOTOR_M2, motorPowerM2);
        motorsSetRatio(MOTOR_M3, motorPowerM3);
        motorsSetRatio(MOTOR_M4, motorPowerM4);

        attitudeCounter = 0;
      }
    }
  }
}

void stabilizerInit(void)
{
  if(isInit)
    return;

  motorsInit(motorMapDefaultBrushed);
  imu6Init();
  sensfusion6Init();
  attitudeControllerInit();

  xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
              STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= motorsTest();
  pass &= imu6Test();
  pass &= sensfusion6Test();
  pass &= attitudeControllerTest();

  return pass;
}

static uint16_t limitThrust(int32_t value)
{
  return limitUint16(value);
}

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &eulerRollActual)
LOG_ADD(LOG_FLOAT, pitch, &eulerPitchActual)
LOG_ADD(LOG_FLOAT, yaw, &eulerYawActual)
LOG_ADD(LOG_UINT16, thrust, &actuatorThrust)
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
