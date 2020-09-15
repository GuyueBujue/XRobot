/*
        姿态解算任务。

        控制IMU加热到指定温度防止温漂，收集IMU数据给AHRS算法。
        解算后的数据发送给需要用的任务。
*/

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "bsp\mm.h"
#include "bsp\pwm.h"
#include "bsp\usb.h"
#include "component\ahrs.h"
#include "component\pid.h"
#include "device\bmi088.h"
#include "device\ist8310.h"
#include "task\user_task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static BMI088_t bmi088;
IST8310_t ist8310;  // TODO: Add static when release

static AHRS_t gimbal_ahrs;
AHRS_Eulr_t eulr_to_send;  // TODO: Add static when release

static PID_t imu_temp_ctrl_pid;
static const PID_Params_t imu_temp_ctrl_pid_param = {
    .kp = 0.5,
    .ki = 0.5,
    .kd = 0.5,
    .i_limit = 0.5,
    .out_limit = 0.5,
};

/* Private function  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_AttiEsti(void *argument) {
  Task_Param_t *task_param = (Task_Param_t *)argument;

  task_param->msgq.gimbal_accl =
      osMessageQueueNew(6u, sizeof(AHRS_Accl_t), NULL);
  
  task_param->msgq.gimbal_eulr =
      osMessageQueueNew(6u, sizeof(AHRS_Eulr_t), NULL);
  
  task_param->msgq.gimbal_gyro =
      osMessageQueueNew(6u, sizeof(AHRS_Gyro_t), NULL);

  BMI088_Init(&bmi088);
  IST8310_Init(&ist8310);

#if 0
  ST8310_WaitNew(osWaitForever);
  ST8310_StartDmaRecv();
  ST8310_WaitDmaCplt();
  IST8310_Parse(&ist8310);
#endif
  
  AHRS_Init(&gimbal_ahrs, &ist8310.magn, BMI088_GetUpdateFreq(&bmi088));

  PID_Init(&imu_temp_ctrl_pid, PID_MODE_NO_D,
           1.f / BMI088_GetUpdateFreq(&bmi088), &imu_temp_ctrl_pid_param);

  BSP_PWM_Set(BSP_PWM_IMU_HEAT, 0.f);
  BSP_PWM_Start(BSP_PWM_IMU_HEAT);

  while (1) {
#ifdef DEBUG
    task_param->stack_water_mark.atti_esti = osThreadGetStackSpace(NULL);
#endif
    /* Task body */
    BMI088_WaitNew();

    BMI088_AcclStartDmaRecv();
    BMI088_AcclWaitDmaCplt();

    BMI088_GyroStartDmaRecv();
    BMI088_GyroWaitDmaCplt();
    
    
#if 0
    if (ST8310_WaitNew(0)) {
      ST8310_StartDmaRecv();
      ST8310_WaitDmaCplt();
    }
#endif

    osKernelLock();
    BMI088_ParseAccl(&bmi088);
    BMI088_ParseGyro(&bmi088);
    IST8310_Parse(&ist8310);
    osKernelUnlock();

    AHRS_Update(&gimbal_ahrs, &bmi088.accl, &bmi088.gyro, &ist8310.magn);
    AHRS_GetEulr(&eulr_to_send, &gimbal_ahrs);
    
    osMessageQueuePut(task_param->msgq.gimbal_accl, &bmi088.accl, 0, 0);
    osMessageQueuePut(task_param->msgq.gimbal_eulr, &eulr_to_send, 0, 0);
    osMessageQueuePut(task_param->msgq.gimbal_gyro, &bmi088.gyro, 0, 0);
                                             
    BSP_PWM_Set(BSP_PWM_IMU_HEAT,
                PID_Calc(&imu_temp_ctrl_pid, 50.f, bmi088.temp, 0.f, 0.f));
  }
}
