/*
	姿态解算任务。
	
	控制IMU加热到指定温度防止温漂，收集IMU数据给AHRS算法。
	解算后的数据发送给需要用的任务。
*/

/* Includes ------------------------------------------------------------------*/
#include "task_common.h"

/* Include 标准库 */
#include <string.h>

/* Include Board相关的头文件 */
#include "bsp_pwm.h"

/* Include Device相关的头文件 */
#include "imu.h"

/* Include Component相关的头文件 */
#include "ahrs.h"
#include "pid.h"

/* Include Module相关的头文件 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint32_t delay_ms = 1000u / TASK_POSESTI_FREQ_HZ;
static int result = 0;
static osStatus os_status = osOK;

osPoolDef(ahrs_pool, 2, AHRS_Eulr_t);
osMessageQDef(ahrs_message, 2, AHRS_Eulr_t);

IMU_t onboard_imu;
AHRS_t gimbal_ahrs;
PID_t imu_temp_ctrl_pid;

/* Private function  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_PosEsti(void const *argument) {
	Task_Param_t *task_param = (Task_Param_t*)argument;
	
	task_param->pool.ahrs = osPoolCreate(osPool(ahrs_pool));
	task_param->message.ahrs = osMessageCreate(osMessageQ(ahrs_message), NULL);
	
	onboard_imu.received_alert = task_param->thread.pos_esti;
	result += IMU_Init(&onboard_imu);
	
	result += IMU_StartReceiving(&onboard_imu);
	osSignalWait(IMU_SIGNAL_RAW_ACCL_REDY | IMU_SIGNAL_RAW_GYRO_REDY, osWaitForever);
	osSignalWait(IMU_SIGNAL_RAW_MAGN_REDY, 1u);
	result += IMU_Parse(&onboard_imu);
	
	result += AHRS_Init(&gimbal_ahrs, &onboard_imu, TASK_POSESTI_FREQ_HZ);
	
	result += PID_Init(&imu_temp_ctrl_pid, PID_MODE_DERIVATIV_NONE, 1.f/TASK_POSESTI_FREQ_HZ);
	result += PID_SetParameters(&imu_temp_ctrl_pid, .005f, .001f, 0.f, 1.f, 1.f);
	
	result += BSP_PWM_Set(BSP_PWM_IMU_HEAT, 0.f);
	result += BSP_PWM_Start(BSP_PWM_IMU_HEAT);
	
	uint32_t previous_wake_time = osKernelSysTick();
	while(1) {
		/* Task */
		result += IMU_StartReceiving(&onboard_imu);
		osSignalWait(IMU_SIGNAL_RAW_ACCL_REDY | IMU_SIGNAL_RAW_GYRO_REDY, osWaitForever);
		osSignalWait(IMU_SIGNAL_RAW_MAGN_REDY, 1u);
		result += IMU_Parse(&onboard_imu);
		
		uint32_t now = osKernelSysTick();
		result += AHRS_Update(&gimbal_ahrs, &onboard_imu);
		
		AHRS_Eulr_t *eulr_to_send;
		eulr_to_send = osPoolAlloc(task_param->pool.ahrs);
		memcpy(eulr_to_send, &(gimbal_ahrs.eulr), sizeof(AHRS_Eulr_t));
		
		/* Drop data if queue is full. */
		os_status = osMessagePut(task_param->message.ahrs, (uint32_t)eulr_to_send, 1);
		
		/* Free memory if data dropped. */
		if (os_status == osErrorOS)
			osPoolFree(task_param->pool.ahrs, eulr_to_send);

		result += BSP_PWM_Set(BSP_PWM_IMU_HEAT, PID_Calculate(&imu_temp_ctrl_pid, 50.f, onboard_imu.data.temp, 0.f, 0.f));
		
		os_status += osDelayUntil(&previous_wake_time, delay_ms);
	}
}
