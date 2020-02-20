/*
	控制云台。

*/

/* Includes ------------------------------------------------------------------*/
#include "task_common.h"

/* Include 标准库 */
/* Include Board相关的头文件 */
/* Include Device相关的头文件 */
/* Include Component相关的头文件 */
/* Include Module相关的头文件 */
#include "gimbal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint32_t delay_ms = 1000u / TASK_CTRL_GIMBAL_FREQ_HZ;

static CAN_Device_t *cd;
static DR16_t *dr16;

static Gimbal_t gimbal;
static Gimbal_Ctrl_t gimbal_ctrl;

/* Runtime status. */
int stat_c_g = 0;
osStatus os_stat_c_g = osOK;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_CtrlGimbal(void const *argument) {
	Task_Param_t *task_param = (Task_Param_t*)argument;
	
	/* Task Setup */
	osDelay(TASK_CTRL_GIMBAL_INIT_DELAY);
	
	cd = CAN_GetDevice();
	dr16 = DR16_GetDevice();
	
	gimbal.imu = IMU_GetDevice();
	Gimbal_Init(&gimbal);
	
	uint32_t previous_wake_time = osKernelSysTick();
	while(1) {
		/* Task body */
		
		/* Try to get new rc command. */
		osSignalWait(DR16_SIGNAL_DATA_REDY, 0);
		Gimbal_ParseCommand(&gimbal_ctrl, dr16);
		
		/* Wait for motor feedback. */
		osSignalWait(CAN_DEVICE_SIGNAL_MOTOR_RECV, osWaitForever);
		
		taskENTER_CRITICAL();
		Gimbal_UpdateFeedback(&gimbal, cd);
		taskEXIT_CRITICAL();
		
		/* Wait for new eulr data. */
		osEvent evt = osMessageGet(task_param->message.gimb_eulr, osWaitForever);
		if (evt.status == osEventMessage) {
			if (gimbal.eulr) {
				vPortFree(gimbal.eulr);
			}
			gimbal.eulr = evt.value.p;
		}
		
		Gimbal_SetMode(&gimbal, gimbal_ctrl.mode);
		Gimbal_Control(&gimbal, &gimbal_ctrl.ctrl_eulr);
		
		// Check can error
		CAN_Motor_ControlGimbal(gimbal.yaw_cur_out, gimbal.pit_cur_out);
		
		osDelayUntil(&previous_wake_time, delay_ms);
	}
}
