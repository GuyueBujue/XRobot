/*
	控制云台。

*/

/* Includes ------------------------------------------------------------------*/
#include "task_common.h"

/* Include 标准库 */
/* Include Board相关的头文件 */
/* Include Device相关的头文件 */
#include "can_device.h"

/* Include Component相关的头文件 */
#include "mixer.h"
#include "pid.h"
#include "ahrs.h"

/* Include Module相关的头文件 */
#include "gimbal.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static const uint32_t delay_ms = 1000u / TASK_CTRL_GIMBAL_FREQ_HZ;
static int result = 0;
static osStatus os_status = osOK;

static CAN_Device_t *cd;
static AHRS_Eulr_t  *gimbal_eulr;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_CtrlGimbal(void const *argument) {
	Task_Param_t *task_param = (Task_Param_t*)argument;
	
	/* Task Setup */
	osSignalWait(TASK_SIGNAL_CTRL_CHASSIS_READY, TASK_CTRL_CHASSIS_INIT_DELAY);
	osSignalWait(TASK_SIGNAL_POSESTI_READY, TASK_CTRL_CHASSIS_INIT_DELAY);
	
	cd = CAN_GetDevice();
	
	uint32_t previous_wake_time = osKernelSysTick();
	while(1) {
		/* Task */
		osSignalWait(CAN_DEVICE_SIGNAL_GIMBAL_RECV, osWaitForever);
		
		
		osEvent evt = osMessageGet(task_param->message.ahrs, osWaitForever);
		if (evt.status == osEventMessage)
			gimbal_eulr = evt.value.p; 
		
		
		
		osPoolFree(task_param->pool.ahrs, gimbal_eulr);

		osDelayUntil(&previous_wake_time, delay_ms);
	}
}
