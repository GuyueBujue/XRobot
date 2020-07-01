/*
	控制射击。

*/

/* Includes ------------------------------------------------------------------*/
#include "task_common.h"

/* Include 标准库 */
/* Include Board相关的头文件 */
#include "bsp_usb.h"

/* Include Device相关的头文件 */
/* Include Component相关的头文件 */
#include "robot_config.h"

/* Include Module相关的头文件 */
#include "shoot.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static CAN_Device_t *cd;

static Shoot_t shoot;
static CMD_t cmd;

/* Private function prototypes -----------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_CtrlShoot(void *argument) {
	const uint32_t delay_tick = osKernelGetTickFreq() / TASK_FREQ_HZ_CTRL_SHOOT;
	const Task_Param_t *task_param = (Task_Param_t*)argument;
	/* Task Setup */
	osDelay(TASK_INIT_DELAY_CTRL_SHOOT);
	
	cd = CAN_GetDevice();

	Shoot_Init(&shoot, &(RobotConfig_Get(ROBOT_CONFIG_MODEL_INFANTRY)->param.shoot));
	shoot.dt_sec = (float32_t)delay_tick / (float32_t)osKernelGetTickFreq();
	
	uint32_t tick = osKernelGetTickCount();
	while(1) {
		/* Task body */
		tick += delay_tick;
		
		uint32_t flag = CAN_DEVICE_SIGNAL_MOTOR_RECV;
		if (osThreadFlagsWait(flag, osFlagsWaitAll, delay_tick) != osFlagsErrorTimeout) {
			osMessageQueueGet(task_param->messageq.cmd, &cmd, NULL, 0);
			
			osKernelLock();
			Shoot_UpdateFeedback(&shoot, cd);
			osKernelUnlock();
			
			Shoot_Control(&shoot, &(cmd.shoot));
			
			// TODO: Check can error.
			CAN_Motor_ControlShoot(
				shoot.fric_cur_out[0],
				shoot.fric_cur_out[1],
				shoot.trig_cur_out
			);
			
			osDelayUntil(tick);
		} else {
			CAN_Motor_ControlShoot(0.f, 0.f, 0.f);
		}
	}
}
