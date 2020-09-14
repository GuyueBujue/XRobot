/*
        接收解释指令。

*/

/* Includes ------------------------------------------------------------------*/
#include "device\dr16.h"
#include "task\user_task.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static DR16_t dr16;
static CMD_RC_t rc;
static CMD_t cmd;

/* Private function ----------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
void Task_Command(void *argument) {
  Task_Param_t *task_param = (Task_Param_t *)argument;

  task_param->msgq.cmd = osMessageQueueNew(9u, sizeof(CMD_t), NULL);

  /* Task Setup */
  osDelay(TASK_INIT_DELAY_COMMAND);

  DR16_Init(&dr16, osThreadGetId());
  CMD_Init(&cmd, &(task_param->config_pilot->param.cmd));

  while (1) {
#ifdef DEBUG
    task_param->stack_water_mark.command = osThreadGetStackSpace(NULL);
#endif
    /* Task body */
    DR16_StartReceiving(&dr16);
    osThreadFlagsWait(SIGNAL_DR16_RAW_REDY, osFlagsWaitAll, osWaitForever);

    if (DR16_ParseRC(&dr16, &rc)) {
      DR16_Restart();

    } else {
      CMD_Parse(&rc, &cmd);
      for (uint8_t i = 0; i < 3; i++)
        osMessageQueuePut(task_param->msgq.cmd, &cmd, 0, 0);
    }
  }
}
