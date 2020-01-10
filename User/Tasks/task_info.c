/* 
	提供机器人运行信息的任务，主要包括OLED显示、LED显示等。

*/
#include "task_info.h"
#include "cmsis_os.h"

#include "oled.h"
#include "io.h"

#define DISPLAY_TASK_FREQ_HZ (10)
#define DISPLAY_TASK_STATUS_LED LED4

void Task_Display(const void* argument) {
	uint32_t last_tick = osKernelSysTick();
	
	LED_Set(DISPLAY_TASK_STATUS_LED, LED_ON);
	
	while (1) {
		OLED_Refresh();
		
		LED_Set(DISPLAY_TASK_STATUS_LED, LED_TAGGLE);
		osDelayUntil(1000 / DISPLAY_TASK_FREQ_HZ);
	}
}
