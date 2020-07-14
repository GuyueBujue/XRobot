#pragma once


/* Includes ------------------------------------------------------------------*/
/* Include cmsis_os2.h头文件 */
#include "cmsis_os2.h"

#include "cmd.h"
#include "user_math.h"

/* Exported constants --------------------------------------------------------*/
#define DR16_OK			(0)
#define DR16_ERR_NULL	(-1)

#define DR16_SIGNAL_RAW_REDY	(1u<<7)

#define DR16_CH_VALUE_MIN		(364u)
#define DR16_CH_VALUE_MID		(1024u)
#define DR16_CH_VALUE_MAX		(1684u)

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
typedef __packed struct {
	uint16_t ch_r_x:11;
	uint16_t ch_r_y:11;
	uint16_t ch_l_x:11;
	uint16_t ch_l_y:11;
	uint8_t sw_l:2;
	uint8_t sw_r:2;
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t press_l;
	uint8_t press_r;
	uint16_t key;
	uint16_t res;
} DR16_Data_t;


typedef struct {
	osThreadId_t thread_alert;

	DR16_Data_t data;
} DR16_t;

/* Exported functions prototypes ---------------------------------------------*/
int8_t DR16_Init(DR16_t *dr16, osThreadId_t thread_alert);
DR16_t *DR16_GetDevice(void);
int8_t DR16_Restart(void);

int8_t DR16_StartReceiving(DR16_t *dr16);
int8_t DR16_Parse(const DR16_t *dr16, CMD_RC_t *rc);
