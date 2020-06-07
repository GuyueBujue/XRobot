#pragma once


/* Includes ------------------------------------------------------------------*/
/* Include cmsis_os.h头文件 */
#include "cmsis_os.h"

/* Exported constants --------------------------------------------------------*/
#define CAN_OK			(0)
#define CAN_ERR			(-1)
#define CAN_ERR_NULL	(-2)
#define CAN_ERR_INITED	(-3)

#define CAN_DEVICE_SIGNAL_MOTOR_RECV			(1u<<4)
#define CAN_DEVICE_SIGNAL_UWB_RECV				(1u<<5)
#define CAN_DEVICE_SIGNAL_SUPERCAP_RECV			(1u<<6)

/* Motor */
#define CAN_GM6020_FEEDBACK_ID_BASE				0x205
#define CAN_GM6020_RECEIVE_ID_BASE				0x1ff
#define CAN_GM6020_RECEIVE_ID_EXTAND			0x2ff

#define CAN_M3508_M2006_FEEDBACK_ID_BASE			0x201
#define CAN_M3508_M2006_RECEIVE_ID_BASE				0x200
#define CAN_M3508_M2006_RECEIVE_ID_EXTAND			0x1ff
#define CAN_M3508_M2006_ID_SETTING_ID				0x700

#define CAN_GM6020_MAX_ABS_VOLTAGE			30000
#define CAN_M3508_MAX_ABS_VOLTAGE			16384
#define CAN_M2006_MAX_ABS_VOLTAGE			10000

#define CAN_MOTOR_MAX_NUM				9
#define CAN_CHASSIS_MOTOR_NUM			4
#define CAN_GIMBAL_MOTOR_NUM			5

#define CAN_MOTOR_MAX_ENCODER			8191

#define CAN_MOTOR_CAN_RX_FIFO		CAN_RX_FIFO0

/* UWB */
#define CAN_UWB_FEEDBACK_ID_BASE		(0x259)
#define CAN_UWB_RX_FIFO_SIZE				(22)
#define CAN_UWB_TX_FIFO_SIZE				22)

#define CAN_UWB_MAX_YAW					36000
#define CAN_UWB_CAN_RX_FIFO			CAN_RX_FIFO1

/* Super capacitor */
#define CAN_SUPERCAP_FEEDBACK_ID_BASE				0x000
#define CAN_SUPERCAP_RECEIVE_ID_BASE				0x000

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Motor */
typedef enum
{
	CAN_M3508_M1_ID = 0x201, /* 1 */
	CAN_M3508_M2_ID = 0x202, /* 2 */
	CAN_M3508_M3_ID = 0x203, /* 3 */
	CAN_M3508_M4_ID = 0x204, /* 4 */
	
	CAN_M3508_FRIC1_ID = 0x205, /* 5 */
	CAN_M3508_FRIC2_ID = 0x206, /* 6 */
	CAN_M2006_TRIG_ID = 0x207, /* 7 */

	CAN_GM6020_YAW_ID = 0x209, /* 5 */
	CAN_GM6020_PIT_ID = 0x20A, /* 6 */
} CAN_MotorId_t;

typedef struct {
	uint16_t rotor_angle;
	int16_t  rotor_speed;
	int16_t  torque_current;
	uint8_t  temp;
} CAN_MotorFeedback_t;

/* UWB */
typedef union {
	uint8_t raw[CAN_UWB_RX_FIFO_SIZE];
	
	__packed struct
	{
		int16_t coor_x;
		int16_t coor_y;
		uint16_t yaw;
		int16_t distance[6];
		uint16_t err_mask : 14;
		uint16_t sig_level : 2;
		uint16_t reserved;
	} data;
} CAN_UWBFeedback_t;

/* Super capacitor */
typedef struct {
	uint16_t cap_voltage;
	int16_t  battery_voltage;
	int16_t  power_limit;
	uint8_t  temp;
} CAN_SuperCapFeedback_t;

typedef struct {
	int16_t power_limit;
} CAN_SuperCapControl_t;

typedef struct {
	osThreadId_t motor_alert[3];
	osThreadId_t uwb_alert;
	osThreadId_t supercap_alert;
	
	CAN_MotorFeedback_t chassis_motor_fb[4];
	
	struct {
		CAN_MotorFeedback_t fric_fb[2];
		CAN_MotorFeedback_t trig_fb;
		CAN_MotorFeedback_t yaw_fb;
		CAN_MotorFeedback_t pit_fb;
	} gimbal_motor_fb;
	
	CAN_UWBFeedback_t uwb_feedback;
	CAN_SuperCapFeedback_t supercap_feedback;
} CAN_Device_t;


/* Exported functions prototypes ---------------------------------------------*/
int CAN_DeviceInit(CAN_Device_t *can_device);
CAN_Device_t *CAN_GetDevice(void);

int CAN_Motor_ControlChassis(float m1, float m2, float m3, float m4);
int CAN_Motor_ControlGimbal(float yaw, float pitch);
int CAN_Motor_ControlShoot(float fric1, float fric2, float trig);

int CAN_Motor_QuickIdSetMode(void);
int CAN_SuperCapControl(CAN_SuperCapControl_t *sc_ctrl);

