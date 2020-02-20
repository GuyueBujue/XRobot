/* 
	底盘模组

*/


/* Includes ------------------------------------------------------------------*/
#include "gimbal.h"

/* Include 标准库 */
/* Include Board相关的头文件 */
/* Include Device相关的头文件 */
/* Include Component相关的头文件 */
#include "user_math.h"

/* Include Module相关的头文件 */
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function  ---------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
int Gimbal_Init(Gimbal_t *gimb) {
	if (gimb == NULL)
		return -1;
	
	gimb->mode = GIMBAL_MODE_INIT;

	PID_Init(&(gimb->yaw_inner_pid), PID_MODE_DERIVATIV_SET, gimb->dt_sec);
	PID_SetParameters(&(gimb->yaw_inner_pid), 5.f, 1.f, 0.f, 1.f, 1.f);
	
	PID_Init(&(gimb->yaw_outer_pid), PID_MODE_DERIVATIV_NONE, gimb->dt_sec);
	PID_SetParameters(&(gimb->yaw_outer_pid), 5.f, 1.f, 0.f, 1.f, 1.f);
	
	PID_Init(&(gimb->pit_inner_pid), PID_MODE_DERIVATIV_SET, gimb->dt_sec);
	PID_SetParameters(&(gimb->pit_inner_pid), 5.f, 1.f, 0.f, 1.f, 1.f);
	
	PID_Init(&(gimb->pit_outer_pid), PID_MODE_DERIVATIV_NONE, gimb->dt_sec);
	PID_SetParameters(&(gimb->pit_outer_pid), 5.f, 1.f, 0.f, 1.f, 1.f);
	
	return 0;
}

int Gimbal_SetMode(Gimbal_t *gimb, Gimbal_Mode_t mode) {
	if (gimb == NULL)
		return -1;
	
	
	// check mode switchable.
	switch (mode) {
		case GIMBAL_MODE_RELAX:
			break;
		
		case GIMBAL_MODE_INIT:
			break;
		
		case GIMBAL_MODE_CALI:
			break;
		
		case GIMBAL_MODE_ABSOLUTE:
			break;
		
		case GIMBAL_MODE_RELATIVE:
			break;
		
		case GIMBAL_MODE_FIX:
			break;
		
		default:
			return -1;
	}
	return 0;
}

int Gimbal_UpdateFeedback(Gimbal_t *gimb, CAN_Device_t *can_device) {
	if (gimb == NULL)
		return -1;
	
	if (can_device == NULL)
		return -1;
	
	const float yaw_angle = can_device->gimbal_motor_fb.yaw_fb.rotor_angle;
	gimb->encoder.yaw = yaw_angle / (float)CAN_MOTOR_MAX_ENCODER * 2.f * PI;
	
	const float pit_angle = can_device->gimbal_motor_fb.yaw_fb.rotor_angle;
	gimb->encoder.pit = pit_angle / (float)CAN_MOTOR_MAX_ENCODER * 2.f * PI;
	
	return 0;
}

int Gimbal_ParseCommand(Gimbal_Ctrl_t *chas_ctrl, const DR16_t *dr16) {
	if (chas_ctrl == NULL)
		return -1;
	
	if (dr16 == NULL)
		return -1;
	
	//TODO
	
	return 0;
}

int Gimbal_Control(Gimbal_t *gimb, AHRS_Eulr_t *ctrl_eulr) {
	if (gimb == NULL)
		return -1;
	
	if (ctrl_eulr == NULL)
		return -1;
	
	if (gimb->imu == NULL)
		return -1;
	
	float motor_gyro_set;
	
	switch(gimb->mode) {
		case GIMBAL_MODE_RELAX:
			gimb->pit_cur_out = 0;
			gimb->yaw_cur_out = 0;
			break;
		
		case GIMBAL_MODE_INIT:
		case GIMBAL_MODE_CALI:
			break;
		
		case GIMBAL_MODE_ABSOLUTE:
			motor_gyro_set = PID_Calculate(&gimb->yaw_inner_pid, ctrl_eulr->yaw, gimb->eulr->yaw, gimb->imu->gyro.z, gimb->dt_sec);
			gimb->yaw_cur_out  = PID_Calculate(&gimb->yaw_outer_pid, motor_gyro_set, gimb->imu->gyro.z, 0.f, gimb->dt_sec);
			
			motor_gyro_set = PID_Calculate(&gimb->pit_inner_pid, ctrl_eulr->pit, gimb->eulr->pit, gimb->imu->gyro.x, gimb->dt_sec);
			gimb->pit_cur_out  = PID_Calculate(&gimb->pit_outer_pid, motor_gyro_set, gimb->imu->gyro.x, 0.f, gimb->dt_sec);
			break;
			
		case GIMBAL_MODE_FIX:
			ctrl_eulr->yaw = 0.f;
			ctrl_eulr->pit = 0.f;
			/* NO break. */
		case GIMBAL_MODE_RELATIVE:
			motor_gyro_set = PID_Calculate(&gimb->yaw_inner_pid, ctrl_eulr->yaw, gimb->encoder.yaw, gimb->imu->gyro.z, gimb->dt_sec);
			gimb->yaw_cur_out  = PID_Calculate(&gimb->yaw_outer_pid, motor_gyro_set, gimb->imu->gyro.z, 0.f, gimb->dt_sec);
			
			motor_gyro_set = PID_Calculate(&gimb->pit_inner_pid, ctrl_eulr->pit, gimb->encoder.pit, gimb->imu->gyro.x, gimb->dt_sec);
			gimb->pit_cur_out  = PID_Calculate(&gimb->pit_outer_pid, motor_gyro_set, gimb->imu->gyro.x, 0.f, gimb->dt_sec);
			break;
			
		default:
			return -1;
	}
	
	// TODO: Filter output.
	
	return 0;
}
