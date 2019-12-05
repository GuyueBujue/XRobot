#pragma once

#include "board.h"

typedef enum {
	LED1,
	LED2,
	LED3,
	LED4,
	LED5,
	LED6,
	LED7,
	LED8,
	LED_RED,
	LED_GRN,
} LED_Num_t;

typedef enum {
	LED_ON,
	LED_OFF,
	LED_TAGGLE,
} LED_Status_t;

typedef enum {
	JOYSTICK_UP,
	JOYSTICK_DOWN,
	JOYSTICK_LEFT,
	JOYSTICK_RIGHT,
	JOYSTICK_PRESSED,
	JOYSTICK_MID,
} Joystick_StatusTypedef;

typedef enum {
	PWM_A,
	PWM_B,
	PWM_C,
	PWM_D,
	PWM_E,
	PWM_F,
	PWM_G,
	PWM_H,
	PWM_S,
	PWM_T,
	PWM_U,
	PWM_V,
	PWM_W,
	PWM_X,
	PWM_Y,
	PWM_Z,
	PWM_IMU_HEAT,
} PWM_Num_t;

typedef enum {
	POWER_PORT1,
	POWER_PORT2,
	POWER_PORT3,
	POWER_PORT4,
} Power_Port_t;

Board_Status_t LED_Set(LED_Num_t n, LED_Status_t s);

Board_Status_t Joystick_Update(Joystick_StatusTypedef* val);
Board_Status_t Joystick_WaitInput(void);
Board_Status_t Joystick_WaitNoInput(void);

Board_Status_t PWM_Start(PWM_Num_t n);
Board_Status_t PWM_Set(PWM_Num_t n, uint16_t pulse);

Board_Status_t Power_On(Power_Port_t port);
Board_Status_t Power_Off(Power_Port_t port);

Board_Status_t Laser_On(void);
Board_Status_t Laser_Off(void);

Board_Status_t Friction_On(uint16_t pulse);
Board_Status_t Friction_Off(void);

Board_Status_t Buzzer_On(uint16_t pulse);
Board_Status_t Buzzer_Off(void);
