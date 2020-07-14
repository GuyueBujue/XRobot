/* 
	
*/

#pragma once

#include <stdint.h>

#include "cmd.h"

#include "chassis.h"
#include "gimbal.h"
#include "shoot.h"

#define CONFIG_ADDRESS 

typedef enum {
	CONFIG_ROBOT_MODEL_INFANTRY,
	CONFIG_ROBOT_MODEL_HERO,
	CONFIG_ROBOT_MODEL_ENGINEER,
	CONFIG_ROBOT_MODEL_DRONE,
	CONFIG_ROBOT_MODEL_SENTRY,
} Config_Model_t;

typedef enum {
	CONFIG_USER_DEFAULT,
} Config_UserName_t;

typedef const struct {
	const struct {
		const Chassis_Params_t chassis;
		const Gimbal_Params_t gimbal;
		const Shoot_Params_t shoot;
		const CMD_Params_t cmd;
	} param;
} Config_Robot_t;

typedef const struct {
	const struct {
		const CMD_Params_t cmd;
	} param;
} Config_User_t;

const Config_Robot_t *Config_GetRobot(Config_Model_t model);
const Config_User_t *Config_GetUser(Config_UserName_t user);
