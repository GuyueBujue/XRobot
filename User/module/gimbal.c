/*
  云台模组
*/

/* Includes ----------------------------------------------------------------- */
#include "gimbal.h"

#include "bsp/mm.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
/* Private function  -------------------------------------------------------- */
static int8_t Gimbal_SetMode(Gimbal_t *g, CMD_GimbalMode_t mode) {
  if (g == NULL) return -1;
  if (mode == g->mode) return GIMBAL_OK;
  g->mode = mode;

  /* 切换模式后重置PID和滤波器 */
  for (uint8_t i = 0; i < GIMBAL_PID_NUM; i++) {
    PID_Reset(g->pid + i);
  }
  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
    LowPassFilter2p_Reset(g->filter_out + i, 0.0f);
  }

  AHRS_ResetEulr(&(g->setpoint.eulr));

  return 0;
}

/* Exported functions ------------------------------------------------------- */
int8_t Gimbal_Init(Gimbal_t *g, const Gimbal_Params_t *param,
                   float target_freq) {
  if (g == NULL) return -1;

  g->param = param;

  g->mode = GIMBAL_MODE_RELAX;

  PID_Init(&(g->pid[GIMBAL_PID_YAW_ANGLE_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_YAW_ANGLE_IDX]));
  PID_Init(&(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]), KPID_MODE_CALC_D_FB,
           target_freq, &(g->param->pid[GIMBAL_PID_YAW_OMEGA_IDX]));

  PID_Init(&(g->pid[GIMBAL_PID_PIT_ANGLE_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_PIT_ANGLE_IDX]));
  PID_Init(&(g->pid[GIMBAL_PID_PIT_OMEGA_IDX]), KPID_MODE_CALC_D_FB,
           target_freq, &(g->param->pid[GIMBAL_PID_PIT_OMEGA_IDX]));

  PID_Init(&(g->pid[GIMBAL_PID_REL_YAW_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_REL_YAW_IDX]));
  PID_Init(&(g->pid[GIMBAL_PID_REL_PIT_IDX]), KPID_MODE_NO_D, target_freq,
           &(g->param->pid[GIMBAL_PID_REL_PIT_IDX]));

  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) {
    LowPassFilter2p_Init(g->filter_out + i, target_freq,
                         g->param->low_pass_cutoff_freq.out);
  }
  return 0;
}

int8_t Gimbal_CANtoFeedback(Gimbal_Feedback *gimbal_feedback,
                            const CAN_t *can) {
  if (gimbal_feedback == NULL) return -1;
  if (can == NULL) return -1;

  gimbal_feedback->eulr.encoder.yaw =
      can->gimbal_motor_feedback[CAN_MOTOR_GIMBAL_YAW].rotor_angle;
  gimbal_feedback->eulr.encoder.pit =
      can->gimbal_motor_feedback[CAN_MOTOR_GIMBAL_PIT].rotor_angle;

  return 0;
}

int8_t Gimbal_Control(Gimbal_t *g, Gimbal_Feedback *fb, CMD_GimbalCmd_t *g_cmd,
                      float dt_sec) {
  if (g == NULL) return -1;
  if (g_cmd == NULL) return -1;

  Gimbal_SetMode(g, g_cmd->mode);

  /* 设置初始yaw目标值 */
  if (g->setpoint.eulr.yaw == 0.0f) {
    g->setpoint.eulr.yaw = fb->eulr.imu.yaw;
  }

  /* 处理控制命令，限制setpoint范围 */
  CircleAdd(&(g->setpoint.eulr.yaw), g_cmd->delta_eulr.yaw * dt_sec,
            2.0f * M_PI);
  CircleAdd(&(g->setpoint.eulr.pit), g_cmd->delta_eulr.pit * dt_sec,
            2.0f * M_PI);

  g->setpoint.eulr.pit = AbsClip(g->setpoint.eulr.pit, M_PI / 2.0f);

  AHRS_ResetEulr(&(g_cmd->delta_eulr));

  /* 控制相关逻辑 */
  float yaw_omega_set_point, pit_omega_set_point;
  switch (g->mode) {
    case GIMBAL_MODE_RELAX:
      for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++) g->out[i] = 0.0f;
      break;

    case GIMBAL_MODE_ABSOLUTE:
      yaw_omega_set_point =
          PID_Calc(&(g->pid[GIMBAL_PID_YAW_ANGLE_IDX]), g->setpoint.eulr.yaw,
                   fb->eulr.imu.yaw, 0.0f, dt_sec);
      g->out[GIMBAL_ACTR_YAW_IDX] =
          PID_Calc(&(g->pid[GIMBAL_PID_YAW_OMEGA_IDX]), yaw_omega_set_point,
                   fb->gyro.z, 0.f, dt_sec);

      pit_omega_set_point =
          PID_Calc(&(g->pid[GIMBAL_PID_PIT_ANGLE_IDX]), g->setpoint.eulr.pit,
                   fb->eulr.imu.pit, 0.0f, dt_sec);
      g->out[GIMBAL_ACTR_PIT_IDX] =
          PID_Calc(&(g->pid[GIMBAL_PID_PIT_OMEGA_IDX]), pit_omega_set_point,
                   fb->gyro.x, 0.f, dt_sec);
      break;

    case GIMBAL_MODE_FIX:
      g->setpoint.eulr.yaw = g->param->encoder_center.yaw;
      g->setpoint.eulr.pit = g->param->encoder_center.pit;
      /* 这里不要加break */

    case GIMBAL_MODE_RELATIVE:
      g->out[GIMBAL_ACTR_YAW_IDX] =
          PID_Calc(&(g->pid[GIMBAL_PID_REL_YAW_IDX]), g->setpoint.eulr.yaw,
                   fb->eulr.encoder.yaw, fb->gyro.z, dt_sec);
      g->out[GIMBAL_ACTR_PIT_IDX] =
          PID_Calc(&(g->pid[GIMBAL_PID_REL_PIT_IDX]), g->setpoint.eulr.pit,
                   fb->eulr.encoder.pit, fb->gyro.x, dt_sec);
      break;
  }

  /* 输出滤波 */
  for (uint8_t i = 0; i < GIMBAL_ACTR_NUM; i++)
    g->out[i] = LowPassFilter2p_Apply(g->filter_out + i, g->out[i]);

  return 0;
}
