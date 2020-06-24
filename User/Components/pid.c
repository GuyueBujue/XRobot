/* 
	Modified from https://github.com/PX4/Firmware/blob/master/src/lib/pid/pid.cpp

*/

#include "pid.h"

#include "user_math.h"

#define SIGMA 0.000001f

int PID_Init(PID_t *pid, PID_Mode_t mode, float dt_min, const PID_Params_t *param) {
	if (pid == NULL)
		return -1;
	
	if (isfinite(dt_min))
		pid->dt_min = dt_min;
	else
		return -1;
	
	if (isfinite(param->kp))
		pid->kp = param->kp;
	else
		return -1;

	if (isfinite(param->ki))
		pid->ki = param->ki;
	else
		return -1;

	if (isfinite(param->kd))
		pid->kd = param->kd;
	else
		return -1;

	if (isfinite(param->integral_limit))
		pid->integral_limit = param->integral_limit;
	else
		return -1;

	if (isfinite(param->output_limit))
		pid->output_limit = param->output_limit;
	else
		return -1;

	pid->mode = mode;
	pid->output_limit = 0.0f;
	pid->error_previous = 0.0f;
	pid->last_output = 0.0f;
	return 0;
}

float PID_Calc(PID_t *pid, float sp, float val, float val_dot, float dt) {
	if (!isfinite(sp) || !isfinite(val) || !isfinite(val_dot) || !isfinite(dt)) {
		return pid->last_output;
	}
	
	float i, d;

	/* current error value */
	float error = sp - val;

	/* current error derivative */
	switch (pid->mode) {
		case PID_MODE_DERIVATIV_CALC:
			d = (error - pid->error_previous) / fmaxf(dt, pid->dt_min);
			pid->error_previous = error;
			break;
		
		case PID_MODE_DERIVATIV_CALC_NO_SP:
			d = (-val - pid->error_previous) / fmaxf(dt, pid->dt_min);
			pid->error_previous = -val;
			break;
		
		case PID_MODE_DERIVATIV_SET:
			d = val_dot;
			break;
		
		case PID_MODE_DERIVATIV_NONE:
			d = 0.0f;
			break;
	}
	
	if (!isfinite(d))
		d = 0.0f;
	
	/* calculate PD output */
	float output = (error * pid->kp) + (d * pid->kd);

	if (pid->ki > SIGMA) {
		// Calculate the error integral and check for saturation
		i = pid->integral + (error * dt);

		/* check for saturation */
		if (isfinite(i)) {
			if ((pid->output_limit < SIGMA || (fabsf(output + (i * pid->ki)) <= pid->output_limit)) &&
				fabsf(i) <= pid->integral_limit) {
				/* not saturated, use new integral value */
				pid->integral = i;
			}
		}

		/* add I component to output */
		output += pid->integral * pid->ki;
	}

	/* limit output */
	if (isfinite(output)) {
		if (pid->output_limit > SIGMA) {
			output = AbsClip(output, pid->output_limit);
		}
		pid->last_output = output;
	}

	return pid->last_output;
}

int PID_ResetIntegral(PID_t *pid) {
	if (pid == NULL)
		return -1;
	
	pid->integral = 0.0f;
	
	return 0;
}
