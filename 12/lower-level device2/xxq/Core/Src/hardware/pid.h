#ifndef __PID_H
#define	__PID_H
#include "stm32f4xx_hal.h"
#include "usart.h"
#include <stdio.h>
#include <stdlib.h>

typedef struct
{
    float target_val;           // Target value
    float actual_val;        		// Actual value
    float err;             			// Error value
    float err_last;          		// Previous error value
	float err_next;
    float Kp,Ki,Kd;          		// Proportional, integral, derivative coefficients

	float integral;          		// Integral value (position PID)
	float I_Control;
	float absmax;
	float absmin;
}_pid;

extern void PID_param_init(void);
extern void PID_target_val_init(void);
extern void set_pid_target(int temp_val);
extern void set_p_i_d(float p, float i, float d);
extern float PID_realize(_pid *pid, float actual_val);
extern void time_period_fun(void);
float PID_Angle(_pid *pid, float target_angle, float actual_angle);
float PID_location(_pid *pid, float actual_val);
float PID_Position(_pid *pid, float target_position, float actual_position);

#endif
