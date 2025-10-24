#include "pid.h"
#include <math.h>

// Define global variables

_pid pid;
_pid pid1;
_pid pid2;
_pid pid3;
_pid pid4;
_pid pid5;

uint8_t Location_Flag;

/**
  * @brief  PID parameter initialization
	*	@note 	None
  * @retval None
  */
void PID_param_init(void)
{
	/* Gyroscope PID */

    pid.actual_val=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
	  pid.err_next=0.0;
		pid.I_Control = 30;
		pid.Kp = 1.5;
		pid.Ki = 0;
		pid.Kd = 0.0;
	  pid.absmax = 60;
  	pid.absmin = 1;
		/* Initialize parameters */

    pid1.actual_val=0.0;
    pid1.err=0.0;
    pid1.err_last=0.0;
	  pid1.err_next=0.0;

		pid1.Kp = 13;
		pid1.Ki = 4.2;
		pid1.Kd = 7;

		/* Initialize parameters */

    pid2.actual_val=0.0;
    pid2.err=0.0;
    pid2.err_last=0.0;
	  pid2.err_next=0.0;

		pid2.Kp = 13;
		pid2.Ki = 4.2;
		pid2.Kd = 7;

		/* Initialize parameters */
    pid3.target_val=0;
    pid3.actual_val=0.0;
    pid3.err=0.0;
    pid3.err_last=0.0;
	  pid3.err_next=0.0;

		pid3.Kp = 13;
		pid3.Ki = 4.2;
		pid3.Kd = 7;

			/* Initialize parameters */

    pid4.actual_val=0.0;
    pid4.err=0.0;
    pid4.err_last=0.0;
	  pid4.err_next=0.0;

		pid4.Kp = 13;
		pid4.Ki = 4.2;
		pid4.Kd = 7;

		/* Initialize parameters */

    pid5.actual_val=0.0;
    pid5.err=0.0;
    pid5.err_last=0.0;
	  pid5.err_next=0.0;

		pid5.Kp = 0.005;
		pid5.Ki = 0;
		pid5.Kd = 0;

#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid5.Kp, pid5.Ki, pid5.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值

#endif
}

void PID_target_val_init(void)
{
	pid.target_val=0;
	pid1.target_val=0;
	pid2.target_val=0;
	pid4.target_val=0;
	pid5.target_val=0;
}


/**
  * @brief  Set target value
  * @param  val		Target value
	*	@note 	None
  * @retval None---------------
  */
void set_pid_target(int temp_val)
{

  //pid.target_val = (int)temp_val/10;    // 设置当前的目标值
//		pid5.target_val = temp_val;
//	Kinematics_Inverse.M1_RPM = (int)temp_val/1000;
//	pid1.target_val = Kinematics_Inverse.M1_RPM;
//	pid1.target_val = (int)temp_val/1000;
//	pid2.target_val = (int)temp_val/1000;
//	pid3.target_val = (int)temp_val/1000;
//	pid4.target_val = (int)temp_val/1000;
}

/**
  * @brief  获取目标值
  * @param  无
	*	@note 	无
  * @retval 目标值
  */
//float get_pid_target(void)
//{
//
//	  return Kinematics_Inverse.Linear_X;    // 设置当前的目标值
////	return pid2.target_val;    // 设置当前的目标值
////	return pid3.target_val;    // 设置当前的目标值
////  return pid1.target_val;    // 设置当前的目标值
////	return pid2.target_val;    // 设置当前的目标值
////	return pid3.target_val;    // 设置当前的目标值
////	return pid4.target_val;    // 设置当前的目标值
//}

/**
  * @brief  Set proportional, integral, derivative coefficients
  * @param  p: Proportional coefficient P
  * @param  i: Integral coefficient i
  * @param  d: Derivative coefficient d
	*	@note 	None
  * @retval None
  */
void set_p_i_d(float p, float i, float d)
{
//		pid.Kp = p;    // 设置比例系数 P
//		pid.Ki = i;    // 设置积分系数 I
//		pid.Kd = d;    // 设置微分系数 D
//	pid1.Kp = p;    // 设置比例系数 P
//		pid1.Ki = i;    // 设置积分系数 I
//		pid1.Kd = d;    // 设置微分系数 D
//	pid2.Kp = p;    // 设置比例系数 P
//		pid2.Ki = i;    // 设置积分系数 I
//		pid2.Kd = d;    // 设置微分系数 D
//	pid3.Kp = p;    // 设置比例系数 P
//		pid3.Ki = i;    // 设置积分系数 I
//		pid3.Kd = d;    // 设置微分系数 D
//	pid4.Kp = p;    // 设置比例系数 P
//		pid4.Ki = i;    // 设置积分系数 I
//		pid4.Kd = d;    // 设置微分系数 D
		pid5.Kp = p;    // 设置比例系数 P
		pid5.Ki = i;    // 设置积分系数 I
		pid5.Kd = d;    // 设置微分系数 D
}

/**
  * @brief  PID algorithm implementation
  * @param  actual_val: Actual value
	*	@note 	None
  * @retval Output after PID calculation
  */
float PID_realize(_pid *pid, float actual_val)
{
	if(actual_val>2000)
	{
		actual_val = 0;
	}
	else if(actual_val<-2000)
	{
		actual_val = 0;
	}

	pid->err = pid->target_val - actual_val;
	pid->actual_val += pid->Kp*(pid->err - pid->err_next)
		 + pid->Ki*pid->err
		 + pid->Kd*(pid->err - 2 * pid->err_next + pid->err_last);
    pid->err_last = pid->err_next;
	pid->err_next = pid->err;

	return pid->actual_val;
}

float PID_location(_pid *pid, float actual_val)
{
	// Calculate error
	pid->err = pid->target_val - actual_val;
	
	// Integral accumulation (accumulate first then use)
	pid->integral += pid->err;
	
	// Integral limiting (prevent integral saturation)
	#define INTEGRAL_MAX_LIMIT 30.0f
	if (pid->integral > INTEGRAL_MAX_LIMIT) {
		pid->integral = INTEGRAL_MAX_LIMIT;
	} else if (pid->integral < -INTEGRAL_MAX_LIMIT) {
		pid->integral = -INTEGRAL_MAX_LIMIT;
	}
	
	// Position PID calculation: output = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                    // Proportional term
	                + pid->Ki * pid->integral               // Integral term (using accumulated value)
	                + pid->Kd * (pid->err - pid->err_last); // Derivative term
	
	// Update error history
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

//float PID_Angle(float actual_val)
//{
//	/*计算目标值与实际值的误差*/
//    pid.err=pid.target_val-actual_val;
//    /*误差累积*/
//    pid.integral+=pid.err;
//		/*PID算法实现*/
//    pid.actual_val=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
//		/*误差传递*/
//    pid.err_last=pid.err;
//	if(pid.target_val-actual_val <2 && pid.target_val-actual_val >-2)
//	{
//		return 0;
//	}
//		/*返回当前实际值*/
//    return pid.actual_val;
//}

static float VariableIntegralCoefficient(float error)
{
  float factor=0.0;

 if(abs(error)<= (pid.absmin))
  {
   factor=1.0;
  }
  else if(abs(error) > pid.absmax)
  {
   factor=0.0;
  }
  else
  {
   factor=( pid.absmax-abs(error))/( pid.absmax- pid.absmin);
  }

  return factor;
}

/**
 * @brief Angle loop PID control (position PID, with dead zone and shortest path)
 * @param pid PID controller pointer
 * @param target_angle Target angle (-180~180 degrees)
 * @param actual_angle Current angle (-180~180 degrees)
 * @return PID output value (control quantity)
 * @note Automatically selects shortest path turning, dead zone 1.5 degrees
 */
float PID_Angle(_pid *pid, float target_angle, float actual_angle)
{
	// ========== Step 1: Calculate angle error (shortest path) ==========
	float angle_error = target_angle - actual_angle;
	
	// Handle -180/180 boundary crossing problem (select shortest path)
	if (angle_error > 180.0f) {
		angle_error -= 360.0f;  // Example: target=-170, current=170, error=20, correct to -340 -> clockwise turn 20 degrees
	} else if (angle_error < -180.0f) {
		angle_error += 360.0f;  // Example: target=170, current=-170, error=340, correct to -20 -> counterclockwise turn 20 degrees
	}
	
	// ========== Step 2: Dead zone control (no action if less than 1.5 degrees) ==========
	#define ANGLE_DEADZONE 1.5f
	if (fabsf(angle_error) < ANGLE_DEADZONE) {
		// In dead zone, clear output and integral to avoid jitter
		pid->actual_val = 0.0f;
		pid->integral = 0.0f;  // Prevent integral accumulation
		pid->err_last = 0.0f;
		return 0.0f;
	}
	
	// ========== Step 3: Position PID calculation ==========
	pid->err = angle_error;
	
	// Integral accumulation (variable integral coefficient)
	float factor = VariableIntegralCoefficient(pid->err);
	float effective_Ki = pid->Ki * factor;
	pid->integral += pid->err;
	
	// Integral limiting (prevent integral saturation)
	#define ANGLE_INTEGRAL_LIMIT 50.0f
	if (pid->integral > ANGLE_INTEGRAL_LIMIT) {
		pid->integral = ANGLE_INTEGRAL_LIMIT;
	} else if (pid->integral < -ANGLE_INTEGRAL_LIMIT) {
		pid->integral = -ANGLE_INTEGRAL_LIMIT;
	}
	
	// Position PID: output = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                      // Proportional term
	                + effective_Ki * pid->integral            // Integral term (variable integral)
	                + pid->Kd * (pid->err - pid->err_last);   // Derivative term
	
	// Update error history
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

/**
 * @brief Position loop PID control (position PID, with dead zone and variable integral)
 * @param pid PID controller pointer
 * @param target_position Target position (centimeters)
 * @param actual_position Current position (centimeters)
 * @return PID output value (speed control quantity, RPS)
 * @note Dead zone 0.5cm, uses variable integral algorithm
 */
float PID_Position(_pid *pid, float target_position, float actual_position)
{
	// ========== Step 1: Calculate position error ==========
	float position_error = target_position - actual_position;
	
	// ========== Step 2: Dead zone control (no action if less than 0.5cm) ==========
	#define POSITION_DEADZONE 0.5f  // 0.5cm dead zone
	if (fabsf(position_error) < POSITION_DEADZONE) {
		// In dead zone, clear output and integral to avoid jitter
		pid->actual_val = 0.0f;
		pid->integral = 0.0f;  // Prevent integral accumulation
		pid->err_last = 0.0f;
		return 0.0f;
	}
	
	// ========== Step 3: Position PID calculation ==========
	pid->err = position_error;
	
	// Integral accumulation (variable integral coefficient)
	float factor = VariableIntegralCoefficient(pid->err);
	float effective_Ki = pid->Ki * factor;
	pid->integral += pid->err;
	
	// Integral limiting (prevent integral saturation)
	#define POSITION_INTEGRAL_LIMIT 100.0f
	if (pid->integral > POSITION_INTEGRAL_LIMIT) {
		pid->integral = POSITION_INTEGRAL_LIMIT;
	} else if (pid->integral < -POSITION_INTEGRAL_LIMIT) {
		pid->integral = -POSITION_INTEGRAL_LIMIT;
	}
	
	// Position PID: output = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                      // Proportional term
	                + effective_Ki * pid->integral            // Integral term (variable integral)
	                + pid->Kd * (pid->err - pid->err_last);   // Derivative term
	
	// Update error history
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

