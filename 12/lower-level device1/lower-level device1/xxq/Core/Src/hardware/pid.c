#include "pid.h"
#include <math.h>

//定义全局变量

_pid pid;
_pid pid1;
_pid pid2;
_pid pid3;
_pid pid4;
_pid pid5;

uint8_t Location_Flag;

/**
  * @brief  PID参数初始化
	*	@note 	无
  * @retval 无
  */
void PID_param_init(void)
{
	/* 陀螺仪pid */

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
		/* 初始化参数 */

    pid1.actual_val=0.0;
    pid1.err=0.0;
    pid1.err_last=0.0;
	  pid1.err_next=0.0;

		pid1.Kp = 13;
		pid1.Ki = 4.2;
		pid1.Kd = 7;

		/* 初始化参数 */

    pid2.actual_val=0.0;
    pid2.err=0.0;
    pid2.err_last=0.0;
	  pid2.err_next=0.0;

		pid2.Kp = 13;
		pid2.Ki = 4.2;
		pid2.Kd = 7;

		/* 初始化参数 */
    pid3.target_val=0;
    pid3.actual_val=0.0;
    pid3.err=0.0;
    pid3.err_last=0.0;
	  pid3.err_next=0.0;

		pid3.Kp = 13;
		pid3.Ki = 4.2;
		pid3.Kd = 7;

			/* 初始化参数 */

    pid4.actual_val=0.0;
    pid4.err=0.0;
    pid4.err_last=0.0;
	  pid4.err_next=0.0;

		pid4.Kp = 13;
		pid4.Ki = 4.2;
		pid4.Kd = 7;

		/* 初始化参数 */

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
  * @brief  设置目标值
  * @param  val		目标值
	*	@note 	无
  * @retval 无---------------
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
  * @brief  设置比例、积分、微分系数
  * @param  p：比例系数 P
  * @param  i：积分系数 i
  * @param  d：微分系数 d
	*	@note 	无
  * @retval 无
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
  * @brief  PID算法实现
  * @param  actual_val:实际值
	*	@note 	无
  * @retval 通过PID计算后的输出
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
	// 计算误差
	pid->err = pid->target_val - actual_val;
	
	// 积分累加（先累加再使用）
	pid->integral += pid->err;
	
	// 积分限幅（防止积分饱和）
	#define INTEGRAL_MAX_LIMIT 30.0f
	if (pid->integral > INTEGRAL_MAX_LIMIT) {
		pid->integral = INTEGRAL_MAX_LIMIT;
	} else if (pid->integral < -INTEGRAL_MAX_LIMIT) {
		pid->integral = -INTEGRAL_MAX_LIMIT;
	}
	
	// 位置式PID计算：输出 = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                    // 比例项
	                + pid->Ki * pid->integral               // 积分项（使用累积值）
	                + pid->Kd * (pid->err - pid->err_last); // 微分项
	
	// 更新误差历史
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
 * @brief 角度环PID控制（位置式PID，带死区和最短路径）
 * @param pid PID控制器指针
 * @param target_angle 目标角度（-180~180度）
 * @param actual_angle 当前角度（-180~180度）
 * @return PID输出值（控制量）
 * @note 自动选择最短路径转向，死区1.5度
 */
float PID_Angle(_pid *pid, float target_angle, float actual_angle)
{
	// ========== 步骤1：计算角度误差（最短路径）==========
	float angle_error = target_angle - actual_angle;
	
	// 处理-180/180边界跨越问题（选择最短路径）
	if (angle_error > 180.0f) {
		angle_error -= 360.0f;  // 例如：目标=-170，当前=170，误差=20，修正为-340 -> 顺时针转20度
	} else if (angle_error < -180.0f) {
		angle_error += 360.0f;  // 例如：目标=170，当前=-170，误差=340，修正为-20 -> 逆时针转20度
	}
	
	// ========== 步骤2：死区控制（小于1.5度不动作）==========
	#define ANGLE_DEADZONE 1.5f
	if (fabsf(angle_error) < ANGLE_DEADZONE) {
		// 在死区内，清零输出和积分，避免抖动
		pid->actual_val = 0.0f;
		pid->integral = 0.0f;  // 防止积分累积
		pid->err_last = 0.0f;
		return 0.0f;
	}
	
	// ========== 步骤3：位置式PID计算 ==========
	pid->err = angle_error;
	
	// 积分累加（变积分系数）
	float factor = VariableIntegralCoefficient(pid->err);
	float effective_Ki = pid->Ki * factor;
	pid->integral += pid->err;
	
	// 积分限幅（防止积分饱和）
	#define ANGLE_INTEGRAL_LIMIT 50.0f
	if (pid->integral > ANGLE_INTEGRAL_LIMIT) {
		pid->integral = ANGLE_INTEGRAL_LIMIT;
	} else if (pid->integral < -ANGLE_INTEGRAL_LIMIT) {
		pid->integral = -ANGLE_INTEGRAL_LIMIT;
	}
	
	// 位置式PID：输出 = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                      // 比例项
	                + effective_Ki * pid->integral            // 积分项（变积分）
	                + pid->Kd * (pid->err - pid->err_last);   // 微分项
	
	// 更新误差历史
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

/**
 * @brief 位置环PID控制（位置式PID，带死区和变积分）
 * @param pid PID控制器指针
 * @param target_position 目标位置（厘米）
 * @param actual_position 当前位置（厘米）
 * @return PID输出值（速度控制量，RPS）
 * @note 死区0.5cm，使用变积分算法
 */
float PID_Position(_pid *pid, float target_position, float actual_position)
{
	// ========== 步骤1：计算位置误差 ==========
	float position_error = target_position - actual_position;
	
	// ========== 步骤2：死区控制（小于0.5cm不动作）==========
	#define POSITION_DEADZONE 0.5f  // 0.5cm死区
	if (fabsf(position_error) < POSITION_DEADZONE) {
		// 在死区内，清零输出和积分，避免抖动
		pid->actual_val = 0.0f;
		pid->integral = 0.0f;  // 防止积分累积
		pid->err_last = 0.0f;
		return 0.0f;
	}
	
	// ========== 步骤3：位置式PID计算 ==========
	pid->err = position_error;
	
	// 积分累加（变积分系数）
	float factor = VariableIntegralCoefficient(pid->err);
	float effective_Ki = pid->Ki * factor;
	pid->integral += pid->err;
	
	// 积分限幅（防止积分饱和）
	#define POSITION_INTEGRAL_LIMIT 100.0f
	if (pid->integral > POSITION_INTEGRAL_LIMIT) {
		pid->integral = POSITION_INTEGRAL_LIMIT;
	} else if (pid->integral < -POSITION_INTEGRAL_LIMIT) {
		pid->integral = -POSITION_INTEGRAL_LIMIT;
	}
	
	// 位置式PID：输出 = Kp*e + Ki*∫e + Kd*Δe
	pid->actual_val = pid->Kp * pid->err                      // 比例项
	                + effective_Ki * pid->integral            // 积分项（变积分）
	                + pid->Kd * (pid->err - pid->err_last);   // 微分项
	
	// 更新误差历史
	pid->err_last = pid->err;
	
	return pid->actual_val;
}

