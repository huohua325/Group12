#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "main.h"
#include "motor.h"
#include "PID.h"
#include "encoder.h"

// ============================================================================
// Hardware Definition Area
// ============================================================================
extern UART_HandleTypeDef huart4;
// Motor identifiers
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2

// Left motor dual PWM channel definitions (H-bridge drive)
#define AIN1_PWM_CHANNEL TIM_CHANNEL_1   // PA8  - Left motor forward PWM
#define AIN2_PWM_CHANNEL TIM_CHANNEL_2   // PA9  - Left motor reverse PWM

// Right motor dual PWM channel definitions (H-bridge drive)
#define BIN1_PWM_CHANNEL TIM_CHANNEL_3   // PA10 - Right motor forward PWM
#define BIN2_PWM_CHANNEL TIM_CHANNEL_4   // PA11 - Right motor reverse PWM

// External timer handles
extern TIM_HandleTypeDef htim1;  // PWM output timer
extern TIM_HandleTypeDef htim2;  // Physical right wheel encoder (PA15/PB3)
extern TIM_HandleTypeDef htim3;  // Physical left wheel encoder (PC6/PC7)

// ============================================================================
// Speed Control Parameter Configuration Area (PID Removed)
// ============================================================================

#define SPEED_LPF_ALPHA    0.5f    // Speed low-pass filter coefficient (0~1, larger = faster response)
#define CONTROL_INTERVAL   20      // Control period (milliseconds, 50Hz)

// Limit angle loop output (prevent excessive differential speed)
#define ANGLE_OUTPUT_LIMIT 2.0f  // Maximum angle loop output (RPS difference)
// Limit position loop output (prevent excessive speed, avoid wheelie)
#define POSITION_OUTPUT_LIMIT 2.0f  // Maximum position loop output (RPS) - reduced limit to avoid wheelie
// Soft start: position loop output gradual change (avoid sudden acceleration)
#define POSITION_RAMP_RATE 0.1f  // Gradual rate (0~1, smaller = smoother)
// Soft start: angle loop output gradual change (avoid sudden turning)
#define ANGLE_RAMP_RATE 0.2f  // Gradual rate (0~1, smaller = smoother)
// ============================================================================
// Global Variable Area
// ============================================================================


// ---------- Movement Direction (for ODO use) ----------
int8_t target_direction = 0;             // Movement direction (1=forward, -1=backward, 0=stop) - exported for ODO use

// ---------- Actual Speed (after filtering) ----------
static float actual_speed_left = 0.0f;   // Left wheel actual speed (RPS)
static float actual_speed_right = 0.0f;  // Right wheel actual speed (RPS)

// ---------- Control Timing ----------
static uint32_t last_control_time = 0;   // Last control timestamp
static float current_yaw_angle;          // Current yaw angle (-180~180, calibrated)
static float raw_yaw_angle;              // Gyroscope raw angle (-180~180, uncalibrated)
static float last_raw_yaw_angle = 0.0f; // Last raw angle (for detecting boundary crossing)

// Gyroscope accumulated angle tracking (for calibration)
static float accumulated_raw_angle = 0.0f;      // Raw accumulated angle (uncalibrated)
static float accumulated_calibrated_angle = 0.0f; // Calibrated accumulated angle
static uint8_t angle_tracking_initialized = 0;  // Whether initialized

// Gyroscope angle calibration coefficient
static float gyro_angle_calibration = 0.99557522f;  // Adjusted based on actual measurement
// Calculation formula: calibration coefficient = actual angle / gyroscope angle
// Example: turn 3600 degrees, gyroscope shows 3616 degrees, coefficient = 3600/3616 = 0.99557522

// ---------- Motor Speed Loop PID Controllers ----------
static _pid pid_motor_left;              // Left wheel speed loop PID
static _pid pid_motor_right;             // Right wheel speed loop PID

// ---------- Angle Loop PID Controller ----------
static _pid pid_angle;                   // Angle loop PID (outer loop)
static float target_yaw_angle = 0.0f;    // Target yaw angle (-180~180 degrees)
static float angle_control_output = 0.0f; // Angle loop output (superimposed on speed loop)

// ---------- Position Loop PID Controller ----------
static _pid pid_position;                // Position loop PID (outer loop, parallel with angle loop)
static float target_position = 0.0f;     // Target position (centimeters)
static float actual_position = 0.0f;     // Current accumulated position (centimeters)
static float position_control_output = 0.0f; // Position loop output (speed control quantity)

// Tire parameters
#define WHEEL_DIAMETER_CM  7.5f          // Tire diameter (centimeters)
#define WHEEL_CIRCUMFERENCE_CM (WHEEL_DIAMETER_CM * 3.14159265f) // Tire circumference (centimeters)

// Position calibration coefficient (for compensating wheel diameter error)
static float position_calibration_factor = 0.88888889f;  // Default 1.0, adjustable via function

// Position loop soft start filter (needs to be externally resettable)
static float position_output_filtered = 0.0f;

// Angle loop soft start filter
static float angle_output_filtered = 0.0f;

// ---------- PWM Output Variables ----------
static float pwm_left = 0.0f;
static float pwm_right = 0.0f;

// ---------- Target Speed (RPS) ----------
static float target_speed_left = 0.0f;   // Left wheel target speed (revolutions per second)
static float target_speed_right = 0.0f;  // Right wheel target speed (revolutions per second)
static float base_speed_left = 0.0f;     // Base target speed (before angle loop superposition)
static float base_speed_right = 0.0f;    // Base target speed (before angle loop superposition)
// ============================================================================
// Low-level Motor Control Functions
// ============================================================================

/**
 * @brief Update yaw angle data (called from UART5 sensor data) ⭐
 * @param yaw_angle Gyroscope raw yaw angle (degrees, range -180~180)
 * @note Automatically tracks accumulated angle changes and applies calibration coefficient
 * @note Calculates accumulated rotation angle by detecting ±180 boundary crossing
 */
void Motor_UpdateYawAngle(float yaw_angle) {
    raw_yaw_angle = yaw_angle;  // Save current raw angle
    
    // Initialize on first call
    if (!angle_tracking_initialized) {
        last_raw_yaw_angle = yaw_angle;
        accumulated_raw_angle = yaw_angle;
        accumulated_calibrated_angle = yaw_angle * gyro_angle_calibration;
        angle_tracking_initialized = 1;
        
        // Use current angle directly (first time)
        current_yaw_angle = yaw_angle * gyro_angle_calibration;
        
        // Normalize
        while (current_yaw_angle > 180.0f) current_yaw_angle -= 360.0f;
        while (current_yaw_angle < -180.0f) current_yaw_angle += 360.0f;
        
        return;
    }
    
    // Calculate angle change (considering boundary crossing)
    float delta_angle = yaw_angle - last_raw_yaw_angle;
    
    // Detect boundary crossing
    if (delta_angle > 180.0f) {
        // Cross from +180 to -180 (counterclockwise boundary crossing)
        delta_angle -= 360.0f;
    } else if (delta_angle < -180.0f) {
        // Cross from -180 to +180 (clockwise boundary crossing)
        delta_angle += 360.0f;
    }
    
    // Accumulate raw angle (uncalibrated)
    accumulated_raw_angle += delta_angle;
    
    // Apply calibration coefficient to accumulated angle
    accumulated_calibrated_angle = accumulated_raw_angle * gyro_angle_calibration;
    
    // Calculate current angle from accumulated angle (normalize to -180~180)
    current_yaw_angle = accumulated_calibrated_angle;
    while (current_yaw_angle > 180.0f) current_yaw_angle -= 360.0f;
    while (current_yaw_angle < -180.0f) current_yaw_angle += 360.0f;
    
    // Update last angle
    last_raw_yaw_angle = yaw_angle;
}

/**
 * @brief Motor initialization (configure 4 PWM channels)
 * @param htim PWM timer handle (TIM1)
 * @note Use 4 PWM channels to control dual H-bridge: AIN1/AIN2 (left motor), BIN1/BIN2 (right motor)
 */
void Motor_Init(TIM_HandleTypeDef *htim) {
    // Start 4 PWM channels
    HAL_TIM_PWM_Start(htim, AIN1_PWM_CHANNEL);  // PA8
    HAL_TIM_PWM_Start(htim, AIN2_PWM_CHANNEL);  // PA9
    HAL_TIM_PWM_Start(htim, BIN1_PWM_CHANNEL);  // PA10
    HAL_TIM_PWM_Start(htim, BIN2_PWM_CHANNEL);  // PA11

    // Default stop motors (all PWM set to 0)
    __HAL_TIM_SET_COMPARE(htim, AIN1_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(htim, AIN2_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(htim, BIN1_PWM_CHANNEL, 0);
    __HAL_TIM_SET_COMPARE(htim, BIN2_PWM_CHANNEL, 0);

    // Check encoder initial count values (for debugging)
    int16_t leftCount = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
    int16_t rightCount = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
    (void)leftCount;   // Avoid unused variable warning
    (void)rightCount;
}

/**
 * @brief Set individual motor speed and direction (dual PWM control)
 * @param htim PWM timer handle (TIM1)
 * @param motor Motor identifier (MOTOR_LEFT or MOTOR_RIGHT)
 * @param speedRatio Speed ratio (-1.0~1.0, positive=forward, negative=reverse)
 * 
 * @note Left motor control logic (H-bridge drive):
 *       - speedRatio > 0 (forward): AIN1 outputs PWM, AIN2 set to low level
 *       - speedRatio < 0 (reverse): AIN1 set to low level, AIN2 outputs PWM
 *       - speedRatio = 0 (stop): Both AIN1 and AIN2 set to low level
 * 
 * @note Right motor control logic (H-bridge drive):
 *       - speedRatio > 0 (forward): BIN1 outputs PWM, BIN2 set to low level
 *       - speedRatio < 0 (reverse): BIN1 set to low level, BIN2 outputs PWM
 *       - speedRatio = 0 (stop): Both BIN1 and BIN2 set to low level
 */
void Motor_SetSpeed(TIM_HandleTypeDef *htim, int motor, float speedRatio) {
    // Limit processing
    if (speedRatio > 1.0f) speedRatio = 1.0f;
    if (speedRatio < -1.0f) speedRatio = -1.0f;
    
    // Extract direction and PWM duty cycle
    float abs_ratio = fabsf(speedRatio);  // Take absolute value as PWM duty cycle
    uint32_t pwm_value = (uint32_t)(abs_ratio * 65535);
    
    // ========== Left Motor Control (AIN1 / AIN2) ==========
    if (motor == MOTOR_RIGHT) {
        if (speedRatio > 0.0f) {
            // Forward: AIN1 outputs PWM, AIN2 set to low level
            __HAL_TIM_SET_COMPARE(htim, AIN1_PWM_CHANNEL, pwm_value);  // PA8 outputs PWM
            __HAL_TIM_SET_COMPARE(htim, AIN2_PWM_CHANNEL, 0);          // PA9 set to 0
        } 
        else if (speedRatio < 0.0f) {
            // Reverse: AIN1 set to low level, AIN2 outputs PWM
            __HAL_TIM_SET_COMPARE(htim, AIN1_PWM_CHANNEL, 0);          // PA8 set to 0
            __HAL_TIM_SET_COMPARE(htim, AIN2_PWM_CHANNEL, pwm_value);  // PA9 outputs PWM
        } 
        else {
            // Stop: Both AIN1 and AIN2 set to low level
            __HAL_TIM_SET_COMPARE(htim, AIN1_PWM_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(htim, AIN2_PWM_CHANNEL, 0);
        }
    }
    // ========== Right Motor Control (BIN1 / BIN2) ==========
    else if (motor == MOTOR_LEFT) {
        if (speedRatio < 0.0f) {
            // Forward: BIN1 outputs PWM, BIN2 set to low level
            __HAL_TIM_SET_COMPARE(htim, BIN1_PWM_CHANNEL, pwm_value);  // PA10 outputs PWM
            __HAL_TIM_SET_COMPARE(htim, BIN2_PWM_CHANNEL, 0);          // PA11 set to 0
        } 
        else if (speedRatio > 0.0f) {
            // Reverse: BIN1 set to low level, BIN2 outputs PWM
            __HAL_TIM_SET_COMPARE(htim, BIN1_PWM_CHANNEL, 0);          // PA10 set to 0
            __HAL_TIM_SET_COMPARE(htim, BIN2_PWM_CHANNEL, pwm_value);  // PA11 outputs PWM
        } 
        else {
            // Stop: Both BIN1 and BIN2 set to low level
            __HAL_TIM_SET_COMPARE(htim, BIN1_PWM_CHANNEL, 0);
            __HAL_TIM_SET_COMPARE(htim, BIN2_PWM_CHANNEL, 0);
        }
    }
}



// ============================================================================
// Motor Control System (PID closed-loop removed, only basic control retained)
// ============================================================================

/**
 * @brief Initialize motor control system (with speed loop PID and angle loop PID)
 * @note Initialize control parameters and PID controllers (dual loop control: angle loop + speed loop)
 */
void Motor_PID_Init(void) {
    // 初始化目标速度和方向
    target_direction = 0;
    target_speed_left = 0.0f;
    target_speed_right = 0.0f;
    base_speed_left = 0.0f;
    base_speed_right = 0.0f;
    
    // 清零速度滤波器
    actual_speed_left = 0.0f;
    actual_speed_right = 0.0f;
    
    // 清零PWM输出
    pwm_left = 0.0f;
    pwm_right = 0.0f;
    
    // ========== 初始化角度环PID（外环）==========
    pid_angle.target_val = 0.0f;           // 目标角度（-180~180度）
    pid_angle.actual_val = 0.0f;           // PID输出值
    pid_angle.err = 0.0f;                  // 当前角度误差
    pid_angle.err_last = 0.0f;             // 上次角度误差
    pid_angle.err_next = 0.0f;             // 预留
    pid_angle.integral = 0.0f;             // 积分累积值
    
    // 角度环PID参数（需根据实际调试）
    pid_angle.Kp = 0.025f;    // 比例系数（角度误差->速度调整量）
    pid_angle.Ki = 0.002f;   // 积分系数（消除稳态角度误差）
    pid_angle.Kd = 0.04f;   // 微分系数（抑制转向震荡）
    
    // 变积分参数（用于PID_Angle内部）
    pid_angle.absmax = 30.0f;  // 误差大于30度时积分系数为0
    pid_angle.absmin = 3.0f;   // 误差小于3度时积分系数为1
    
    target_yaw_angle = 0.0f;
    angle_control_output = 0.0f;
    
    // ========== 初始化位置环PID（外环，与角度环并列）==========
    pid_position.target_val = 0.0f;        // 目标位置（厘米）
    pid_position.actual_val = 0.0f;        // PID输出值
    pid_position.err = 0.0f;               // 当前位置误差
    pid_position.err_last = 0.0f;          // 上次位置误差
    pid_position.err_next = 0.0f;          // 预留
    pid_position.integral = 0.0f;          // 积分累积值
    
    // 位置环PID参数（需根据实际调试）
    pid_position.Kp = 0.04f;    // 比例系数（位置误差->速度控制量）
    pid_position.Ki = 0.002f;   // 积分系数（消除稳态位置误差）
    pid_position.Kd = 0.02f;    // 微分系数（抑制震荡）
    
    // 变积分参数（用于PID_Position内部）
    pid_position.absmax = 20.0f;  // 误差大于20cm时积分系数为0
    pid_position.absmin = 2.0f;   // 误差小于2cm时积分系数为1
    
    target_position = 0.0f;
    actual_position = 0.0f;
    position_control_output = 0.0f;
    
    // ========== 初始化左轮速度环PID（内环）==========
    pid_motor_left.target_val = 0.0f;      // 目标速度（RPS）
    pid_motor_left.actual_val = 0.0f;      // PID输出值
    pid_motor_left.err = 0.0f;             // 当前误差
    pid_motor_left.err_last = 0.0f;        // 上次误差
    pid_motor_left.err_next = 0.0f;        // 预留
    pid_motor_left.integral = 0.0f;        // 积分累积值
    
    // PID参数（位置式PID，需根据实际调试）
    pid_motor_left.Kp = 0.8f;    // 比例系数（主要响应项）
    pid_motor_left.Ki = 0.02f;   // 积分系数（消除稳态误差）
    pid_motor_left.Kd = 0.2f;    // 微分系数（抑制震荡）
    
    // ========== 初始化右轮速度环PID（内环）==========
    pid_motor_right.target_val = 0.0f;
    pid_motor_right.actual_val = 0.0f;
    pid_motor_right.err = 0.0f;
    pid_motor_right.err_last = 0.0f;
    pid_motor_right.err_next = 0.0f;
    pid_motor_right.integral = 0.0f;
    
    // PID参数（位置式PID）
    pid_motor_right.Kp = 0.8f;
    pid_motor_right.Ki = 0.02f;
    pid_motor_right.Kd = 0.2f;
    
    last_control_time = HAL_GetTick();
}
extern uint8_t CTL;
/**
 * @brief 三环PID控制循环（位置环+角度环+速度环）
 * @param htim_pwm PWM定时器句柄（TIM1）
 * @param htim_encoder_left 左轮编码器定时器句柄（TIM3）
 * @param htim_encoder_right 右轮编码器定时器句柄（TIM2）
 * @note 外环：位置环+角度环（并列），内环：速度环控制电机转速
 */
void Motor_PID_Control(TIM_HandleTypeDef *htim_pwm, TIM_HandleTypeDef *htim_encoder_left, 
                       TIM_HandleTypeDef *htim_encoder_right) {
    uint32_t current_time = HAL_GetTick();
    
    // 每20ms执行一次PID控制（50Hz）
    if (current_time - last_control_time < CONTROL_INTERVAL) {
        return;
    }
    
    float dt = (current_time - last_control_time) / 1000.0f;
    last_control_time = current_time;
    
    // ========== 步骤1：读取编码器，获取当前速度 ==========
    float l_spd_raw, r_spd_raw;
    Encoder_GetSpeeds(htim_encoder_left, htim_encoder_right, &l_spd_raw, &r_spd_raw, dt);
    
    // 低通滤波，减少速度噪声
//    actual_speed_left  = (1.0f - SPEED_LPF_ALPHA) * actual_speed_left  + SPEED_LPF_ALPHA * l_spd_raw;
//    actual_speed_right = (1.0f - SPEED_LPF_ALPHA) * actual_speed_right + SPEED_LPF_ALPHA * r_spd_raw;
    actual_speed_left  =  l_spd_raw;
    actual_speed_right =  r_spd_raw;
    
    // ========== 步骤2：计算位置累积（两轮平均）==========
    // 计算本次采样周期内行驶的距离（厘米）
    float avg_speed_rps = (actual_speed_left + actual_speed_right) / 2.0f;  // 平均速度（RPS）
    float distance_increment = avg_speed_rps * WHEEL_CIRCUMFERENCE_CM * dt * position_calibration_factor;  // 本次行驶距离（厘米，含校准）
    actual_position += distance_increment;  // 累积到总位置
    
    // ========== 步骤3：位置环PID控制（外环1）==========
    // 调用PID_Position计算位置环输出（速度控制量）
    position_control_output = PID_Position(&pid_position, target_position, actual_position);
    

    if (position_control_output > POSITION_OUTPUT_LIMIT) {
        position_control_output = POSITION_OUTPUT_LIMIT;
    } else if (position_control_output < -POSITION_OUTPUT_LIMIT) {
        position_control_output = -POSITION_OUTPUT_LIMIT;
    }
    

    position_output_filtered = position_output_filtered * (1.0f - POSITION_RAMP_RATE) 
                              + position_control_output * POSITION_RAMP_RATE;
    position_control_output = position_output_filtered;
    
    // ========== 步骤4：角度环PID控制（外环2）==========
    // 调用PID_Angle计算角度环输出（差速控制量）
    angle_control_output = PID_Angle(&pid_angle, target_yaw_angle, current_yaw_angle);
    

    if (angle_control_output > ANGLE_OUTPUT_LIMIT) {
        angle_control_output = ANGLE_OUTPUT_LIMIT;
    } else if (angle_control_output < -ANGLE_OUTPUT_LIMIT) {
        angle_control_output = -ANGLE_OUTPUT_LIMIT;
    }
    

    angle_output_filtered = angle_output_filtered * (1.0f - ANGLE_RAMP_RATE) 
                           + angle_control_output * ANGLE_RAMP_RATE;
    angle_control_output = angle_output_filtered;
    
    // ========== 步骤5：合成速度环目标 ==========
    // 位置环输出作为基础速度，角度环输出作为差速调整
    target_speed_left = base_speed_left + position_control_output - angle_control_output;
    target_speed_right = base_speed_right + position_control_output + angle_control_output;
    
    // ========== 步骤6：左轮速度环PID控制（内环）==========
    pid_motor_left.target_val = target_speed_left;   // 设置目标速度
    float pid_output_left = PID_location(&pid_motor_left, actual_speed_left);
    
    // 限幅处理（-1.0 ~ 1.0）
    if (pid_output_left > 1.0f) pid_output_left = 1.0f;
    if (pid_output_left < -1.0f) pid_output_left = -1.0f;
    
    pwm_left = pid_output_left;

    // ========== 步骤7：右轮速度环PID控制（内环）==========
    pid_motor_right.target_val = target_speed_right;  // 设置目标速度
    float pid_output_right = PID_location(&pid_motor_right, actual_speed_right);
    
    // 限幅处理（-1.0 ~ 1.0）
    if (pid_output_right > 1.0f) pid_output_right = 1.0f;
    if (pid_output_right < -1.0f) pid_output_right = -1.0f;
    
    pwm_right = pid_output_right;
    
    // ========== 步骤8：应用PWM到电机 ==========
    if(CTL!=1){
    Motor_SetSpeed(htim_pwm, MOTOR_LEFT, pwm_left);
    Motor_SetSpeed(htim_pwm, MOTOR_RIGHT, pwm_right);
    }
    // ========== 步骤9：检测死区并发送完成信号 ==========
    // 计算位置和角度误差
    float position_error = fabsf(actual_position - target_position);
    float angle_error = target_yaw_angle - current_yaw_angle;
    if (angle_error > 180.0f) angle_error -= 360.0f;
    if (angle_error < -180.0f) angle_error += 360.0f;
    angle_error = fabsf(angle_error);
    
    // 检查是否都在死区内
    uint8_t in_position_deadzone = (position_error < 0.5f);
    uint8_t in_angle_deadzone = (angle_error < 1.5f);

    
    // ========== 步骤10：调试输出（位置环+角度环信息，每10次输出一次）==========
    static uint8_t debug_counter = 0;
    if (++debug_counter >= 10) {
        debug_counter = 0;

//        // 位置环+角度环调试信息
//        char debug_msg[250];
//        snprintf(debug_msg, sizeof(debug_msg),
//                "[PID] Pos:%.1f->%.1f(%.3f) Ang:%.1f->%.1f(%.3f) | L:%.2f R:%.2f | PWM:%.2f/%.2f\r\n",
//                target_position, actual_position, position_control_output,
//                target_yaw_angle, current_yaw_angle, angle_control_output,
//                actual_speed_left, actual_speed_right, pwm_left, pwm_right);
//        HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    }
}


/**
 * @brief 设置左右轮基础目标速度（角度环叠加前）
 * @param left_rps 左轮目标速度（RPS，圈/秒，可为负值表示后退）
 * @param right_rps 右轮目标速度（RPS，圈/秒，可为负值表示后退）
 * @note 角度环输出会叠加到此速度上，形成最终速度环目标
 */
void Motor_SetTargetSpeed(float left_rps, float right_rps) {
    base_speed_left = left_rps;
    base_speed_right = right_rps;
}

/**
 * @brief 设置目标偏航角（角度环控制）⭐
 * @param target_angle 目标偏航角（-180~180度）
 * @note 角度环会自动选择最短路径转向，死区1.5度
 * @example
 *   Motor_SetTargetAngle(0.0f);    // 朝向正北
 *   Motor_SetTargetAngle(90.0f);   // 朝向正东
 *   Motor_SetTargetAngle(-90.0f);  // 朝向正西
 */
void Motor_SetTargetAngle(float target_angle) {
    target_yaw_angle = target_angle;
}

/**
 * @brief 获取当前目标角度
 * @return 目标偏航角（-180~180度）
 */
float Motor_GetTargetAngle(void) {
    return target_yaw_angle;
}

/**
 * @brief 获取角度环输出值（用于调试）
 * @return 角度环控制输出（RPS差值）
 */
float Motor_GetAngleControlOutput(void) {
    return angle_control_output;
}

/**
 * @brief 设置目标位置（位置环控制）⭐
 * @param target_pos 目标位置（厘米，相对于位置清零点）
 * @note 位置环会自动控制车辆前进到目标位置，死区0.5cm
 * @note 配合Motor_ResetPosition()使用，先清零位置再设置目标
 * @example
 *   Motor_ResetPosition();          // 清零位置
 *   Motor_SetTargetPosition(50.0f); // 前进50cm
 *   Motor_SetTargetPosition(-30.0f); // 后退30cm
 */
void Motor_SetTargetPosition(float target_pos) {
    target_position = target_pos;
}

/**
 * @brief 获取当前累积位置
 * @return 当前位置（厘米）
 */
float Motor_GetActualPosition(void) {
    return actual_position;
}

/**
 * @brief 获取位置环输出值（用于调试）
 * @return 位置环控制输出（RPS）
 */
float Motor_GetPositionControlOutput(void) {
    return position_control_output;
}

/**
 * @brief 获取当前设置的目标速度
 * @param left_rps 输出：左轮目标速度（RPS）
 * @param right_rps 输出：右轮目标速度（RPS）
 */
void Motor_GetTargetSpeed(float *left_rps, float *right_rps) {
    *left_rps = target_speed_left;
    *right_rps = target_speed_right;
}

/**
 * @brief 获取实际速度（经过滤波的速度值）
 * @param left_rps 输出：左轮实际速度（单位：RPS）
 * @param right_rps 输出：右轮实际速度（单位：RPS）
 */
void Motor_GetActualSpeed(float *left_rps, float *right_rps) {
    *left_rps = actual_speed_left;
    *right_rps = actual_speed_right;
}

/**
 * @brief 获取当前PWM输出值
 * @param left_pwm 输出：左轮PWM（-1.0 ~ 1.0）
 * @param right_pwm 输出：右轮PWM（-1.0 ~ 1.0）
 */
void Motor_GetPWM(float *left_pwm, float *right_pwm) {
    *left_pwm = pwm_left;
    *right_pwm = pwm_right;
}

/**
 * @brief 重置PID控制器状态（清除积分累积）
 * @note 当电机切换方向或出现震荡时调用，防止积分饱和
 * @note 会同时重置位置环、角度环和速度环的PID状态
 */
void Motor_ResetPID(void) {
    // 清除位置环PID状态
    pid_position.actual_val = 0.0f;
    pid_position.err = 0.0f;
    pid_position.err_last = 0.0f;
    pid_position.integral = 0.0f;  // 关键：清除积分累积
    position_control_output = 0.0f;
    
    // 清除角度环PID状态
    pid_angle.actual_val = 0.0f;
    pid_angle.err = 0.0f;
    pid_angle.err_last = 0.0f;
    pid_angle.integral = 0.0f;  // 关键：清除积分累积
    angle_control_output = 0.0f;
    
    // 清除左轮PID状态
    pid_motor_left.actual_val = 0.0f;
    pid_motor_left.err = 0.0f;
    pid_motor_left.err_last = 0.0f;
    pid_motor_left.integral = 0.0f;  // 关键：清除积分累积
    
    // 清除右轮PID状态
    pid_motor_right.actual_val = 0.0f;
    pid_motor_right.err = 0.0f;
    pid_motor_right.err_last = 0.0f;
    pid_motor_right.integral = 0.0f;
    
    // 清除软启动滤波器
    position_output_filtered = 0.0f;
    angle_output_filtered = 0.0f;
    
    // 清零PWM输出
    pwm_left = 0.0f;
    pwm_right = 0.0f;
}

/**
 * @brief 重置位置累积（清零里程计）⭐
 * @note 用于位置环控制，清零当前累积位置
 * @note 同时清零目标位置和位置环PID状态
 * @example
 *   Motor_ResetPosition();          // 清零位置
 *   Motor_SetTargetPosition(50.0f); // 从当前位置前进50cm
 */
void Motor_ResetPosition(void) {
    actual_position = 0.0f;
    target_position = 0.0f;
    
    // 清除位置环PID状态
    pid_position.actual_val = 0.0f;
    pid_position.err = 0.0f;
    pid_position.err_last = 0.0f;
    pid_position.integral = 0.0f;
    position_control_output = 0.0f;
    
    // 清除软启动滤波器
    position_output_filtered = 0.0f;
    angle_output_filtered = 0.0f;
}

/**
 * @brief 设置位置校准系数⭐
 * @param factor 校准系数（例如：1.02表示实际距离是编码器计算值的1.02倍）
 * @note 用于补偿轮径测量误差，提高位置精度
 * @note 如果小车实际前进距离小于目标，增大系数；反之减小系数
 * @example
 *   // 目标前进100cm，实际只走了98cm
 *   Motor_SetPositionCalibration(100.0f / 98.0f);  // = 1.02
 */
void Motor_SetPositionCalibration(float factor) {
    if (factor > 0.5f && factor < 1.5f) {  // 限制在合理范围内
        position_calibration_factor = factor;
    }
}

/**
 * @brief 获取当前位置校准系数
 * @return 校准系数
 */
float Motor_GetPositionCalibration(void) {
    return position_calibration_factor;
}

/**
 * @brief 设置陀螺仪角度校准系数⭐
 * @param factor 校准系数（实际角度/陀螺仪角度）
 * @note 用于补偿陀螺仪累积角度误差
 * @note 如果陀螺仪角度偏大，系数<1；如果偏小，系数>1
 * @note 系数范围限制在0.9~1.1之间
 * @example
 *   // 顺时针转3600度，陀螺仪显示3616度
 *   Motor_SetGyroCalibration(3600.0f / 3616.0f);  // = 0.99557522
 */
void Motor_SetGyroCalibration(float factor) {
    if (factor > 0.9f && factor < 1.1f) {  // 限制在合理范围内
        gyro_angle_calibration = factor;
    }
}

/**
 * @brief 获取当前陀螺仪角度校准系数
 * @return 校准系数
 */
float Motor_GetGyroCalibration(void) {
    return gyro_angle_calibration;
}

/**
 * @brief 陀螺仪角度校准助手（自动计算校准系数）⭐
 * @param actual_rotation_deg 实际转动角度（度）
 * @param gyro_rotation_deg 陀螺仪测量的转动角度（度）
 * @return 计算得到的校准系数
 * @note 自动计算并设置校准系数
 * @example
 *   // 方法1：使用累积角度
 *   // 转10圈（3600度），陀螺仪显示3616度
 *   float factor = Motor_CalibrateGyro(3600.0f, 3616.0f);
 *   printf("校准系数：%.8f\n", factor);
 *   
 *   // 方法2：使用起止角度
 *   float start_angle = Motor_GetRawYawAngle();  // 记录起始角度
 *   // ... 转10圈 ...
 *   float end_angle = Motor_GetRawYawAngle();    // 记录结束角度
 *   float gyro_rotation = end_angle - start_angle;
 *   Motor_CalibrateGyro(3600.0f, gyro_rotation);
 */
float Motor_CalibrateGyro(float actual_rotation_deg, float gyro_rotation_deg) {
    if (fabsf(gyro_rotation_deg) < 360.0f) {
        // 转动角度太小，不建议校准
        return gyro_angle_calibration;  // 返回当前值
    }
    
    // 计算校准系数
    float new_factor = actual_rotation_deg / gyro_rotation_deg;
    
    // 应用校准系数
    Motor_SetGyroCalibration(new_factor);
    
    return new_factor;
}

/**
 * @brief 获取陀螺仪原始角度（未校准）
 * @return 原始偏航角（度，-180~180）
 * @note 用于调试，这是陀螺仪直接输出的归一化角度
 */
float Motor_GetRawYawAngle(void) {
    return raw_yaw_angle;
}

/**
 * @brief 获取陀螺仪校准后角度
 * @return 校准后偏航角（度，-180~180）
 * @note 这是实际使用的角度值
 */
float Motor_GetCalibratedYawAngle(void) {
    return current_yaw_angle;
}

/**
 * @brief 获取累积原始角度（未校准）
 * @return 累积转动角度（度，可超过±180）
 * @note 用于查看陀螺仪累积转动了多少度（未校准）
 */
float Motor_GetAccumulatedRawAngle(void) {
    return accumulated_raw_angle;
}

/**
 * @brief 获取累积校准角度
 * @return 累积转动角度（度，可超过±180，已校准）
 * @note 用于查看校准后累积转动了多少度
 */
float Motor_GetAccumulatedCalibratedAngle(void) {
    return accumulated_calibrated_angle;
}

/**
 * @brief 重置累积角度追踪⭐
 * @note 清零累积角度，但保持校准系数
 * @note 在重新标定或系统重启时调用
 */
void Motor_ResetAngleTracking(void) {
    accumulated_raw_angle = raw_yaw_angle;
    accumulated_calibrated_angle = raw_yaw_angle * gyro_angle_calibration;
    last_raw_yaw_angle = raw_yaw_angle;
    angle_tracking_initialized = 1;
}

// ============================================================================
// 高层控制命令（组合位置环和角度环）
// ============================================================================

/**
 * @brief 命令：前进指定距离（非阻塞）⭐
 * @param distance_cm 前进距离（厘米，正值=前进，负值=后退）
 * @param timeout_ms 超时时间（毫秒，保留参数但不使用）
 * @return 0=命令已设置
 * @note 非阻塞执行，设置目标后立即返回
 * @note 到达目标（死区0.5cm）后会通过UART4发送字符'Z'
 * @example
 *   Motor_Command_MoveForward(60.0f, 0);  // 前进60cm（非阻塞）
 *   // 在主循环中调用Motor_PID_Control()
 *   // 当收到'Z'时表示到达目标
 */
int Motor_Command_MoveForward(float distance_cm, uint32_t timeout_ms) {
    (void)timeout_ms;  // 未使用的参数
    
    // 记录起始位置
    float start_position = target_position;
    float target_pos = start_position + distance_cm;
    
    // 设置目标（非阻塞）
    Motor_SetTargetPosition(target_pos);
    Motor_SetTargetSpeed(0.0f, 0.0f);     // 基础速度为0，让位置环控制
    
    return 0;  // 立即返回
}

/**
 * @brief 命令：原地左转指定角度（非阻塞）⭐
 * @param angle_deg 转动角度（度数，正值=逆时针，负值=顺时针）
 * @param timeout_ms 超时时间（毫秒，保留参数但不使用）
 * @return 0=命令已设置
 * @note 非阻塞执行，设置目标后立即返回
 * @note 到达目标（死区1.5度）后会通过UART4发送字符'Z'
 * @example
 *   Motor_Command_TurnLeft(90.0f, 0);  // 左转90度（非阻塞）
 *   // 在主循环中调用Motor_PID_Control()
 *   // 当收到'Z'时表示到达目标
 */
int Motor_Command_TurnLeft(float angle_deg, uint32_t timeout_ms) {
    (void)timeout_ms;  // 未使用的参数
    
    // 直接在目标角度上累加（避免累积误差）
    target_yaw_angle += angle_deg;
    
    // 角度归一化到-180~180
    while (target_yaw_angle > 180.0f) target_yaw_angle -= 360.0f;
    while (target_yaw_angle < -180.0f) target_yaw_angle += 360.0f;
    
    // 设置基础速度为0（原地转向）
    Motor_SetTargetSpeed(0.0f, 0.0f);
    
    return 0;  // 立即返回
}

/**
 * @brief 命令：原地右转指定角度（非阻塞）⭐
 * @param angle_deg 转动角度（度数，正值=顺时针，负值=逆时针）
 * @param timeout_ms 超时时间（毫秒，保留参数但不使用）
 * @return 0=命令已设置
 * @note 非阻塞执行，设置目标后立即返回
 * @note 到达目标（死区1.5度）后会通过UART4发送字符'Z'
 * @example
 *   Motor_Command_TurnRight(90.0f, 0);  // 右转90度（非阻塞）
 *   // 在主循环中调用Motor_PID_Control()
 *   // 当收到'Z'时表示到达目标
 */
int Motor_Command_TurnRight(float angle_deg, uint32_t timeout_ms) {
    // 右转就是左转负角度
    return Motor_Command_TurnLeft(-angle_deg, timeout_ms);
}
