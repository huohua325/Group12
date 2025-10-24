/**
 ******************************************************************************
 * @file    Motor.h
 * @brief   Dual Wheel Differential Motor Control Module (PID closed-loop control removed)
 * @author  Ye Jin
 * @date    2025
 ******************************************************************************
 * @description
 * This module provides motor drive control and encoder speed acquisition functions:
 * 1. Open-loop control: Directly set PWM duty cycle
 * 2. Encoder speed acquisition: Real-time wheel speed reading and filtering
 * 
 * Note: PID closed-loop control functionality has been removed, only basic motor drive and speed feedback remain.
 ******************************************************************************
 */

#ifndef __MOTOR_H
#define __MOTOR_H

#include <stdint.h>
#include "stm32f4xx_hal.h"

// ============================================================================
// Constant Definitions
// ============================================================================

// ---------- Motor Movement Direction Definitions ----------
#define MOTOR_FORWARD   1   // Forward
#define MOTOR_BACKWARD -1   // Backward
#define MOTOR_STOP      0   // Stop

// ---------- Motor Channel Identifiers ----------
#define MOTOR_LEFT   1      // Left motor
#define MOTOR_RIGHT  2      // Right motor

// ---------- Hardware Pin Definitions (Dual PWM H-bridge drive) ----------

// Left motor dual PWM channel definitions (TIM1)
#define AIN1_PWM_CHANNEL   TIM_CHANNEL_1  // PA8  - Left motor forward PWM
#define AIN2_PWM_CHANNEL   TIM_CHANNEL_2  // PA9  - Left motor reverse PWM

// Right motor dual PWM channel definitions (TIM1)
#define BIN1_PWM_CHANNEL   TIM_CHANNEL_3  // PA10 - Right motor forward PWM
#define BIN2_PWM_CHANNEL   TIM_CHANNEL_4  // PA11 - Right motor reverse PWM

// ============================================================================
// Low-level Motor Control Functions (Direct Hardware Operation)
// ============================================================================

/**
 * @brief Motor module initialization
 * @param htim PWM timer handle (usually TIM1)
 * @note Initialize GPIO and PWM channels, motors default to stop state
 * @note Must be called before using any motor control functions
 */
void Motor_Init(TIM_HandleTypeDef *htim);

/**
 * @brief Set individual motor speed and direction (Dual PWM H-bridge drive) ⭐
 * @param htim PWM timer handle (TIM1)
 * @param motor Motor channel (MOTOR_LEFT or MOTOR_RIGHT)
 * @param speedRatio Speed ratio (-1.0~1.0, positive=forward, negative=reverse, 0=stop)
 * 
 * @note Left motor control logic:
 *       - speedRatio > 0: AIN1 outputs PWM, AIN2 set to low level
 *       - speedRatio < 0: AIN1 set to low level, AIN2 outputs PWM
 *       - speedRatio = 0: Both AIN1 and AIN2 set to low level
 * 
 * @note Right motor control logic:
 *       - speedRatio > 0: BIN1 outputs PWM, BIN2 set to low level
 *       - speedRatio < 0: BIN1 set to low level, BIN2 outputs PWM
 *       - speedRatio = 0: Both BIN1 and BIN2 set to low level
 */
void Motor_SetSpeed(TIM_HandleTypeDef *htim, int motor, float speedRatio);

// ============================================================================
// Legacy Open-loop Control Functions All Removed
// ============================================================================
// Removed: Car_Forward, Car_Backward, Car_Stop, Car_TurnLeft, Car_TurnRight
// Please use new PID speed loop control: Motor_SetTargetSpeed()

// ============================================================================
// Control Functions (PID Closed-loop Removed)
// ============================================================================
// The following functions were originally used for PID closed-loop control, now simplified to state recording and basic control
// Usage:
//   1. Call Motor_PID_Init() to initialize
//   2. Periodically call Motor_PID_Control() in main loop to get encoder speed
//   3. Use Car_Forward_PID() / Car_Backward_PID() to record target speed state
//   4. Use Car_Stop_PID() to stop motors

/**
 * @brief Motor control system initialization (with speed loop PID)
 * @note Initialize control parameters and PID controllers, must be called before use
 */
void Motor_PID_Init(void);

/**
 * @brief Dual motor speed loop PID control loop ⭐
 * @param htim_pwm PWM timer handle (TIM1)
 * @param htim_encoder_left Left wheel encoder timer handle (TIM3)
 * @param htim_encoder_right Right wheel encoder timer handle (TIM2)
 * @note Uses PID_realize() function from pid.c to implement dual motor independent speed loop control
 * @note Must be called continuously in main while loop (internal 20ms timing)
 * @example
 *   while(1) {
 *       Motor_PID_Control(&htim1, &htim3, &htim2);
 *       // Other code...
 *   }
 */
void Motor_PID_Control(TIM_HandleTypeDef *htim_pwm, 
                       TIM_HandleTypeDef *htim_encoder_left, 
                       TIM_HandleTypeDef *htim_encoder_right);

/**
 * @brief Set left/right wheel base target speeds (before angle loop superposition) ⭐
 * @param left_rps Left wheel target speed (RPS, revolutions per second, can be negative for backward)
 * @param right_rps Right wheel target speed (RPS, revolutions per second, can be negative for backward)
 * @note Angle loop output will be superimposed on this speed to form final speed loop target
 * @example
 *   Motor_SetTargetSpeed(2.0f, 2.0f);  // Both wheels base speed 2 RPS (straight line)
 *   Motor_SetTargetSpeed(0.0f, 0.0f);  // Base speed 0 (in-place turning)
 */
void Motor_SetTargetSpeed(float left_rps, float right_rps);

/**
 * @brief Set target yaw angle (angle loop control) ⭐
 * @param target_angle Target yaw angle (-180~180 degrees)
 * @note Angle loop automatically selects shortest path turning, dead zone 1.5 degrees
 * @note Angle loop output will be superimposed on speed loop target to achieve differential turning
 * @example
 *   Motor_SetTargetAngle(0.0f);    // Face north
 *   Motor_SetTargetAngle(90.0f);   // Face east
 *   Motor_SetTargetAngle(-90.0f);  // Face west
 */
void Motor_SetTargetAngle(float target_angle);

/**
 * @brief Get current target angle
 * @return Target yaw angle (-180~180 degrees)
 */
float Motor_GetTargetAngle(void);
void Motor_UpdateYawAngle(float yaw_angle);
/**
 * @brief Get angle loop output value (for debugging)
 * @return Angle loop control output (RPS difference)
 */
float Motor_GetAngleControlOutput(void);

/**
 * @brief Set target position (position loop control) ⭐
 * @param target_pos Target position (centimeters, relative to position zero point)
 * @note Position loop automatically controls vehicle to advance to target position, dead zone 0.5cm
 * @note Use with Motor_ResetPosition(), clear position first then set target
 * @note Position based on average of two wheel encoders, tire diameter 75mm
 * @example
 *   Motor_ResetPosition();          // Clear position
 *   Motor_SetTargetPosition(50.0f); // Advance 50cm
 *   Motor_SetTargetPosition(-30.0f); // Backward 30cm (relative to zero point)
 */
void Motor_SetTargetPosition(float target_pos);

/**
 * @brief Get current accumulated position
 * @return Current position (centimeters)
 */
float Motor_GetActualPosition(void);

/**
 * @brief Get position loop output value (for debugging)
 * @return Position loop control output (RPS)
 */
float Motor_GetPositionControlOutput(void);

/**
 * @brief Get currently set target speeds
 * @param left_rps Output: Left wheel target speed (RPS)
 * @param right_rps Output: Right wheel target speed (RPS)
 */
void Motor_GetTargetSpeed(float *left_rps, float *right_rps);

/**
 * @brief Get current PWM output values
 * @param left_pwm Output: Left wheel PWM (-1.0 ~ 1.0)
 * @param right_pwm Output: Right wheel PWM (-1.0 ~ 1.0)
 * @note Used to monitor PID output PWM values
 */
void Motor_GetPWM(float *left_pwm, float *right_pwm);

/**
 * @brief Reset PID controller state (clear integral accumulation) ⭐
 * @note Call when motor switches direction or oscillation occurs to prevent integral saturation
 * @note Will clear position loop, angle loop, speed loop integral terms, error history and PWM output
 * @example
 *   Motor_SetTargetSpeed(2.0f, 2.0f);  // Forward
 *   HAL_Delay(2000);
 *   Motor_ResetPID();                  // Clear integral
 *   Motor_SetTargetSpeed(-2.0f, -2.0f); // Backward (avoid integral saturation)
 */
void Motor_ResetPID(void);

// ============================================================================
// Status Query Functions
// ============================================================================

/**
 * @brief Get actual speeds (filtered speed values)
 * @param left_rps Output parameter: Left wheel actual speed (unit: RPS)
 * @param right_rps Output parameter: Right wheel actual speed (unit: RPS)
 * @note This function will not interfere with PID control encoder reading
 * @example
 *   float left, right;
 *   Motor_GetActualSpeed(&left, &right);
 *   printf("Left: %.2f RPS, Right: %.2f RPS\n", left, right);
 */
void Motor_GetActualSpeed(float *left_rps, float *right_rps);

/**
 * @brief Current movement direction (global variable, for ODO data transmission)
 * @note 1=forward, -1=backward, 0=stop
 * @note Only valid in PID mode
 */
extern int8_t target_direction;


/**
 * @brief Reset position accumulation (clear odometer) ⭐
 * @note Used for position loop control, clear current accumulated position
 * @note Also clear target position and position loop PID state
 * @note Must be called before using Motor_SetTargetPosition()
 * @example
 *   Motor_ResetPosition();          // Clear position
 *   Motor_SetTargetPosition(50.0f); // Advance 50cm from current position
 */
void Motor_ResetPosition(void);

/**
 * @brief Set position calibration factor ⭐
 * @param factor Calibration factor (e.g.: 1.02 means actual distance is 1.02 times encoder calculated value)
 * @note Used to compensate for wheel diameter measurement error, improve position accuracy
 * @note If vehicle actual forward distance is less than target, increase factor; otherwise decrease factor
 * @note Factor range limited to 0.5~1.5
 * @example
 *   // Calibration method: target forward 100cm, actual only moved 98cm
 *   Motor_SetPositionCalibration(100.0f / 98.0f);  // = 1.02
 */
void Motor_SetPositionCalibration(float factor);

/**
 * @brief Get current position calibration factor
 * @return Calibration factor
 */
float Motor_GetPositionCalibration(void);

/**
 * @brief Set gyroscope angle calibration factor ⭐
 * @param factor Calibration factor (actual angle / gyroscope angle)
 * @note Used to compensate for gyroscope accumulated angle error
 * @note If gyroscope angle is too large, factor<1; if too small, factor>1
 * @note Factor range limited to 0.9~1.1
 * @example
 *   // Clockwise turn 3600 degrees, gyroscope shows 3616 degrees (too large)
 *   Motor_SetGyroCalibration(3600.0f / 3616.0f);  // = 0.99557522
 */
void Motor_SetGyroCalibration(float factor);

/**
 * @brief Get current gyroscope angle calibration factor
 * @return Calibration factor
 */
float Motor_GetGyroCalibration(void);

/**
 * @brief Gyroscope angle calibration helper (automatically calculate calibration factor) ⭐
 * @param actual_rotation_deg Actual rotation angle (degrees)
 * @param gyro_rotation_deg Gyroscope measured rotation angle (degrees)
 * @return Calculated calibration factor
 * @note Automatically calculate and set calibration factor
 * @note Recommend rotation angle >=360 degrees to improve accuracy
 * @example
 *   // Turn 10 circles (3600 degrees), gyroscope shows 3616 degrees
 *   float factor = Motor_CalibrateGyro(3600.0f, 3616.0f);
 *   printf("Calibration factor: %.8f\n", factor);  // Output: 0.99557522
 */
float Motor_CalibrateGyro(float actual_rotation_deg, float gyro_rotation_deg);

/**
 * @brief Get gyroscope raw angle (uncalibrated)
 * @return Raw yaw angle (degrees, -180~180)
 * @note Used for debugging, this is the gyroscope direct output normalized angle
 */
float Motor_GetRawYawAngle(void);

/**
 * @brief Get gyroscope calibrated angle
 * @return Calibrated yaw angle (degrees, -180~180)
 * @note This is the actual used angle value
 */
float Motor_GetCalibratedYawAngle(void);

/**
 * @brief Get accumulated raw angle (uncalibrated)
 * @return Accumulated rotation angle (degrees, can exceed ±180)
 * @note Used to view how many degrees gyroscope has accumulated rotation (uncalibrated)
 * @example
 *   // After turning 10 circles
 *   float acc = Motor_GetAccumulatedRawAngle();
 *   printf("Accumulated rotation: %.1f degrees\n", acc);  // May show 3616 degrees
 */
float Motor_GetAccumulatedRawAngle(void);

/**
 * @brief Get accumulated calibrated angle
 * @return Accumulated rotation angle (degrees, can exceed ±180, calibrated)
 * @note Used to view how many degrees accumulated rotation after calibration
 * @example
 *   // After turning 10 circles
 *   float acc = Motor_GetAccumulatedCalibratedAngle();
 *   printf("Calibrated accumulated: %.1f degrees\n", acc);  // Should show 3600 degrees
 */
float Motor_GetAccumulatedCalibratedAngle(void);

/**
 * @brief Reset accumulated angle tracking ⭐
 * @note Clear accumulated angle, but keep calibration factor
 * @note Call when recalibrating or need to clear accumulated angle
 * @example
 *   Motor_ResetAngleTracking();  // Clear accumulated angle
 */
void Motor_ResetAngleTracking(void);


// ============================================================================
// High-level Control Commands (Combining Position Loop and Angle Loop)
// ============================================================================

/**
 * @brief Command: Move forward specified distance (non-blocking) ⭐
 * @param distance_cm Forward distance (centimeters, positive=forward, negative=backward)
 * @param timeout_ms Timeout time (milliseconds, reserved parameter but not used)
 * @return 0=command set
 * @note Non-blocking execution, returns immediately after setting target
 * @note Need to continuously call Motor_PID_Control() in main loop to execute control
 * @note When target reached (position dead zone 0.5cm + angle dead zone 1.5 degrees), character 'Z' will be sent via UART4
 * @example
 *   // Move forward 60cm (non-blocking)
 *   Motor_Command_MoveForward(60.0f, 0);
 *   
 *   // Main loop
 *   while (1) {
 *       Motor_PID_Control(&htim1, &htim3, &htim2);
 *       // When 'Z' is received, target is reached
 *   }
 */
int Motor_Command_MoveForward(float distance_cm, uint32_t timeout_ms);

/**
 * @brief Command: Turn left in place specified angle (non-blocking) ⭐
 * @param angle_deg Rotation angle (degrees, positive=counterclockwise left turn, negative=clockwise right turn)
 * @param timeout_ms Timeout time (milliseconds, reserved parameter but not used)
 * @return 0=command set
 * @note Non-blocking execution, returns immediately after setting target
 * @note Need to continuously call Motor_PID_Control() in main loop to execute control
 * @note When target reached (angle dead zone 1.5 degrees), character 'Z' will be sent via UART4
 * @example
 *   // Turn left 90 degrees (non-blocking)
 *   Motor_Command_TurnLeft(90.0f, 0);
 *   
 *   // Main loop
 *   while (1) {
 *       Motor_PID_Control(&htim1, &htim3, &htim2);
 *       // When 'Z' is received, target is reached
 *   }
 */
int Motor_Command_TurnLeft(float angle_deg, uint32_t timeout_ms);

/**
 * @brief Command: Turn right in place specified angle (non-blocking) ⭐
 * @param angle_deg Rotation angle (degrees, positive=clockwise right turn, negative=counterclockwise left turn)
 * @param timeout_ms Timeout time (milliseconds, reserved parameter but not used)
 * @return 0=command set
 * @note Non-blocking execution, returns immediately after setting target
 * @note Need to continuously call Motor_PID_Control() in main loop to execute control
 * @note When target reached (angle dead zone 1.5 degrees), character 'Z' will be sent via UART4
 * @example
 *   // Turn right 90 degrees (non-blocking)
 *   Motor_Command_TurnRight(90.0f, 0);
 *   
 *   // Main loop
 *   while (1) {
 *       Motor_PID_Control(&htim1, &htim3, &htim2);
 *       // When 'Z' is received, target is reached
 *   }
 */
int Motor_Command_TurnRight(float angle_deg, uint32_t timeout_ms);

#endif /* __MOTOR_H */
