/**
 ******************************************************************************
 * @file    Encoder.c
 * @brief   Motor Encoder Interface Implementation
 * @author  Ye Jin
 * @date    2025
 ******************************************************************************
 */

#include "main.h"
#include "encoder.h"

// ============================================================================
// Encoder Initialization Functions
// ============================================================================

/**
 * @brief Encoder module initialization
 * @param htim_left Left wheel encoder timer handle (TIM3)
 * @param htim_right Right wheel encoder timer handle (TIM2)
 * @note Left wheel uses 4x frequency mode (A+B phases), right wheel uses 2x frequency mode (A-phase only)
 */
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right) {
    
    // ========== Configure Left Wheel Encoder (TIM3) - 4x Frequency Mode ==========
    TIM_Encoder_InitTypeDef sEncoderConfigLeft = {0};
    sEncoderConfigLeft.EncoderMode = TIM_ENCODERMODE_TI12;  // 4x frequency mode (A+B phases)
    sEncoderConfigLeft.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigLeft.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfigLeft.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigLeft.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    // Initialize left wheel encoder
    if (HAL_TIM_Encoder_Init(htim_left, &sEncoderConfigLeft) != HAL_OK) {
        Error_Handler();  // Initialization failed, enter error handler
    }

    // ========== Configure Right Wheel Encoder (TIM2) - 4x Frequency Mode ==========

    TIM_Encoder_InitTypeDef sEncoderConfigRight = {0};
    sEncoderConfigRight.EncoderMode = TIM_ENCODERMODE_TI12;  // 2x frequency mode (CH1 only)
    sEncoderConfigRight.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigRight.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfigRight.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigRight.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    // Initialize right wheel encoder
    if (HAL_TIM_Encoder_Init(htim_right, &sEncoderConfigRight) != HAL_OK) {
        Error_Handler();  // Initialization failed, enter error handler
    }

    // ========== Start Encoder Timers ==========

    // Start left wheel encoder (both channels)
    HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_1);  // Left wheel CH1 (A-phase)
    HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_2);  // Left wheel CH2 (B-phase)

    // Start right wheel encoder (only start CH1, avoid B-phase interference)
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_1); // Right wheel CH1 (A-phase)
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_2); // Right wheel CH1 (A-phase)
    // Right wheel CH2 not started because B-phase signal is unstable

    // ========== Clear Encoder Counters ==========
    __HAL_TIM_SET_COUNTER(htim_left, 0);
    __HAL_TIM_SET_COUNTER(htim_right, 0);
}

// ============================================================================
// Encoder Speed Reading Functions
// ============================================================================

/**
 * @brief Get rotation speeds of two motors (RPS)
 * @param htim_left Left wheel encoder timer handle (TIM3)
 * @param htim_right Right wheel encoder timer handle (TIM2)
 * @param rps_left Output parameter: Left wheel speed (unit: revolutions per second)
 * @param rps_right Output parameter: Right wheel speed (unit: revolutions per second)
 * @param dt Time interval (unit: seconds)
 * @note Counter is immediately cleared after reading, preparing for next measurement
 * @note Both wheel speeds are positive when moving forward, negative when moving backward
 */
void Encoder_GetSpeeds(TIM_HandleTypeDef* htim_left, TIM_HandleTypeDef* htim_right, 
                      float* rps_left, float* rps_right, float dt)
{
    // ========== Step 1: Read Encoder Count Values ==========
    // Use int16_t type to support positive/negative counting (forward/backward)
    int16_t cnt_left = (int16_t)__HAL_TIM_GET_COUNTER(htim_left);
    int16_t cnt_right = (int16_t)__HAL_TIM_GET_COUNTER(htim_right);
    
    // ========== Step 2: Immediately Clear Counters (prepare for next measurement) ==========
    __HAL_TIM_SET_COUNTER(htim_left, 0);
    __HAL_TIM_SET_COUNTER(htim_right, 0);
    
    // ========== Step 3: Time Interval Protection (prevent division by zero) ==========
    if(dt < 0.001f) {
        dt = 0.001f;  // Minimum time interval limited to 1ms
    }
    
    // ========== Step 4: Calculate Speed (RPS = count value / PPR / time interval) ==========
    
    // Left wheel speed calculation (no inversion)
    // Measured: TIM3 counts positive when moving forward, direction correct, no inversion needed
    *rps_left = (float)cnt_left / ENCODER_PPR_LEFT / dt;
    
    // Right wheel speed calculation (inverted)
    // Measured: TIM2 counts negative when moving forward, needs inversion to match left wheel direction
    // Goal: Both wheel speeds are positive when moving forward
    *rps_right = -(float)cnt_right / ENCODER_PPR_RIGHT / dt;
    
    /**
     * Speed calculation explanation:
     * - Encoder generates PPR pulses per revolution
     * - In dt time, count value is cnt
     * - Speed (revolutions/second) = count value / pulses per revolution / time interval
     * 
     * Left wheel example (4x frequency, PPR=1560):
     *   If dt=0.02 seconds (20ms), cnt_left=31
     *   Then rps_left = 31 / 1560 / 0.02 ≈ 0.99 RPS
     * 
     * Right wheel example (2x frequency, PPR=780):
     *   If dt=0.02 seconds (20ms), cnt_right=-16 (negative)
     *   Then rps_right = -(-16) / 780 / 0.02 ≈ 1.03 RPS
     * 
     * Notes:
     * - Left/right wheel PPR differ because right wheel only uses single phase (2x frequency)
     * - Right wheel inversion is due to encoder mounting direction or wiring causing opposite counting direction
     * - Final result: Both wheel speeds are positive when moving forward, convenient for PID control
     */
}
