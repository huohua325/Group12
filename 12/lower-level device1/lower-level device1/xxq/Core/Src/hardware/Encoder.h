/**
 ******************************************************************************
 * @file    Encoder.h
 * @brief   Motor encoder interface module (for speed measurement and position feedback)
 * @author  Ye Jin
 * @date    2025
 ******************************************************************************
 * @description
 * This module provides motor encoder initialization and speed reading functions.
 * 
 * Hardware configuration:
 * - Left wheel encoder: TIM3 (PC6/PC7), quadruple frequency mode, PPR=1560
 * - Right wheel encoder: TIM2 (PA15/PB3), double frequency mode, PPR=780 (B phase unstable, only A phase used)
 * 
 * Motor parameters (MC520P30):
 * - Encoder lines: 13 lines
 * - Reduction ratio: 1:30
 * - Output shaft pulses per revolution: 13 × 30 = 390
 * 
 * Usage:
 * 1. Call Encoder_Init() to initialize encoder timer
 * 2. Periodically call Encoder_GetSpeeds() to get wheel speeds
 * 3. Speed unit is RPS (revolutions per second), positive when moving forward
 ******************************************************************************
 */

#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"
#include "stm32f4xx_hal.h"

// ============================================================================
// Encoder parameter definitions (MC520P30 motor)
// ============================================================================

// ---------- Basic parameters ----------
#define ENCODER_LINES       390    // Encoder lines (output shaft pulses per revolution = 13 lines × 30 reduction ratio)
#define ENCODER_MULTIPLIER  4      // Quadruple frequency mode multiplier

// ---------- Total pulses per revolution (Pulses Per Revolution) ----------
#define ENCODER_PPR (ENCODER_LINES * ENCODER_MULTIPLIER)  // Standard quadruple frequency: 390 × 4 = 1560

// ---------- Left/right wheel PPR settings (compensating for hardware differences) ----------
#define ENCODER_PPR_LEFT   ENCODER_PPR        // Left wheel: 1560 (quadruple frequency, A+B phases normal)
#define ENCODER_PPR_RIGHT  ENCODER_PPR  // Right wheel: 780 (double frequency, A phase only)

/**
 * @note Right wheel encoder description:
 * Due to unstable B phase signal of the right wheel encoder (possible poor contact or electrical interference),
 * TIM_ENCODERMODE_TI1 mode is used (only CH1/A phase), which has double frequency characteristics in practice,
 * therefore PPR is set to half the standard value.
 */

// ============================================================================
// Hardware pin definitions (actual physical connections)
// ============================================================================

// ---------- Right wheel encoder (TIM2) ----------
#define RIGHT_ENCODER_TIM      TIM2
#define RIGHT_ENCODER_CH_A     TIM_CHANNEL_1  // PA15 → Right wheel A phase (main counting channel)
#define RIGHT_ENCODER_CH_B     TIM_CHANNEL_2  // PB3  → Right wheel B phase (unused, unstable)

// ---------- Left wheel encoder (TIM3) ----------
#define LEFT_ENCODER_TIM       TIM3
#define LEFT_ENCODER_CH_A      TIM_CHANNEL_1  // PC6 → Left wheel A phase
#define LEFT_ENCODER_CH_B      TIM_CHANNEL_2  // PC7 → Left wheel B phase

// ============================================================================
// Encoder interface functions
// ============================================================================

/**
 * @brief Encoder module initialization
 * @param htim_left Left wheel encoder timer handle (TIM3)
 * @param htim_right Right wheel encoder timer handle (TIM2)
 * @note Configure encoder mode (left wheel quadruple frequency, right wheel double frequency) and start timer
 * @note Counter is cleared after initialization, ready to start counting
 * @note Must be called once before using encoder to read speed
 * @warning Right wheel only starts CH1 channel, CH2 is not started (avoid B phase interference)
 */
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right);

/**
 * @brief Get the speed of two motors
 * @param htim_left Left wheel encoder timer handle (TIM3)
 * @param htim_right Right wheel encoder timer handle (TIM2)
 * @param rps_left Output parameter: left wheel speed (unit: RPS, revolutions per second)
 * @param rps_right Output parameter: right wheel speed (unit: RPS, revolutions per second)
 * @param dt Time interval (unit: seconds)
 * @note Read encoder count, calculate speed and immediately clear counter
 * @note Both wheel speeds are positive when moving forward, negative when moving backward
 * @note Speed calculation formula: RPS = count value / PPR / time interval
 * @note Right wheel speed is automatically inverted to compensate for encoder direction difference
 * @warning dt cannot be 0, function internally limits minimum value to 0.001 seconds
 * @example
 *   float left_speed, right_speed;
 *   Encoder_GetSpeeds(&htim3, &htim2, &left_speed, &right_speed, 0.02f);
 *   printf("Left: %.2f RPS, Right: %.2f RPS\n", left_speed, right_speed);
 */
void Encoder_GetSpeeds(TIM_HandleTypeDef *htim_left, 
                       TIM_HandleTypeDef *htim_right, 
                       float *rps_left, 
                       float *rps_right, 
                       float dt);

#endif /* __ENCODER_H */
