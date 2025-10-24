/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif



/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define cs_Pin GPIO_PIN_4
#define cs_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */

/* ============================================================================ */
/*                             SYSTEM DEFINITIONS                              */
/* ============================================================================ */

/**
 * @brief System state enumeration
 */
typedef enum {
    SYSTEM_STATE_INIT = 0,      ///< System initializing
    SYSTEM_STATE_READY,         ///< System ready but not running
    SYSTEM_STATE_RUNNING,       ///< System running normally
    SYSTEM_STATE_ERROR          ///< System in error state
} SystemState_t;

/**
 * @brief Motor control mode enumeration
 */
typedef enum {
    MOTOR_MODE_STOPPED = 0,     ///< Motors stopped
    MOTOR_MODE_MANUAL,          ///< Manual PWM control
    MOTOR_MODE_PID_CONTROL      ///< PID feedback control
} MotorControlMode_t;

/**
 * @brief Legacy car state enumeration (keep for compatibility)
 */
typedef enum {
    STOP,
    FORWARD,
    BACKWARD,
    TURN_LEFT,
    TURN_RIGHT
} CarState;

/* ============================================================================ */
/*                             SYSTEM CONSTANTS                                */
/* ============================================================================ */

/**
 * @brief Speed control parameters
 */
#define DEFAULT_TARGET_SPEED    1.5f        ///< Default target speed (RPS)
#define MAX_TARGET_SPEED        5.0f        ///< Maximum target speed (RPS)
#define MIN_TARGET_SPEED        0.5f        ///< Minimum target speed (RPS)
#define SPEED_ADJUST_STEP       0.5f        ///< Speed adjustment step (RPS)

/**
 * @brief Communication parameters
 */
#define UART_TIMEOUT_MS         10          ///< UART receive timeout (ms)

/* ============================================================================ */
/*                             EXTERNAL DECLARATIONS                           */
/* ============================================================================ */

extern CarState currentState;               ///< Legacy car state variable
extern TIM_HandleTypeDef htim1;             ///< PWM timer handle
extern uint16_t mpu6500_address;            ///< MPU6500 I2C address

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
