/**
 ******************************************************************************
 * @file           : radar.h
 * @brief          : LiDAR module header file (continuous 360-degree point cloud scanning solution)
 ******************************************************************************
 * @attention
 * 
 * 🔄 **Refactoring Notes** (2025-10-20)
 * - Removed three-zone obstacle avoidance processing (handled by upper computer)
 * - Implemented continuous 360-degree point cloud data acquisition
 * - Using binary frame format, efficient transmission of 500 points/circle data
 * - Angular resolution: 0.72° (500 points/circle)
 * - Communication rate: 460800 baud rate, 10Hz update frequency
 * 
 ******************************************************************************
 */

#ifndef __RADAR_H__
#define __RADAR_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Exported constants --------------------------------------------------------*/

/**
 * @brief Radar command definitions
 */
#define RADAR_CMD_START_SCAN    {0xA5, 0x20}  ///< Start scan command (2 bytes)
#define RADAR_CMD_STOP_SCAN     {0xA5, 0x25}  ///< Stop scan command (2 bytes)


/* Required macros (adjust to your project values) */
#define LIDAR_RX_DMA_BUF_SZ      512   // DMA receive buffer size (DMA writes each time)
#define LIDAR_PEND_RING_SZ       2048  // Ring buffer (must accommodate multiple DMA blocks)
#define LIDAR_FRAME_LEN          5
#define LIDAR_MAX_POINTS         500
#define LIDAR_MIN_POINTS_PER_ROTATION 100 // Minimum point threshold to avoid false triggering (adjustable)

typedef struct {
    uint16_t angle_q6;    // angle in q6 (angle_q6/64.0 = degrees)
    uint16_t distance_q2; // distance in q2 (distance_q2/4.0 = mm)
    uint8_t  quality;     // quality (raw)
    uint32_t timestamp;   // HAL_GetTick() when parsed
} LidarPoint_t;

// PackPoint_t has same structure as LidarPoint_t, used for packing and sending
typedef LidarPoint_t PackPoint_t;

/* Initialize/start receiving (caller calls after UART and DMA are configured) */
void LIDAR_Start(UART_HandleTypeDef *huart);

/* Called in USART interrupt (IRQHandler) to handle IDLE interrupt */
void LIDAR_UART_IdleCallback(UART_HandleTypeDef *huart);

/* Called from interrupt context or main loop: process DMA received data block */
void LIDAR_ProcessDMAData(UART_HandleTypeDef *huart, uint16_t len);

/* For upper layer to read completed rotation data (returns point count) */
uint16_t LIDAR_GetCompletedRotation(LidarPoint_t *out_buffer, uint16_t max_count);

/* Reset rotation completion flag (or Get will automatically reset) */
uint8_t LIDAR_IsRotationReady(void);

/**
 * @brief Initialize radar continuous scan mode
 * @param huart_radar Radar communication UART handle (UART3)
 * @retval 0=success, 1=failure
 */
uint8_t Radar_Init(UART_HandleTypeDef *huart_radar);

/**
 * @brief Start radar scanning
 * @param huart_radar Radar communication UART handle
 * @retval HAL_StatusTypeDef HAL status
 */
HAL_StatusTypeDef Radar_StartScan(UART_HandleTypeDef *huart_radar);

/**
 * @brief Stop radar scanning
 * @param huart_radar Radar communication UART handle
 * @retval HAL_StatusTypeDef HAL status
 */
HAL_StatusTypeDef Radar_StopScan(UART_HandleTypeDef *huart_radar);



/**
 * @brief IDLE interrupt callback (fast path: only copy data to ring buffer)
 * @param huart UART handle
 * @note Called in USART3_IRQHandler in stm32f4xx_it.c
 */
void LIDAR_UART_IdleCallback_IRQ(UART_HandleTypeDef *huart);

/**
 * @brief Main loop processing function: parse radar data in ring buffer
 * @note Periodically called in main loop or timer task
 */
void LIDAR_ProcessPendingData(void);

/**
 * @brief Pack and send radar data frame via UART4 DMA (1012 bytes)
 * @param points Point cloud data array
 * @param n_points Point count
 * @retval 0=successfully started transmission, -1=UART busy, -2=parameter error
 */
int LIDAR_PackAndSendFrame(const PackPoint_t *points, uint16_t n_points);

/**
 * @brief UART4 DMA transmission complete callback (called from HAL_UART_TxCpltCallback)
 * @note This function should be called in HAL_UART_TxCpltCallback
 */
void LIDAR_TxCpltCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* __RADAR_H__ */
