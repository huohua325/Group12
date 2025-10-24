/**
 ******************************************************************************
 * @file           : radar.c
 * @brief          : LiDAR Module Implementation (Continuous 360° Point Cloud Solution)
 ******************************************************************************
 * @attention
 * 
 * 🔄 **Refactoring Notes** (2025-10-20)
 * - Removed three-zone obstacle avoidance processing (handled by host computer)
 * - Implemented continuous 360° point cloud data acquisition
 * - Binary frame transmission: 1012 bytes/frame × 10Hz = 10.1KB/s
 * - Baud rate 460800, bandwidth utilization 34%, very abundant
 * 
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "radar.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
/* Private define ------------------------------------------------------------*/



/* ============================================================================
 *  🔧 Core Function Implementation
 * ========================================================================== */

/**
 * @brief Initialize radar continuous scanning mode (DMA optimized version)
 */
uint8_t Radar_Init(UART_HandleTypeDef *huart_radar)
{
    if (huart_radar == NULL) {
        return 1;  // Parameter error
    }
    
    // Stop radar (ensure in known state)
    Radar_StopScan(huart_radar);
    HAL_Delay(100);
    // Initialize global buffer
    // Start radar scanning
    HAL_StatusTypeDef ret = Radar_StartScan(huart_radar);

    LIDAR_Start(huart_radar);
    //ret = HAL_UART_Receive_DMA(huart_radar, (uint8_t*)g_radar_dma_buffer, RADAR_DMA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(huart_radar, UART_IT_IDLE);
    return 0;  // Success
}

/**
 * @brief Start radar scanning
 */
HAL_StatusTypeDef Radar_StartScan(UART_HandleTypeDef *huart_radar)
{
    uint8_t start_cmd[] = RADAR_CMD_START_SCAN;
    return HAL_UART_Transmit(huart_radar, start_cmd, sizeof(start_cmd), 100);
}

/**
 * @brief Stop radar scanning
 */
HAL_StatusTypeDef Radar_StopScan(UART_HandleTypeDef *huart_radar)
{
    uint8_t stop_cmd[] = RADAR_CMD_STOP_SCAN;
    return HAL_UART_Transmit(huart_radar, stop_cmd, sizeof(stop_cmd), 100);
}


/**
 * @brief ⭐ UART3 IDLE interrupt callback function (DMA receive core)
 * @note Called in USART3_IRQHandler in stm32f4xx_it.c
 */
void Radar_UART_IdleCallback(UART_HandleTypeDef *huart_radar)
{
    
    if (huart_radar == NULL || huart_radar->hdmarx == NULL) {
        return;
    }
    LIDAR_UART_IdleCallback(&huart3);
    // Clear IDLE interrupt flag
    __HAL_UART_CLEAR_IDLEFLAG(huart_radar);
}

/**

/**
 * @brief Calculate CRC16 checksum (CRC-16/MODBUS)
 * @param data Data buffer
 * @param length Data length
 * @return CRC16 checksum value
 */
static uint16_t Radar_CalculateCRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;  // Initial value

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // Polynomial
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}



/* External UART handles */
extern UART_HandleTypeDef huart3; // Radar receive (UART3)
extern UART_HandleTypeDef huart4; // Host computer send (UART4)

/* ============================================================================
 *  DMA Receive + Ring Buffer (ISR Fast Path)
 * ========================================================================== */

/* DMA receive buffer (associated with HAL_UART_Receive_DMA) */
static uint8_t rx_dma_buf[LIDAR_RX_DMA_BUF_SZ];

/* Ring processing buffer (for quickly copying DMA data to main loop parsing) */
static uint8_t proc_ring[LIDAR_PEND_RING_SZ];
static volatile uint16_t proc_head = 0; // Write index (ISR)
static volatile uint16_t proc_tail = 0; // Read index (main loop)

/* UART4 DMA send status flag */
static volatile uint8_t uart4_tx_busy = 0;

/* Final point storage area (written by parsing completion) */
static volatile LidarPoint_t lidar_points[LIDAR_MAX_POINTS];
static volatile uint16_t lidar_point_index = 0;

/* Completed rotation backup area and flag */
static volatile LidarPoint_t completed_rotation[LIDAR_MAX_POINTS];
static volatile uint16_t completed_count = 0;
static volatile uint8_t rotation_ready = 0;

/* Parsing state variables */
static uint16_t last_angle_q6 = 0xFFFF; // Previous angle (initialization flag)
static uint32_t last_rotation_tick = 0;
static uint32_t point_counter_total = 0;
static uint32_t last_rotation_point_counter = 0;

/* State machine: Wait for start response → Receive data */
typedef enum {
    WAIT_START_RESP,
    RECEIVING_DATA
} ParserState_t;

static ParserState_t parser_state = WAIT_START_RESP;
static uint8_t frame_len = 0;
static uint8_t data_type = 0;

/* Data quality statistics (for debugging) */
static uint32_t total_frames_received = 0;
static uint32_t valid_frames_count = 0;
static uint32_t sync_errors = 0;
static uint32_t quality_filtered = 0;
static uint32_t range_filtered = 0;

/* Internal function declarations */
static void try_emit_rotation_by_angle_wrap(uint16_t angle_q6);

/* ============================================================================
 *  IDLE Interrupt Fast Path (Only Copy Data, No Parsing)
 * ========================================================================== */

/**
 * @brief IDLE interrupt callback - fast path (very short, only copy)
 * @note Called in interrupt context, must return as soon as possible
 */
void LIDAR_UART_IdleCallback_IRQ(UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx) return;

    /* Clear IDLE flag: read SR/DR */
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    /* Stop DMA, get received byte count */
    HAL_UART_DMAStop(huart);
    uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(huart->hdmarx); // Remaining count
    uint16_t recvd = (uint16_t)(LIDAR_RX_DMA_BUF_SZ - dma_cnt);

    if (recvd > 0) {
        /* Fast copy to ring buffer (avoid stack overflow) */
        uint16_t write_idx = proc_head;
        for (uint16_t i = 0; i < recvd; ++i) {
            proc_ring[write_idx++] = rx_dma_buf[i];
            if (write_idx >= LIDAR_PEND_RING_SZ) write_idx = 0;
        }
        /* Update write index (atomic operation, interrupt context) */
        proc_head = write_idx;
    }

    /* Restart DMA receive (as soon as possible) */
    HAL_UART_Receive_DMA(huart, rx_dma_buf, LIDAR_RX_DMA_BUF_SZ);
}

/* Start DMA receive */
void LIDAR_Start(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(huart, (uint8_t *)rx_dma_buf, LIDAR_RX_DMA_BUF_SZ);
}

/* ============================================================================
 *  Main Loop Processing Function (Sliding Window + State Machine Parsing)
 * ========================================================================== */

/**
 * @brief Main loop processing function: parse radar data in ring buffer
 * @note Called periodically in main loop, performs sliding window frame sync and frame-by-frame verification
 */
void LIDAR_ProcessPendingData(void)
{
    /* Quickly read index (avoid being modified by ISR in the middle) */
    uint16_t tail = proc_tail;
    uint16_t head = proc_head;

    if (tail == head) return; // No data

    /* Temporary linear buffer to reduce per-byte read overhead */
    static uint8_t temp_buf[1024];
    uint16_t temp_len = 0;

    /* Pull ring buffer data into linear block at once */
    while (tail != head && temp_len < sizeof(temp_buf)) {
        temp_buf[temp_len++] = proc_ring[tail++];
        if (tail >= LIDAR_PEND_RING_SZ) tail = 0;
    }
    proc_tail = tail; // Update global tail

    /* Concatenate temp_buf to parsing buffer */
    static uint8_t parse_buf[1024];
    static uint16_t parse_len = 0;

    /* Prevent overflow */
    if (parse_len + temp_len > sizeof(parse_buf)) {
        parse_len = 0; // Discard old data, restart
    }
    memcpy(parse_buf + parse_len, temp_buf, temp_len);
    parse_len += temp_len;

    /* Sliding window parsing (try byte by byte) */
    uint16_t pos = 0;

    while (parse_len - pos >= 2) {
        if (parser_state == WAIT_START_RESP) {
            /* Look for 0xA5 0x5A start response */
            if (parse_buf[pos] == 0xA5 && parse_buf[pos+1] == 0x5A) {
                /* Ensure start response is complete (7 bytes) */
                if (parse_len - pos >= 7) {
                    /* Parse DataLen & DataType
                     * Manual format: A5 5A [len_low] [len_mid] [len_high] [mode] [dtype]
                     */
                    uint32_t data_len_raw = ((uint32_t)parse_buf[pos+2]) |
                                            ((uint32_t)parse_buf[pos+3] << 8) |
                                            ((uint32_t)parse_buf[pos+4] << 16);
                    uint8_t dtype = parse_buf[pos+6];

                    /* SCAN mode: dtype=0x81, data_len=5 */
                    if (data_len_raw > 0 && dtype == 0x81) {
                        frame_len = (uint8_t)data_len_raw;
                        parser_state = RECEIVING_DATA;
                        data_type = dtype;
                        pos += 7; // Consume start response
                        continue;
                    } else {
                        pos++; // Skip 1 byte and continue searching
                    }
                } else {
                    break; // Wait for more bytes
                }
            } else {
                pos++; // Header not found
            }
        } else { /* RECEIVING_DATA */
            if (frame_len == 0 || frame_len != 5) {
                parser_state = WAIT_START_RESP;
                break;
            }

            /* Ensure complete frame */
            if (parse_len - pos < frame_len) break;

            total_frames_received++;

            /* Extract bytes */
            uint8_t *f = &parse_buf[pos];
            uint8_t b0 = f[0];
            uint8_t quality = b0 >> 2;
            uint8_t s = b0 & 0x1;
            uint8_t s_inv = (b0 & 0x2) >> 1;
            uint16_t angle_q6 = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
            uint16_t distance_q2 = (uint16_t)f[3] | ((uint16_t)f[4] << 8);

            /* Data validity check (combine multiple factors) */
            uint8_t valid = 1;

            /* S/Sbar consistency (optional, not mandatory) */
            if (s == s_inv) {
                sync_errors++;
                // Not mandatory to discard, only statistics
            }

            /* Quality check (configurable threshold) */
            if (quality < 5) {
                valid = 0;
                quality_filtered++;
            }

            /* Angle range (23040 corresponds to 360°, allow small error) */
            if (angle_q6 > 23500) {
                valid = 0;
                range_filtered++;
            }

            /* Distance range (Q2 format -> mm) */
            uint32_t dist_mm = distance_q2 >> 2;
            if (dist_mm == 0 || dist_mm < 30 || dist_mm > 16000) {
                valid = 0;
                range_filtered++;
            }

            if (valid) {
                /* Store valid point */
                if (lidar_point_index < LIDAR_MAX_POINTS) {
                    LidarPoint_t pt;
                    pt.angle_q6 = angle_q6;
                    pt.distance_q2 = distance_q2;
                    pt.quality = quality;
                    pt.timestamp = HAL_GetTick();

                    lidar_points[lidar_point_index++] = pt;
                    point_counter_total++;
                }

                /* Try to trigger rotation completion */
                try_emit_rotation_by_angle_wrap(angle_q6);
                valid_frames_count++;

                pos += frame_len; // Consume frame
            } else {
                /* Invalid frame: slide 1 byte and continue searching for sync point */
                pos += 1;
            }
        }
    } /* end while parsing */

    /* Keep unconsumed bytes for next processing */
    if (pos < parse_len) {
        uint16_t remain = parse_len - pos;
        memmove(parse_buf, parse_buf + pos, remain);
        parse_len = remain;
    } else {
        parse_len = 0;
    }
}

/**
 * @brief Angle wrap detection triggers rotation completion (optimized version)
 * @note Uses more reasonable angle threshold + time protection
 */
static void try_emit_rotation_by_angle_wrap(uint16_t angle_q6)
{
    /* First initialization */
    if (last_angle_q6 == 0xFFFF) {
        last_angle_q6 = angle_q6;
        last_rotation_tick = HAL_GetTick();
        return;
    }

    uint32_t points_since_last = point_counter_total - last_rotation_point_counter;

    /* Angle wrap detection (Q6 format: 0-23040 corresponds to 0-360°)
     * Optimized parameters: wrap from near 360° to near 0°
     */
    const uint16_t wrap_high_q6 = 22000;  // ~343°
    const uint16_t wrap_low_q6  = 3000;   // ~47°

    /* Condition 1: Angle wrap + sufficient points */
    if ((last_angle_q6 > wrap_high_q6) &&
        (angle_q6 < wrap_low_q6) &&
        (points_since_last >= LIDAR_MIN_POINTS_PER_ROTATION)) {

        /* Atomic copy to completion buffer */
        __disable_irq();
        uint16_t ncopy = (lidar_point_index <= LIDAR_MAX_POINTS) ? lidar_point_index : LIDAR_MAX_POINTS;
        for (uint16_t i = 0; i < ncopy; ++i) {
            completed_rotation[i] = lidar_points[i];
        }
        completed_count = ncopy;
        rotation_ready = 1;

        /* Reset next rotation */
        lidar_point_index = 0;
        last_rotation_point_counter = point_counter_total;
        last_rotation_tick = HAL_GetTick();
        __enable_irq();
    }

    /* Condition 2: Time protection (1 second timeout) */
    uint32_t now = HAL_GetTick();
    if ((now - last_rotation_tick) >= 1000 &&
        (points_since_last >= LIDAR_MIN_POINTS_PER_ROTATION)) {

        __disable_irq();
        uint16_t ncopy = (lidar_point_index <= LIDAR_MAX_POINTS) ? lidar_point_index : LIDAR_MAX_POINTS;
        for (uint16_t i = 0; i < ncopy; ++i) {
            completed_rotation[i] = lidar_points[i];
        }
        completed_count = ncopy;
        rotation_ready = 1;

        lidar_point_index = 0;
        last_rotation_point_counter = point_counter_total;
        last_rotation_tick = now;
        __enable_irq();
    }

    last_angle_q6 = angle_q6;
}

/* Upper layer call to get completed rotation data; returns point count (returns 0 if not ready) */
uint16_t LIDAR_GetCompletedRotation(LidarPoint_t *out_buffer, uint16_t max_count)
{
    uint16_t ret = 0;
    if (!rotation_ready) return 0;

    __disable_irq();
    uint16_t n = (completed_count <= max_count) ? completed_count : max_count;
    for (uint16_t i = 0; i < n; ++i) {
        out_buffer[i] = completed_rotation[i];
    }
    /* Reset ready flag */
    rotation_ready = 0;
    completed_count = 0;
    __enable_irq();

    ret = n;
    return ret;
}

uint8_t LIDAR_IsRotationReady(void)
{
    return rotation_ready;
}

/* ============================================================================
 *  Data Packing and Host Computer Transmission (1012-byte Binary Frame)
 * ========================================================================== */

#define LIDAR_PACKET_SIZE 1012U
#define LIDAR_POINT_COUNT 500U
#define ANGLE_PER_INDEX 0.72f    // 度/索引

/* helper - write little-endian */
static void write_u16_le(uint8_t *dst, uint16_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void write_u32_le(uint8_t *dst, uint32_t v)
{
    dst[0] = (uint8_t)(v & 0xFF);
    dst[1] = (uint8_t)((v >> 8) & 0xFF);
    dst[2] = (uint8_t)((v >> 16) & 0xFF);
    dst[3] = (uint8_t)((v >> 24) & 0xFF);
}

/**
 * @brief Pack and send radar data frame via UART4 DMA (1012 bytes)
 * @param points Point cloud data array (PackPoint_t = LidarPoint_t)
 * @param n_points Point count
 * @retval 0=successfully started transmission, -1=UART busy, -2=parameter error
 */
int LIDAR_PackAndSendFrame(const PackPoint_t *points, uint16_t n_points)
{
    if (points == NULL || n_points == 0) return -2;

    /* Check UART4 idle */
    if (uart4_tx_busy != 0) {
        return -1;  // Previous transmission not completed
    }

    static uint8_t txbuf[LIDAR_PACKET_SIZE];
    memset(txbuf, 0, sizeof(txbuf));

    /* 1) Frame header 4 bytes */
    txbuf[0] = 0xA5;
    txbuf[1] = 0x5A;
    txbuf[2] = 0x4C;
    txbuf[3] = 0x44;

    /* 2) Timestamp (uint32_t, LE) */
    uint32_t now = HAL_GetTick();
    write_u32_le(&txbuf[4], now);

    /* 3) Point count (uint16_t LE) fixed 500 */
    write_u16_le(&txbuf[8], (uint16_t)LIDAR_POINT_COUNT);

    /* 4) Initialize distance array to 0 (indicating invalid) */
    uint16_t distances[LIDAR_POINT_COUNT];
    for (uint32_t i = 0; i < LIDAR_POINT_COUNT; ++i) distances[i] = 0;

    /* 5) Traverse input points, map to index and keep minimum non-zero distance */
    for (uint32_t i = 0; i < n_points; ++i) {
        uint16_t a_q6 = points[i].angle_q6;
        uint16_t d_q2 = points[i].distance_q2;
        if (d_q2 == 0) continue; // Skip invalid points

        /* Angle (degrees) */
        float angle_deg = ((float)a_q6) / 64.0f;

        /* Calculate index: round(angle / ANGLE_PER_INDEX) mod 500 */
        float idxf = angle_deg / ANGLE_PER_INDEX;
        uint32_t idx = (uint32_t)(roundf(idxf)) % LIDAR_POINT_COUNT;

        /* Convert q2 -> mm (integer) */
        uint16_t dist_mm = (uint16_t)(d_q2 / 4);

        /* If slot is empty (0) or new point is closer (smaller value) then replace */
        if (distances[idx] == 0 || (dist_mm > 0 && dist_mm < distances[idx])) {
            distances[idx] = dist_mm;
        }
    }

    /* 6) Write distances to txbuf[10..1009], little-endian uint16_t */
    uint32_t p = 10;
    for (uint32_t i = 0; i < LIDAR_POINT_COUNT; ++i) {
        txbuf[p++] = (uint8_t)(distances[i] & 0xFF);
        txbuf[p++] = (uint8_t)((distances[i] >> 8) & 0xFF);
    }
    /* At this point p should be 1010 */

    /* 7) Calculate CRC-16/MODBUS (calculate for bytes 0..1009) */
    uint16_t crc = Radar_CalculateCRC16(txbuf, 1010);
    write_u16_le(&txbuf[1010], crc);

    /* 8) Trigger UART4 DMA transmission */
    uart4_tx_busy = 1;  // Mark transmission start
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart4, txbuf, LIDAR_PACKET_SIZE);
    if (st == HAL_OK) {
        return 0;
    } else {
        uart4_tx_busy = 0;  // Transmission start failed, clear flag
        return -1;
    }
}

/**
 * @brief UART4 DMA transmission completion callback
 * @note Called in HAL_UART_TxCpltCallback
 */
void LIDAR_TxCpltCallback(void)
{
    uart4_tx_busy = 0;  // Mark transmission completion
}

/* ============================================================================
 *  Raw Data Pass-through Solution (New - 2025-10-21)
 * ========================================================================== */

/* Frame rate control */
static uint32_t last_send_time = 0;   // Last send time
static uint8_t frame_sequence = 0;    // Frame sequence number (0-255 cycle)

/* Frame format constants */
#define RAW_FRAME_HEADER_SIZE 8
#define RAW_FRAME_TRAILER_SIZE 8
#define RAW_FRAME_META_SIZE 8  // 帧类型1 + 序号1 + 时间戳4 + 点数2
#define RAW_POINT_SIZE 5       // 每个点5字节
#define RAW_FRAME_OVERHEAD (RAW_FRAME_HEADER_SIZE + RAW_FRAME_META_SIZE + 2 + RAW_FRAME_TRAILER_SIZE)  // 26字节

/* Frame header/trailer definitions */
static const uint8_t RAW_FRAME_HEADER[8] = {0xAA, 0x55, 0xAA, 0x55, 0xF0, 0x0F, 0xF0, 0x0F};
static const uint8_t RAW_FRAME_TRAILER[8] = {0x55, 0xAA, 0x55, 0xAA, 0x0F, 0xF0, 0x0F, 0xF0};

/* Frame type definitions */
#define FRAME_TYPE_LIDAR_RAW 0x01   // LiDAR raw data frame

/**
 * @brief Check if new radar frame should be sent (frame rate control)
 * @retval 1=should send, 0=not yet time to send
 */
uint8_t LIDAR_ShouldSendFrame(void)
{
    uint32_t now = HAL_GetTick();
    
    // Send every 500ms (2Hz)
    if (now - last_send_time >= 500) {
        last_send_time = now;
        return 1;
    }
    
    return 0;
}

/**
 * @brief Pack radar raw 5-byte data and send
 * @param points Point cloud data array
 * @param n_points Point count
 * @retval 0=success, -1=UART busy, -2=parameter error, -3=zero points
 */
int LIDAR_PackAndSendRawFrame(const PackPoint_t *points, uint16_t n_points)
{
    if (points == NULL || n_points == 0) {
        return -3;
    }
    
    // Check UART4 idle
    if (uart4_tx_busy != 0) {
        return -1;
    }
    
    // Static buffer (maximum support 700 points: 26 + 700×5 = 3526 bytes)
    static uint8_t txbuf[4096];
    uint16_t offset = 0;
    
    // 1. Frame header (8 bytes)
    memcpy(&txbuf[offset], RAW_FRAME_HEADER, RAW_FRAME_HEADER_SIZE);
    offset += RAW_FRAME_HEADER_SIZE;
    
    // 2. Frame type (1 byte)
    txbuf[offset++] = FRAME_TYPE_LIDAR_RAW;
    
    // 3. Frame sequence (1 byte, cycle 0-255)
    txbuf[offset++] = frame_sequence++;
    
    // 4. Timestamp (4 bytes, little-endian)
    uint32_t timestamp = HAL_GetTick();
    write_u32_le(&txbuf[offset], timestamp);
    offset += 4;
    
    // 5. Point count (2 bytes, little-endian)
    write_u16_le(&txbuf[offset], n_points);
    offset += 2;
    
    // 6. Raw 5-byte data (N×5 bytes)
    for (uint16_t i = 0; i < n_points; ++i) {
        const PackPoint_t *pt = &points[i];
        
        // Reconstruct 5-byte format according to RPLIDAR protocol:
        // byte0: [Start(1bit) | NotStart(1bit) | Quality(6bits)]
        // byte1-2: angle_q6 (little-endian)
        // byte3-4: distance_q2 (little-endian)
        
        uint8_t quality = pt->quality;
        if (quality > 63) quality = 63;  // Limit to 6-bit range
        
        // byte0: quality occupies low 6 bits, high 2 bits set to 0 (simplified processing)
        txbuf[offset++] = (quality << 2) & 0xFC;
        
        // byte1-2: angle_q6 (little-endian)
        txbuf[offset++] = (uint8_t)(pt->angle_q6 & 0xFF);
        txbuf[offset++] = (uint8_t)((pt->angle_q6 >> 8) & 0xFF);
        
        // byte3-4: distance_q2 (little-endian)
        txbuf[offset++] = (uint8_t)(pt->distance_q2 & 0xFF);
        txbuf[offset++] = (uint8_t)((pt->distance_q2 >> 8) & 0xFF);
    }
    
    // 7. CRC-16/MODBUS (calculate for all bytes from header to end of data)
    uint16_t crc = Radar_CalculateCRC16(txbuf, offset);
    write_u16_le(&txbuf[offset], crc);
    offset += 2;
    
    // 8. Frame trailer (8 bytes)
    memcpy(&txbuf[offset], RAW_FRAME_TRAILER, RAW_FRAME_TRAILER_SIZE);
    offset += RAW_FRAME_TRAILER_SIZE;
    
    // 9. Trigger UART4 DMA transmission
    uart4_tx_busy = 1;
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart4, txbuf, offset);
    
    if (st == HAL_OK) {
        return 0;
    } else {
        uart4_tx_busy = 0;
        return -1;
    }
}

