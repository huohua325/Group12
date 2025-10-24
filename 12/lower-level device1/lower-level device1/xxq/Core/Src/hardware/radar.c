/**
 ******************************************************************************
 * @file           : radar.c
 * @brief          : 激光雷达模块实现（连续扫描360度点云方案）
 ******************************************************************************
 * @attention
 * 
 * 🔄 **重构说明**（2025-10-20）
 * - 移除三区域避障处理（由上位机完成）
 * - 实现连续扫描360度点云数据采集
 * - 二进制帧传输：1012字节/帧 × 10Hz = 10.1KB/s
 * - 波特率460800，带宽利用率34%，非常富裕
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
 *  🔧 核心功能实现
 * ========================================================================== */

/**
 * @brief 初始化雷达连续扫描模式（DMA优化版）
 */
uint8_t Radar_Init(UART_HandleTypeDef *huart_radar)
{
    if (huart_radar == NULL) {
        return 1;  // 参数错误
    }
    
    // 停止雷达（确保处于已知状态）
    Radar_StopScan(huart_radar);
    HAL_Delay(100);
    // 初始化全局缓冲区
    // 启动雷达扫描
    HAL_StatusTypeDef ret = Radar_StartScan(huart_radar);

    LIDAR_Start(huart_radar);
    //ret = HAL_UART_Receive_DMA(huart_radar, (uint8_t*)g_radar_dma_buffer, RADAR_DMA_BUFFER_SIZE);
    __HAL_UART_ENABLE_IT(huart_radar, UART_IT_IDLE);
    return 0;  // 成功
}

/**
 * @brief 启动雷达扫描
 */
HAL_StatusTypeDef Radar_StartScan(UART_HandleTypeDef *huart_radar)
{
    uint8_t start_cmd[] = RADAR_CMD_START_SCAN;
    return HAL_UART_Transmit(huart_radar, start_cmd, sizeof(start_cmd), 100);
}

/**
 * @brief 停止雷达扫描
 */
HAL_StatusTypeDef Radar_StopScan(UART_HandleTypeDef *huart_radar)
{
    uint8_t stop_cmd[] = RADAR_CMD_STOP_SCAN;
    return HAL_UART_Transmit(huart_radar, stop_cmd, sizeof(stop_cmd), 100);
}


/**
 * @brief ⭐ UART3 IDLE中断回调函数（DMA接收核心）
 * @note  在 stm32f4xx_it.c 的 USART3_IRQHandler 中调用
 */
void Radar_UART_IdleCallback(UART_HandleTypeDef *huart_radar)
{
    
    if (huart_radar == NULL || huart_radar->hdmarx == NULL) {
        return;
    }
    LIDAR_UART_IdleCallback(&huart3);
    // 清除IDLE中断标志
    __HAL_UART_CLEAR_IDLEFLAG(huart_radar);
}

/**

/**
 * @brief 计算CRC16校验和（CRC-16/MODBUS）
 * @param data 数据缓冲区
 * @param length 数据长度
 * @return CRC16校验值
 */
static uint16_t Radar_CalculateCRC16(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;  // 初始值

    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;  // 多项式
            } else {
                crc >>= 1;
            }
        }
    }

    return crc;
}



/* External UART handles */
extern UART_HandleTypeDef huart3; // 雷达接收（UART3）
extern UART_HandleTypeDef huart4; // 上位机发送（UART4）

/* ============================================================================
 *  DMA接收 + 环形缓冲区（ISR快速路径）
 * ========================================================================== */

/* DMA 接收缓冲（与 HAL_UART_Receive_DMA 关联） */
static uint8_t rx_dma_buf[LIDAR_RX_DMA_BUF_SZ];

/* 环形处理缓冲（用于将DMA数据快速复制到主循环解析） */
static uint8_t proc_ring[LIDAR_PEND_RING_SZ];
static volatile uint16_t proc_head = 0; // 写入索引 (ISR)
static volatile uint16_t proc_tail = 0; // 读取索引 (主循环)

/* UART4 DMA 发送状态标志 */
static volatile uint8_t uart4_tx_busy = 0;

/* 最终点存储区（写入由解析完成） */
static volatile LidarPoint_t lidar_points[LIDAR_MAX_POINTS];
static volatile uint16_t lidar_point_index = 0;

/* 完成一圈的备份区与标志 */
static volatile LidarPoint_t completed_rotation[LIDAR_MAX_POINTS];
static volatile uint16_t completed_count = 0;
static volatile uint8_t rotation_ready = 0;

/* 解析状态变量 */
static uint16_t last_angle_q6 = 0xFFFF; // 上一个角度（初始化标志）
static uint32_t last_rotation_tick = 0;
static uint32_t point_counter_total = 0;
static uint32_t last_rotation_point_counter = 0;

/* 状态机：等待起始应答 → 接收数据 */
typedef enum {
    WAIT_START_RESP,
    RECEIVING_DATA
} ParserState_t;

static ParserState_t parser_state = WAIT_START_RESP;
static uint8_t frame_len = 0;
static uint8_t data_type = 0;

/* 数据质量统计（用于调试） */
static uint32_t total_frames_received = 0;
static uint32_t valid_frames_count = 0;
static uint32_t sync_errors = 0;
static uint32_t quality_filtered = 0;
static uint32_t range_filtered = 0;

/* 内部函数声明 */
static void try_emit_rotation_by_angle_wrap(uint16_t angle_q6);

/* ============================================================================
 *  IDLE 中断快速路径（仅拷贝数据，不解析）
 * ========================================================================== */

/**
 * @brief IDLE中断回调 - 快速路径（非常短，仅拷贝）
 * @note 在中断上下文被调用，必须尽快返回
 */
void LIDAR_UART_IdleCallback_IRQ(UART_HandleTypeDef *huart)
{
    if (!huart || !huart->hdmarx) return;

    /* 清 IDLE 标志：读SR/DR */
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
    (void)tmp;

    /* 停止 DMA，获取已接收字节数 */
    HAL_UART_DMAStop(huart);
    uint32_t dma_cnt = __HAL_DMA_GET_COUNTER(huart->hdmarx); // 剩余计数
    uint16_t recvd = (uint16_t)(LIDAR_RX_DMA_BUF_SZ - dma_cnt);

    if (recvd > 0) {
        /* 快速拷贝到环形缓冲（避免栈溢出） */
        uint16_t write_idx = proc_head;
        for (uint16_t i = 0; i < recvd; ++i) {
            proc_ring[write_idx++] = rx_dma_buf[i];
            if (write_idx >= LIDAR_PEND_RING_SZ) write_idx = 0;
        }
        /* 更新写索引（原子操作，中断上下文） */
        proc_head = write_idx;
    }

    /* 重启 DMA 接收（尽快） */
    HAL_UART_Receive_DMA(huart, rx_dma_buf, LIDAR_RX_DMA_BUF_SZ);
}

/* 启动 DMA 接收 */
void LIDAR_Start(UART_HandleTypeDef *huart)
{
    HAL_UART_Receive_DMA(huart, (uint8_t *)rx_dma_buf, LIDAR_RX_DMA_BUF_SZ);
}

/* ============================================================================
 *  主循环处理函数（滑动窗口 + 状态机解析）
 * ========================================================================== */

/**
 * @brief 主循环处理函数：解析环形缓冲区中的雷达数据
 * @note 在主循环中周期调用，执行滑动窗口帧同步与逐帧校验
 */
void LIDAR_ProcessPendingData(void)
{
    /* 快速读取索引（避免在中间被ISR改写） */
    uint16_t tail = proc_tail;
    uint16_t head = proc_head;

    if (tail == head) return; // 无数据

    /* 临时线性缓冲以减少每个字节读取开销 */
    static uint8_t temp_buf[1024];
    uint16_t temp_len = 0;

    /* 把环形缓冲的数据一次性拉成线性块 */
    while (tail != head && temp_len < sizeof(temp_buf)) {
        temp_buf[temp_len++] = proc_ring[tail++];
        if (tail >= LIDAR_PEND_RING_SZ) tail = 0;
    }
    proc_tail = tail; // 更新全局tail

    /* 将temp_buf串接到解析缓冲区 */
    static uint8_t parse_buf[1024];
    static uint16_t parse_len = 0;

    /* 防止溢出 */
    if (parse_len + temp_len > sizeof(parse_buf)) {
        parse_len = 0; // 放弃旧数据，重新开始
    }
    memcpy(parse_buf + parse_len, temp_buf, temp_len);
    parse_len += temp_len;

    /* 滑动窗口解析（逐字节尝试） */
    uint16_t pos = 0;

    while (parse_len - pos >= 2) {
        if (parser_state == WAIT_START_RESP) {
            /* 查找 0xA5 0x5A 起始应答 */
            if (parse_buf[pos] == 0xA5 && parse_buf[pos+1] == 0x5A) {
                /* 确保起始应答完整（7字节） */
                if (parse_len - pos >= 7) {
                    /* 解析DataLen & DataType
                     * 手册格式：A5 5A [len_low] [len_mid] [len_high] [mode] [dtype]
                     */
                    uint32_t data_len_raw = ((uint32_t)parse_buf[pos+2]) |
                                            ((uint32_t)parse_buf[pos+3] << 8) |
                                            ((uint32_t)parse_buf[pos+4] << 16);
                    uint8_t dtype = parse_buf[pos+6];

                    /* SCAN模式：dtype=0x81, data_len=5 */
                    if (data_len_raw > 0 && dtype == 0x81) {
                        frame_len = (uint8_t)data_len_raw;
                        parser_state = RECEIVING_DATA;
                        data_type = dtype;
                        pos += 7; // 消费起始应答
                        continue;
                    } else {
                        pos++; // 跳过1字节继续找
                    }
                } else {
                    break; // 等待更多字节
                }
            } else {
                pos++; // 未找到头
            }
        } else { /* RECEIVING_DATA */
            if (frame_len == 0 || frame_len != 5) {
                parser_state = WAIT_START_RESP;
                break;
            }

            /* 确保有完整帧 */
            if (parse_len - pos < frame_len) break;

            total_frames_received++;

            /* 提取字节 */
            uint8_t *f = &parse_buf[pos];
            uint8_t b0 = f[0];
            uint8_t quality = b0 >> 2;
            uint8_t s = b0 & 0x1;
            uint8_t s_inv = (b0 & 0x2) >> 1;
            uint16_t angle_q6 = (uint16_t)f[1] | ((uint16_t)f[2] << 8);
            uint16_t distance_q2 = (uint16_t)f[3] | ((uint16_t)f[4] << 8);

            /* 数据有效性检查（组合多因子） */
            uint8_t valid = 1;

            /* S/Sbar一致性（可选，不强制）*/
            if (s == s_inv) {
                sync_errors++;
                // 不强制丢弃，只统计
            }

            /* 质量检查（可配置阈值） */
            if (quality < 5) {
                valid = 0;
                quality_filtered++;
            }

            /* 角度范围（23040对应360°，允许小误差） */
            if (angle_q6 > 23500) {
                valid = 0;
                range_filtered++;
            }

            /* 距离范围（Q2格式->mm） */
            uint32_t dist_mm = distance_q2 >> 2;
            if (dist_mm == 0 || dist_mm < 30 || dist_mm > 16000) {
                valid = 0;
                range_filtered++;
            }

            if (valid) {
                /* 存储有效点 */
                if (lidar_point_index < LIDAR_MAX_POINTS) {
                    LidarPoint_t pt;
                    pt.angle_q6 = angle_q6;
                    pt.distance_q2 = distance_q2;
                    pt.quality = quality;
                    pt.timestamp = HAL_GetTick();

                    lidar_points[lidar_point_index++] = pt;
                    point_counter_total++;
                }

                /* 尝试触发旋转完成 */
                try_emit_rotation_by_angle_wrap(angle_q6);
                valid_frames_count++;

                pos += frame_len; // 消费帧
            } else {
                /* 无效帧：滑动1字节继续找同步点 */
                pos += 1;
            }
        }
    } /* end while parsing */

    /* 保留未消费的字节到下次处理 */
    if (pos < parse_len) {
        uint16_t remain = parse_len - pos;
        memmove(parse_buf, parse_buf + pos, remain);
        parse_len = remain;
    } else {
        parse_len = 0;
    }
}

/**
 * @brief 角度回绕检测触发旋转完成（优化版）
 * @note 使用更合理的角度阈值 + 时间保护
 */
static void try_emit_rotation_by_angle_wrap(uint16_t angle_q6)
{
    /* 首次初始化 */
    if (last_angle_q6 == 0xFFFF) {
        last_angle_q6 = angle_q6;
        last_rotation_tick = HAL_GetTick();
        return;
    }

    uint32_t points_since_last = point_counter_total - last_rotation_point_counter;

    /* 角度回绕检测（Q6格式：0-23040对应0-360°）
     * 优化参数：从接近360°回绕到接近0°
     */
    const uint16_t wrap_high_q6 = 22000;  // ~343°
    const uint16_t wrap_low_q6  = 3000;   // ~47°

    /* 条件1：角度回绕 + 点数足够 */
    if ((last_angle_q6 > wrap_high_q6) &&
        (angle_q6 < wrap_low_q6) &&
        (points_since_last >= LIDAR_MIN_POINTS_PER_ROTATION)) {

        /* 原子复制到完成缓冲区 */
        __disable_irq();
        uint16_t ncopy = (lidar_point_index <= LIDAR_MAX_POINTS) ? lidar_point_index : LIDAR_MAX_POINTS;
        for (uint16_t i = 0; i < ncopy; ++i) {
            completed_rotation[i] = lidar_points[i];
        }
        completed_count = ncopy;
        rotation_ready = 1;

        /* 重置下一圈 */
        lidar_point_index = 0;
        last_rotation_point_counter = point_counter_total;
        last_rotation_tick = HAL_GetTick();
        __enable_irq();
    }

    /* 条件2：时间保护（1秒超时） */
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

/* 上层调用以获取完成的一圈数据；返回点数（若无准备则返回0） */
uint16_t LIDAR_GetCompletedRotation(LidarPoint_t *out_buffer, uint16_t max_count)
{
    uint16_t ret = 0;
    if (!rotation_ready) return 0;

    __disable_irq();
    uint16_t n = (completed_count <= max_count) ? completed_count : max_count;
    for (uint16_t i = 0; i < n; ++i) {
        out_buffer[i] = completed_rotation[i];
    }
    /* 复位准备标志 */
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
 *  数据打包与上位机发送（1012字节二进制帧）
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
 * @brief 打包并通过UART4 DMA发送雷达数据帧（1012字节）
 * @param points 点云数据数组（PackPoint_t = LidarPoint_t）
 * @param n_points 点数
 * @retval 0=成功启动发送, -1=UART忙, -2=参数错误
 */
int LIDAR_PackAndSendFrame(const PackPoint_t *points, uint16_t n_points)
{
    if (points == NULL || n_points == 0) return -2;

    /* 检查 UART4 空闲 */
    if (uart4_tx_busy != 0) {
        return -1;  // 上一次发送未完成
    }

    static uint8_t txbuf[LIDAR_PACKET_SIZE];
    memset(txbuf, 0, sizeof(txbuf));

    /* 1) 帧头 4 字节 */
    txbuf[0] = 0xA5;
    txbuf[1] = 0x5A;
    txbuf[2] = 0x4C;
    txbuf[3] = 0x44;

    /* 2) 时间戳（uint32_t, LE）*/
    uint32_t now = HAL_GetTick();
    write_u32_le(&txbuf[4], now);

    /* 3) 点数 (uint16_t LE) 固定500 */
    write_u16_le(&txbuf[8], (uint16_t)LIDAR_POINT_COUNT);

    /* 4) 初始化距离数组为 0 （表示无效）*/
    uint16_t distances[LIDAR_POINT_COUNT];
    for (uint32_t i = 0; i < LIDAR_POINT_COUNT; ++i) distances[i] = 0;

    /* 5) 遍历输入点，映射到索引并保留最小非0距离 */
    for (uint32_t i = 0; i < n_points; ++i) {
        uint16_t a_q6 = points[i].angle_q6;
        uint16_t d_q2 = points[i].distance_q2;
        if (d_q2 == 0) continue; // 无效点跳过

        /* 角度（度） */
        float angle_deg = ((float)a_q6) / 64.0f;

        /* 计算索引： round(angle / ANGLE_PER_INDEX) mod 500 */
        float idxf = angle_deg / ANGLE_PER_INDEX;
        uint32_t idx = (uint32_t)(roundf(idxf)) % LIDAR_POINT_COUNT;

        /* 将 q2 -> mm (integer) */
        uint16_t dist_mm = (uint16_t)(d_q2 / 4);

        /* 若该槽为空（0）或新点更接近（数值更小）则替换 */
        if (distances[idx] == 0 || (dist_mm > 0 && dist_mm < distances[idx])) {
            distances[idx] = dist_mm;
        }
    }

    /* 6) 把 distances 写入 txbuf[10..1009]，小端 uint16_t */
    uint32_t p = 10;
    for (uint32_t i = 0; i < LIDAR_POINT_COUNT; ++i) {
        txbuf[p++] = (uint8_t)(distances[i] & 0xFF);
        txbuf[p++] = (uint8_t)((distances[i] >> 8) & 0xFF);
    }
    /* 此时 p 应是 1010 */

    /* 7) 计算 CRC-16/MODBUS（对 0..1009 字节计算） */
    uint16_t crc = Radar_CalculateCRC16(txbuf, 1010);
    write_u16_le(&txbuf[1010], crc);

    /* 8) 触发 UART4 DMA 发送 */
    uart4_tx_busy = 1;  // 标记发送开始
    HAL_StatusTypeDef st = HAL_UART_Transmit_DMA(&huart4, txbuf, LIDAR_PACKET_SIZE);
    if (st == HAL_OK) {
        return 0;
    } else {
        uart4_tx_busy = 0;  // 发送启动失败，清除标志
        return -1;
    }
}

/**
 * @brief UART4 DMA发送完成回调
 * @note 在 HAL_UART_TxCpltCallback 中调用
 */
void LIDAR_TxCpltCallback(void)
{
    uart4_tx_busy = 0;  // 标记发送完成
}

