/**
 ******************************************************************************
 * @file    Encoder.c
 * @brief   电机编码器接口实现
 * @author  Ye Jin
 * @date    2025
 ******************************************************************************
 */

#include "main.h"
#include "encoder.h"

// ============================================================================
// 编码器初始化函数
// ============================================================================

/**
 * @brief 编码器模块初始化
 * @param htim_left 左轮编码器定时器句柄（TIM3）
 * @param htim_right 右轮编码器定时器句柄（TIM2）
 * @note 左轮使用四倍频模式（A+B两相），右轮使用双倍频模式（仅A相）
 */
void Encoder_Init(TIM_HandleTypeDef *htim_left, TIM_HandleTypeDef *htim_right) {
    
    // ========== 配置左轮编码器（TIM3）- 四倍频模式 ==========
    TIM_Encoder_InitTypeDef sEncoderConfigLeft = {0};
    sEncoderConfigLeft.EncoderMode = TIM_ENCODERMODE_TI12;  // 四倍频模式（A+B两相）
    sEncoderConfigLeft.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigLeft.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfigLeft.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigLeft.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    // 初始化左轮编码器
    if (HAL_TIM_Encoder_Init(htim_left, &sEncoderConfigLeft) != HAL_OK) {
        Error_Handler();  // 初始化失败，进入错误处理
    }

    // ========== 配置右轮编码器（TIM2）- 四倍频模式 ==========

    TIM_Encoder_InitTypeDef sEncoderConfigRight = {0};
    sEncoderConfigRight.EncoderMode = TIM_ENCODERMODE_TI12;  // 双倍频模式（仅CH1）
    sEncoderConfigRight.IC1Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigRight.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sEncoderConfigRight.IC2Polarity = TIM_ICPOLARITY_RISING;
    sEncoderConfigRight.IC2Selection = TIM_ICSELECTION_DIRECTTI;

    // 初始化右轮编码器
    if (HAL_TIM_Encoder_Init(htim_right, &sEncoderConfigRight) != HAL_OK) {
        Error_Handler();  // 初始化失败，进入错误处理
    }

    // ========== 启动编码器定时器 ==========

    // 启动左轮编码器（两个通道都启动）
    HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_1);  // 左轮CH1（A相）
    HAL_TIM_Encoder_Start(htim_left, TIM_CHANNEL_2);  // 左轮CH2（B相）

    // 启动右轮编码器（只启动CH1，避免B相干扰）
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_1); // 右轮CH1（A相）
    HAL_TIM_Encoder_Start(htim_right, TIM_CHANNEL_2); // 右轮CH1（A相）
    // 右轮CH2不启动，因为B相信号不稳定

    // ========== 清零编码器计数器 ==========
    __HAL_TIM_SET_COUNTER(htim_left, 0);
    __HAL_TIM_SET_COUNTER(htim_right, 0);
}

// ============================================================================
// 编码器速度读取函数
// ============================================================================

/**
 * @brief 获取两个电机的转速（RPS）
 * @param htim_left 左轮编码器定时器句柄（TIM3）
 * @param htim_right 右轮编码器定时器句柄（TIM2）
 * @param rps_left 输出参数：左轮转速（单位：圈/秒）
 * @param rps_right 输出参数：右轮转速（单位：圈/秒）
 * @param dt 时间间隔（单位：秒）
 * @note 读取计数后会立即清零计数器，准备下次测量
 * @note 前进时两轮速度均为正值，后退时为负值
 */
void Encoder_GetSpeeds(TIM_HandleTypeDef* htim_left, TIM_HandleTypeDef* htim_right, 
                      float* rps_left, float* rps_right, float dt)
{
    // ========== 第一步：读取编码器计数值 ==========
    // 使用int16_t类型，支持正负计数（前进/后退）
    int16_t cnt_left = (int16_t)__HAL_TIM_GET_COUNTER(htim_left);
    int16_t cnt_right = (int16_t)__HAL_TIM_GET_COUNTER(htim_right);
    
    // ========== 第二步：立即清零计数器（准备下次测量）==========
    __HAL_TIM_SET_COUNTER(htim_left, 0);
    __HAL_TIM_SET_COUNTER(htim_right, 0);
    
    // ========== 第三步：时间间隔保护（防止除零错误）==========
    if(dt < 0.001f) {
        dt = 0.001f;  // 最小时间间隔限制为1ms
    }
    
    // ========== 第四步：计算速度（RPS = 计数值 / PPR / 时间间隔）==========
    
    // 左轮速度计算（不取反）
    // 实测：TIM3前进时计数为正，方向正确，无需反转
    *rps_left = (float)cnt_left / ENCODER_PPR_LEFT / dt;
    
    // 右轮速度计算（取反）
    // 实测：TIM2前进时计数为负，需要取反以匹配左轮方向
    // 目标：前进时两个轮子的速度均为正数
    *rps_right = -(float)cnt_right / ENCODER_PPR_RIGHT / dt;
    
    /**
     * 速度计算说明：
     * - 编码器每转一圈产生 PPR 个脉冲
     * - 在 dt 时间内，计数值为 cnt
     * - 转速（圈/秒）= 计数值 / 每圈脉冲数 / 时间间隔
     * 
     * 左轮示例（四倍频，PPR=1560）：
     *   如果 dt=0.02秒（20ms），cnt_left=31
     *   则 rps_left = 31 / 1560 / 0.02 ≈ 0.99 RPS
     * 
     * 右轮示例（双倍频，PPR=780）：
     *   如果 dt=0.02秒（20ms），cnt_right=-16（负数）
     *   则 rps_right = -(-16) / 780 / 0.02 ≈ 1.03 RPS
     * 
     * 注意：
     * - 左右轮PPR不同，是因为右轮只用单相（双倍频）
     * - 右轮取反是因为编码器安装方向或接线导致计数方向相反
     * - 最终结果：前进时两轮速度都是正数，便于PID控制
     */
}
