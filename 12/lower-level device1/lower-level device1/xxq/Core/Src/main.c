/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 智能小车主控程序
  * @version        : v1.0.0
  * @date           : 2025-01-XX
  ******************************************************************************
  * @description
  * 
  * 本程序实现了基于STM32F446RET6的智能小车控制系统，集成以下功能模块：
  * 
  * ┌─────────────────────────────────────────────────────────────────┐
  * │                    智能小车系统架构                               │
  * ├─────────────────────────────────────────────────────────────────┤
  * │ • 电机控制系统 (TIM1 PWM + PID速度控制)                         │
  * │ • 编码器反馈系统 (TIM2/TIM3 正交编码器)                         │
  * │ • （已移除）MPU6500姿态传感器                                  │
  * │ • 激光雷达系统 (UART3 通信)                                     │
  * │ • 蓝牙通信模块 (UART4 无线控制)                                 │
  * │ • 调试串口 (UART2 数据输出)                                     │
  * └─────────────────────────────────────────────────────────────────┘
  * 
  * @hardware_connections
  * ┌─────────────┬──────────────┬─────────────────────────────────┐
  * │   功能模块   │   STM32引脚   │           描述                  │
  * ├─────────────┼──────────────┼─────────────────────────────────┤
  * │ 左电机PWM    │ PA8 (TIM1_CH1)│ L298N驱动器PWM输入              │
  * │ 右电机PWM    │ PA10(TIM1_CH3)│ L298N驱动器PWM输入              │
  * │ 左编码器     │ PA15/PB3     │ TIM3正交编码器输入              │
  * │ 右编码器     │ PA0/PA1      │ TIM2正交编码器输入              │
  * │ I2C1         │ PB8/PB9      │ 可用I2C总线（当前未接IMU）      │
  * │ 激光雷达     │ PC10/PC5     │ UART3雷达通信接口               │
  * │ 蓝牙模块     │ PA0/PA1      │ UART4无线控制接口               │
  * │ 调试串口     │ PA2/PA3      │ UART2 PC调试接口                │
  * └─────────────┴──────────────┴─────────────────────────────────┘
  * 
  * @author         : Ye Jin
  * @organization   : 12组
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // 阶段1: 添加atoi函数支持
#include <math.h>
#include "i2c.h"
#include "gpio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "hardware/Motor.h"
#include "hardware/pid.h"
#include "hardware/Encoder.h"
#include "hardware/radar.h"



/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* Tim1
 * PA8-Tim1_CH1,PA10-TIM1_CH3
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * */




/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* ============================================================================ */
/*                        PRIVATE FUNCTION PROTOTYPES                          */
/* ============================================================================ */

void SystemClock_Config(void);
static void System_StatusUpdate(void);
static void System_ErrorHandler(const char* error_msg);
static void send_odometry_data(void);
static void Command_ProcessUserInput(uint8_t command);
static void Command_ShowSystemInfo(void);

// 阶段1新增：Python命令处理函数
static void Python_ParseCommand(void);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
// UART5传感器数据处理函数
static void UART5_ProcessSensorData(void);
static void UART5_ParseDataPacket(uint8_t* data, uint16_t length);

/* ============================================================================ */
/*                           GLOBAL VARIABLES                                  */
/* ============================================================================ */

/**
  * @defgroup PERIPHERAL_HANDLES 外设句柄定义
  * @brief 系统中使用的所有外设句柄
  * @{
  */
UART_HandleTypeDef huart2;  ///< USART2: PC调试串口 (PA2-TX, PA3-RX)
UART_HandleTypeDef huart3;  ///< USART3: 激光雷达通信 (PC10-TX, PC5-RX)  
UART_HandleTypeDef huart4;  ///< USART4: 蓝牙无线控制 (PA0-TX, PA1-RX)
UART_HandleTypeDef huart5;  ///< UART5: 传感器数据接收 (PC12-TX, PD2-RX)
TIM_HandleTypeDef htim1;    ///< TIM1: 电机PWM输出定时器 (PA8, PA10)
TIM_HandleTypeDef htim2;    ///< TIM2: 右轮编码器定时器 (PA0, PA1)
TIM_HandleTypeDef htim3;    ///< TIM3: 左轮编码器定时器 (PA15, PB3)
TIM_HandleTypeDef htim6;    ///< TIM6: 系统定时器 (中断服务)
I2C_HandleTypeDef hi2c1;    ///< I2C1: 可用总线（当前未使用IMU）
/**
  * @}
  */

/**
  * @defgroup COMMUNICATION_BUFFERS 通信缓冲区
  * @brief 各种通信接口的数据缓冲区
  * @{
  */
uint8_t g_uart_rx_buffer;          ///< 蓝牙命令接收缓冲区（单字节）
uint8_t g_debug_rx_buffer;         ///< 调试串口接收缓冲区  
uint8_t g_radar_rx_buffer;         ///< 雷达数据接收缓冲区
uint8_t g_uart5_rx_buffer;         ///< UART5传感器数据接收缓冲区（单字节）

// Python端命令接收缓冲区（阶段1新增）
#define RX_BUFFER_SIZE 256
char rx_command_buffer[RX_BUFFER_SIZE];     ///< Python命令缓冲区
volatile uint8_t rx_cmd_index = 0;          ///< 命令缓冲区索引
volatile uint8_t rx_cmd_complete = 0;       ///< 命令接收完成标志
uint8_t CTL = 0;       ///< 命令接收完成标
// UART5传感器数据包解析
#define UART5_RX_BUFFER_SIZE 64             ///< UART5接收缓冲区大小
uint8_t uart5_rx_data[UART5_RX_BUFFER_SIZE]; ///< UART5数据接收缓冲区
volatile uint16_t uart5_rx_index = 0;       ///< UART5接收缓冲区索引
volatile uint8_t uart5_packet_ready = 0;    ///< UART5数据包接收完成标志
volatile uint8_t uart5_header_found = 0;    ///< UART5包头检测标志
volatile uint16_t uart5_packet_start = 0;   ///< UART5数据包起始位置
int16_t g_yaw_data = 0;                     ///< 解析出的Yaw数据（YawL + YawH）
float g_yaw_angle = 0.0f;                   ///< 解析出的偏航角（度）


// ODO数据发送（阶段1: PWM标定支持）
uint32_t last_odo_time = 0;                 ///< 上次发送ODO数据的时间
#define ODO_SEND_INTERVAL 100               ///< ODO数据发送间隔（100ms = 10Hz）
#define LEFT_ENCODER_PPR   1560             ///< 左轮编码器每圈脉冲数
#define RIGHT_ENCODER_PPR  1560              ///< 右轮编码器每圈脉冲数

/**
  * @}
  */

/**
  * @defgroup SYSTEM_STATE_VARIABLES 系统状态变量
  * @brief 系统运行状态和控制参数
  * @{
  */
static SystemState_t g_system_state = SYSTEM_STATE_INIT;
// 已移除：g_motor_mode, g_pid_control_enabled (改用Motor_Command_XXX API)
static float g_target_speed_rps = DEFAULT_TARGET_SPEED;

// 命令序列号机制（阶段3：迷宫探索）
static uint16_t g_last_command_seq = 0;      // 上次接收的命令序列号
static uint8_t g_action_in_progress = 0;     // 动作正在执行标志
static uint32_t g_action_start_time = 0;     // 动作开始时间（ms）

// 已移除 MPU6500 相关全局变量
/**
  * @}
  */
/**
  ******************************************************************************
  * @brief  程序主入口函数
  * @param  None
  * @retval int: 程序退出状态码 (理论上不会返回)
  * 
  * @details
  * 系统初始化流程：
  * 1. 硬件抽象层初始化
  * 2. 系统时钟配置  
  * 3. 外设模块初始化
  * 4. 功能模块初始化
  * 5. 进入主循环
  ******************************************************************************
  */
int main(void)
{
  /* ========================================================================== */
  /*                          SYSTEM INITIALIZATION                            */
  /* ========================================================================== */
  
  // Step 1: Initialize the HAL Library
  HAL_Init();

  // Step 2: Configure the system clock to 168MHz
  SystemClock_Config();

  // Step 3: Initialize all configured peripherals
  MX_GPIO_Init();           // GPIO配置初始化
  MX_I2C1_Init();           // I2C1总线初始化
  MX_I2C2_Init();           // I2C2总线初始化 (备用)
  MX_SPI1_Init();           // SPI1接口初始化 (备用)
  MX_TIM1_Init();           // 电机PWM定时器初始化
  MX_TIM2_Init();           // 右轮编码器定时器初始化
  MX_TIM3_Init();           // 左轮编码器定时器初始化
  MX_TIM6_Init();           // 系统基准定时器初始化
  MX_UART4_Init();          // 蓝牙通信串口初始化
  MX_USART2_UART_Init();    // 调试串口初始化
  MX_USART3_UART_Init();    // 激光雷达串口初始化
  MX_UART5_Init();          // 传感器数据串口初始化
  /* ========================================================================== */
  /*                        FUNCTIONAL MODULES SETUP                           */
  /* ========================================================================== */
  
  // 启动编码器定时器 (正交编码器模式)
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
      System_ErrorHandler("Encoder timer startup failed");
  }

  // 初始化电机控制系统
  Motor_Init(&htim1);
  
  // 初始化PID控制器
  Motor_PID_Init();

  // 初始化激光雷达模块（连续扫描模式）
  if (Radar_Init(&huart3) != 0) {
      System_ErrorHandler("Radar module initialization failed");
  }
  
  // 发送雷达初始化成功消息
  const char* radar_msg = "[INFO] Radar continuous scan mode initialized\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)radar_msg, strlen(radar_msg), 100);
  
  // 系统启动提示
  Command_ShowSystemInfo();
  
  /* ======================================================================== */
  /*                    PYTHON COMMAND INTERFACE SETUP (阶段1)                 */
  /* ======================================================================== */
  
  // 发送就绪消息给Python端
  const char* ready_msg = "STM32 Ready - Waiting for Python commands...\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)ready_msg, strlen(ready_msg), 100);
  
  // 启动UART4接收中断（用于接收Python命令）
  HAL_UART_Receive_IT(&huart4, &g_uart_rx_buffer, 1);
  // 启动UART5接收中断（用于接收传感器数据）
  HAL_StatusTypeDef uart5_status = HAL_UART_Receive_IT(&huart5, &g_uart5_rx_buffer, 1);

  // 发送UART5初始化状态
  char uart5_msg[80];
  if (uart5_status == HAL_OK) {
      snprintf(uart5_msg, sizeof(uart5_msg), "[INFO] UART5 initialized successfully (PC12-TX, PD2-RX, 9600)\r\n");
  } else {
      snprintf(uart5_msg, sizeof(uart5_msg), "[ERROR] UART5 init failed: %d\r\n", uart5_status);
  }
  HAL_UART_Transmit(&huart4, (uint8_t*)uart5_msg, strlen(uart5_msg), 100);

  /* ======================================================================== */
  /*                            MAIN CONTROL LOOP                              */
  /* ======================================================================== */
  
  g_system_state = SYSTEM_STATE_RUNNING;

  uint32_t loop_counter = 0;
  static PackPoint_t points_buf[LIDAR_MAX_POINTS]; // 静态分配，避免栈溢出
  
  while (1)
  {
      loop_counter++;
      
      // ⭐ 步骤1：处理环形缓冲区中的雷达数据（状态机 + 滑动窗口解析）
      LIDAR_ProcessPendingData();
      
      // ⭐ 步骤2：如果有完整的一圈数据，打包并发送到上位机
      uint16_t n = LIDAR_GetCompletedRotation((LidarPoint_t*)points_buf, LIDAR_MAX_POINTS);
      if (n > 0) {
          int r = LIDAR_PackAndSendFrame(points_buf, n);
          if (r != 0) {
              // 发送未能启动（UART忙），下次循环再试
          }
      }
      
      // 系统状态更新
      System_StatusUpdate();
//      // 处理UART5传感器数据
      UART5_ProcessSensorData();
//      // PID控制循环 (20ms周期执行)
      Motor_PID_Control(&htim1, &htim3, &htim2);
//      // 阶段1：处理Python命令（从中断缓冲区）
     Python_ParseCommand();
//
      // ========== 阶段3：动作完成检测与ACK回复 ==========
      // 注意：Motor_Command_XXX 函数是非阻塞的，会在到达目标后自动停止
      // 我们通过检测速度接近0来判断动作完成
      if (g_action_in_progress) {
          uint32_t elapsed = HAL_GetTick() - g_action_start_time;
          float left_rps, right_rps;
          Motor_GetActualSpeed(&left_rps, &right_rps);
          
          // 动作完成判断：速度接近0（进入PID死区）
          float speed_threshold = 0.08f;  // RPS，较小的阈值确保真正停止
          uint32_t min_time = 1000;  // 至少等待1秒，避免误判
          
          // 检测动作完成：时间>1秒且速度接近0
          if (elapsed > min_time && 
              fabsf(left_rps) < speed_threshold && 
              fabsf(right_rps) < speed_threshold) {
              
              // 调试：显示即将发送的序列号
              char pre_ack_debug[64];
              snprintf(pre_ack_debug, sizeof(pre_ack_debug), 
                      "[DEBUG] Sending ACK: g_last_command_seq=%u\r\n", g_last_command_seq);
              HAL_UART_Transmit(&huart4, (uint8_t*)pre_ack_debug, strlen(pre_ack_debug), 100);
              
              // 发送完成回复：ACK,seq\n
              char ack[48];
              snprintf(ack, sizeof(ack), "ACK,%u,OK,%.2fs\r\n", 
                      g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)ack, strlen(ack), 100);
              
              g_action_in_progress = 0;
              
              // 调试日志
              char log[64];
              snprintf(log, sizeof(log), "[DEBUG] Action completed: seq=%u time=%.2fs\r\n",
                      g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)log, strlen(log), 100);
          }
          
          // 超时保护（35秒，给Motor_Command足够时间）
          if (elapsed > 35000) {
              char timeout_msg[64];
              snprintf(timeout_msg, sizeof(timeout_msg), 
                      "ACK,%u,TIMEOUT,%.2fs\r\n", g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)timeout_msg, strlen(timeout_msg), 100);
              g_action_in_progress = 0;
              
              // 强制停止电机
              Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
              Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
          }
      }
//
      // 阶段1：发送里程计数据（10Hz，为PWM标定提供速度数据）
      //send_odometry_data();
  }
}

/* ============================================================================ */
/*                         PRIVATE FUNCTION DEFINITIONS                        */
/* ============================================================================ */

/**
  ******************************************************************************
  * @brief  处理用户命令输入
  * @param  command: 接收到的命令字符
  * @retval None
  * 
  * @details
  * 命令分类：
  * - 运动控制: 0-9 (停止、前进、转向等)
  * - 速度调整: +/- (增减目标速度)
  * - 系统测试: E,Q,L,R,D,P,p,Z (编码器测试)
  * - 传感器: （IMU相关命令已移除）
  * - 雷达: A,a,r,B (激光雷达操作)
  * - 调试: ?,H,C (帮助和诊断)
  ******************************************************************************
  */
static void Command_ProcessUserInput(uint8_t command)
{
    static float current_speed_left = 0.0f;
    static float current_speed_right = 0.0f;
    
    switch(command) {
        /* ================================================================== */
        /*                        MOTION CONTROL COMMANDS                    */
        /* ================================================================== */

        case '5': // 查看当前速度
            {
                char speed_report[120];
                
                // 直接从Motor.c读取速度（Motor_Command API已经在后台运行）
                Motor_GetActualSpeed(&current_speed_left, &current_speed_right);
                snprintf(speed_report, sizeof(speed_report), 
                        "[SPEED] L: %.2f RPS | R: %.2f RPS | Target: %.1f RPS\r\n",
                        current_speed_left, current_speed_right, g_target_speed_rps);
                
                HAL_UART_Transmit(&huart2, (uint8_t*)speed_report, strlen(speed_report), 100);
            }
            break;
            
        case '?': // 显示帮助信息
            {
                const char help_menu[] = 
                    "\r\n+========================== Smart Car Commands ===========================+\r\n"
                    "|                                                                    |\r\n"
                    "| [MOTION CONTROL]              |                                    |\r\n"
                    "| 1 - PID Forward Mode          |                                    |\r\n"
                    "| 4 - PID Backward Mode         |                                    |\r\n"
                    "| 2 - Turn Left (50%% PWM)       |                                    |\r\n"
                    "| 3 - Turn Right (50%% PWM)      |                                    |\r\n"
                    "| 0 - Emergency Stop            |                                    |\r\n"
                    "|                               |                                   |\r\n"
                    "| [SPEED CONTROL]               | [LIDAR RADAR]                     |\r\n"
                    "| + - Increase Speed (+0.5 RPS) | A - Start Scan                    |\r\n"
                    "| - - Decrease Speed (-0.5 RPS) | B - Stop Scan                     |\r\n"
                    "| 5 - Check Current Speed       |                                   |\r\n"
                    "|                               |                                   |\r\n"
                    "| [PYTHON INTERFACE]            |                                   |\r\n"
                    "| CMD,seq,mode - Command        | [SYSTEM]                          |\r\n"
                    "|                               | ? - Show This Help                |\r\n"
                    "|                               | Current Target: %.1f RPS           |\r\n"
                    "|                               | System Status: %s                 |\r\n"
                    "|                                                                    |\r\n"
                    "+====================================================================+\r\n\r\n";
                
                char formatted_help[1200];
                const char* status_str = (g_system_state == SYSTEM_STATE_RUNNING) ? "Running" : 
                                        (g_system_state == SYSTEM_STATE_READY) ? "Ready" : 
                                        (g_system_state == SYSTEM_STATE_ERROR) ? "Error" : "Initializing";
                
                snprintf(formatted_help, sizeof(formatted_help), help_menu, g_target_speed_rps, status_str);
                HAL_UART_Transmit(&huart4, (uint8_t*)formatted_help, strlen(formatted_help), HAL_MAX_DELAY);
            }
            break;
            
        /* ================================================================== */
        /*                        SPEED ADJUSTMENT COMMANDS                   */
        /* ================================================================== */
        case 'F':
        	{
        		Motor_Command_MoveForward(60.0f,30000);

        	}break;
        case 'L':
           	{
           		Motor_Command_TurnLeft(90.0f, 10000);

           	}break;
        case 'R':
           	{
           		Motor_Command_TurnRight(90.0f, 10000);

           	}break;
        case 'S':
           	{
                Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
                Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
                CTL=1;
           	}break;
        /* ================================================================== */
        /*                      LIDAR RADAR COMMANDS                          */
        /* ================================================================== */
        // 注意：雷达现在运行在连续扫描模式，主循环中自动发送数据到上位机
        // 数据格式：二进制帧（1012字节/帧，10Hz更新频率）
            
        case 'A': // 重启雷达扫描
        case 'a':
            {
                Radar_StopScan(&huart3);
                HAL_Delay(100);
                Radar_ClearReceiveBuffer(&huart3);
                
                if (Radar_StartScan(&huart3) == HAL_OK) {
                    HAL_UART_Transmit(&huart4, (uint8_t*)"[LIDAR] Scan restarted (continuous mode)\r\n", 43, 100);
                } else {
                    HAL_UART_Transmit(&huart4, (uint8_t*)"[ERROR] Failed to restart radar\r\n", 33, 100);
                }
            }
            break;
            
        case 'B': // 停止雷达扫描
        case 'b':
            {
                Radar_StopScan(&huart3);
                HAL_UART_Transmit(&huart4, (uint8_t*)"[LIDAR] Scan stopped\r\n", 22, 100);
            }
            break;
            
        // 已移除 'n' 快速校准命令
            
        // 已移除 'N' 重新初始化MPU命令
            
        /* ================================================================== */
        /*                       ENCODER TEST COMMANDS                       */
        /* ================================================================== */
        
        default:
            // 未知命令处理
            {
                char error_msg[50];
                snprintf(error_msg, sizeof(error_msg), 
                        "[ERROR] Unknown command: 0x%02X\r\n", command);
                HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), HAL_MAX_DELAY);
            }
            break;
    }
}

/**
  ******************************************************************************
  * @brief  显示系统信息和启动提示
  * @param  None
  * @retval None
  ******************************************************************************
  */
static void Command_ShowSystemInfo(void)
{
    const char system_info[] = 
        "\r\n"
        "+================================================================+\r\n"
        "|            Smart Car Control System v1.0.0                    |\r\n"
        "|               STM32F446RET6 Main Controller                    |\r\n"
        "+================================================================+\r\n"
        "| System Clock: 168MHz         | Memory: 512KB Flash/128KB RAM  |\r\n"
        "| Motor Control: TIM1 PWM      | Encoders: TIM2/TIM3 Quadrature |\r\n"  
        "| IMU Sensor: (removed)         | LiDAR: UART3 Interface         |\r\n"
        "| Communication: UART4 (BT)    | Debug Port: UART2 (PC)         |\r\n"
        "+================================================================+\r\n"
        "[INFO] System initialization started...\r\n"
        "[INFO] Type '?' for command help\r\n\r\n";
        
    HAL_UART_Transmit(&huart4, (uint8_t*)system_info, strlen(system_info), HAL_MAX_DELAY);
}

/**
  ******************************************************************************
  * @brief  系统状态更新
  * @param  None
  * @retval None
  * 
  * @details
  * 定期更新系统状态，监控关键参数
  ******************************************************************************
  */
static void System_StatusUpdate(void)
{
    static uint32_t last_status_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // 每1秒更新一次状态 (可根据需要调整)
    if (current_time - last_status_update >= 1000) {
        last_status_update = current_time;
        
        // System status check
        if (g_system_state == SYSTEM_STATE_ERROR) {
            HAL_UART_Transmit(&huart4, (uint8_t*)"[WARNING] System in error state\r\n", 33, 100);
        }
    }
}

/**
  ******************************************************************************
  * @brief  系统错误处理
  * @param  error_msg: 错误信息字符串
  * @retval None
  * 
  * @details
  * 统一的错误处理函数，记录错误并进入安全模式
  ******************************************************************************
  */
static void System_ErrorHandler(const char* error_msg)
{
    char full_error_msg[100];
    
    // 设置系统为错误状态
    g_system_state = SYSTEM_STATE_ERROR;
    // 已移除：g_motor_mode, g_pid_control_enabled
    
    // 紧急停止所有电机
    Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
    Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
    
    // 发送错误信息
    snprintf(full_error_msg, sizeof(full_error_msg), 
            "[CRITICAL ERROR] %s\r\n[SYSTEM] Entering safe mode...\r\n", error_msg);
    HAL_UART_Transmit(&huart4, (uint8_t*)full_error_msg, strlen(full_error_msg), HAL_MAX_DELAY);
    
    // 进入无限循环等待复位 (生产环境中可考虑软复位)
    while(1) {
        HAL_Delay(1000);
        // 可以在这里添加错误LED闪烁等提示
    }
}

/**
  ******************************************************************************
  * @brief  发送里程计(ODO)数据到Python端
  * @param  None
  * @retval None
  * 
  * @details
  * 阶段1功能：为PWM标定脚本提供实时速度数据
  * - 发送频率：10Hz (100ms间隔)
  * - 数据格式：ODO,timestamp,left_rps,right_rps,left_count,right_count
  * - 用途：pwm_calibration.py可以收集速度数据建立PWM-速度关系
  * 
  * 数据说明：
  * - left_rps/right_rps: 转速（圈/秒）
  * - left_count/right_count: 编码器累积脉冲数
  * 
  * @note 必须在TIM3(左轮)和TIM2(右轮)编码器启动后才能调用
  ******************************************************************************
  */
static void send_odometry_data(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_odo_time < ODO_SEND_INTERVAL) return;
    last_odo_time = now;
    
    float left_rps = 0.0f;
    float right_rps = 0.0f;
    
    // ⭐ Motor_Command API已经在后台运行PID控制
    // 直接使用Motor.c提供的速度（已经通过PID实时更新）
    Motor_GetActualSpeed(&left_rps, &right_rps);
    
    // 应用方向（前进为正，后退为负）
    if (target_direction < 0) {
        // 后退模式：速度取反
        left_rps = -left_rps;
        right_rps = -right_rps;
    }
    
    // 已移除手动PWM模式代码（改用Motor_Command统一管理）
    if (0) {  // 保留代码结构但不执行
        // 手动PWM模式（TURN命令等）：直接读取编码器计数
        // 此模式下编码器不会被PID清零，可以正常累积
        
        // 读取编码器计数
        int32_t left_count = (int32_t)TIM3->CNT;   // 左轮：TIM3
        int32_t right_count = (int32_t)TIM2->CNT;  // 右轮：TIM2
        
        // 计算速度（RPS - 圈/秒）
        static int32_t last_left_count = 0;
        static int32_t last_right_count = 0;
        static uint8_t first_run = 1;
        
        if (!first_run) {
            float dt = ODO_SEND_INTERVAL / 1000.0f;  // 转换为秒 (0.1秒)
            
            // 计算脉冲数变化
            int32_t left_delta = left_count - last_left_count;
            int32_t right_delta = right_count - last_right_count;
            
            // 转换为RPS（圈/秒）
            // 左轮: 1560 PPR（正常方向）
            left_rps = left_delta / (LEFT_ENCODER_PPR * dt);
            
            // 右轮: 780 PPR（方向相反，需取反）
            // 说明：右轮编码器前进时计数为负，需要取反以匹配左轮方向
            // 目标：前进时两轮速度都为正，后退时都为负
            right_rps = -right_delta / (RIGHT_ENCODER_PPR * dt);
        }
        
        // 更新历史值
        last_left_count = left_count;
        last_right_count = right_count;
        first_run = 0;
    }
    
    // 读取编码器累积计数（用于里程计）
    int32_t left_count = (int32_t)TIM3->CNT;   // 左轮：TIM3
    int32_t right_count = (int32_t)TIM2->CNT;  // 右轮：TIM2
    
    // 🔧 修复：右轮取反（与rps_right一致）
    right_count = -right_count;
   
    // 发送ODO数据（CSV格式）
    // 格式：ODO,timestamp,left_rps,right_rps,left_count,right_count
    char odo_msg[100];
    snprintf(odo_msg, sizeof(odo_msg), 
            "ODO,%lu,%.2f,%.2f,%ld,%ld\r\n",
            now, left_rps, right_rps, (long)left_count, (long)right_count);
    
    HAL_UART_Transmit(&huart4, (uint8_t*)odo_msg, strlen(odo_msg), 100);
}

// 重定向 printf 到 USART2（调试输出）
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* ============================================================================ */
/*                   阶段1: PYTHON命令接收与解析函数                            */
/* ============================================================================ */

/**
  ******************************************************************************
  * @brief  UART接收完成中断回调函数
  * @param  huart: UART句柄指针
  * @retval None
  * 
  * @details
  * 此函数在每接收到一个字节后被自动调用：
  * - 逐字节构建命令字符串
  * - 检测换行符'\n'作为命令结束标志
  * - 设置命令完成标志供主循环处理
  * - 自动重启接收下一个字节
  ******************************************************************************
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        uint8_t received_byte = g_uart_rx_buffer;
        
        // 换行符表示命令结束
        if (received_byte == '\n') {
            rx_command_buffer[rx_cmd_index] = '\0';  // 字符串结束符
            rx_cmd_complete = 1;                      // 设置完成标志
            rx_cmd_index = 0;                         // 重置索引
        }
        // 回车符忽略（Windows系统会发送\r\n）
        else if (received_byte == '\r') {
            // 跳过，等待\n
        }
        // 正常字符：存入缓冲区
        else if (rx_cmd_index < RX_BUFFER_SIZE - 1) {
            rx_command_buffer[rx_cmd_index++] = received_byte;
        }
        // 缓冲区溢出保护 - 强制清空并报告
        else {
            // 发送警告
            const char* error_msg = "[ERROR] Buffer overflow - clearing\r\n";
            HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
            
            // 强制清空缓冲区
            rx_cmd_index = 0;
            rx_cmd_complete = 0;
            memset(rx_command_buffer, 0, RX_BUFFER_SIZE);
        }
        
        // ⭐ 关键修复：无论如何都要重启接收
        HAL_StatusTypeDef status = HAL_UART_Receive_IT(&huart4, &g_uart_rx_buffer, 1);
        
        // 如果重启失败，发送错误消息（调试用）
        if (status != HAL_OK) {
            const char* error_msg = "[ERROR] UART RX restart failed\r\n";
            HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
        }
    }
    else if (huart->Instance == UART5) {
        uint8_t received_byte = g_uart5_rx_buffer;
        // 检查缓冲区是否已满
        if (uart5_rx_index < UART5_RX_BUFFER_SIZE - 1) {
            uart5_rx_data[uart5_rx_index++] = received_byte;

            // 如果还没找到包头，搜索0x55 0x53
            if (!uart5_header_found) {
                // 从当前位置向前搜索包头
                for (int i = 0; i <= uart5_rx_index - 2; i++) {
                    if (uart5_rx_data[i] == 0x55 && uart5_rx_data[i + 1] == 0x53) {
                        uart5_header_found = 1;
                        uart5_packet_start = i;
                        // 如果包头不在开头，移动数据到缓冲区开头
                        if (i > 0) {
                            memmove(uart5_rx_data, &uart5_rx_data[i], uart5_rx_index - i);
                            uart5_rx_index = uart5_rx_index - i;
                            uart5_packet_start = 0;
                        }
                        break;
                    }
                }
            }

            // 如果已找到包头，检查是否接收到完整数据包（11字节）
            if (uart5_header_found && (uart5_rx_index - uart5_packet_start) >= 11) {
                // 验证包头
                if (uart5_rx_data[uart5_packet_start] == 0x55 &&
                    uart5_rx_data[uart5_packet_start + 1] == 0x53) {

                    // 计算校验和
                    uint8_t calculated_sum = 0;
                    for (int i = 0; i < 10; i++) {  // 前10个字节
                        calculated_sum += uart5_rx_data[uart5_packet_start + i];
                    }
                    uint8_t received_sum = uart5_rx_data[uart5_packet_start + 10];

                    // 校验和正确，处理数据包
                    if (calculated_sum == received_sum) {
                        uart5_packet_ready = 1;  // 设置数据包接收完成标志

                        // 调试信息
                        //const char* debug_msg = "[UART5] Valid packet received\r\n";
                        //HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
                    } else {
                        // 校验和错误，重置搜索
                        uart5_header_found = 0;
                        uart5_rx_index = 0;
                    }
                } else {
                    // 包头验证失败，重置搜索
                    uart5_header_found = 0;
                    uart5_rx_index = 0;
                }
            }

            // 如果缓冲区接近满且还没找到完整包，重置搜索
            if (uart5_rx_index >= UART5_RX_BUFFER_SIZE - 5) {
                uart5_header_found = 0;
                uart5_rx_index = 0;
            }
        } else {
            // 缓冲区溢出，重置所有状态
            uart5_rx_index = 0;
            uart5_header_found = 0;
            uart5_packet_start = 0;
        }

        // 重启接收
        HAL_UART_Receive_IT(&huart5, &g_uart5_rx_buffer, 1);
    }
}

/**
 * @brief UART错误回调函数
 * @note 阶段1新增：处理UART错误，自动恢复
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        // 发送错误消息
        const char* error_msg = "[ERROR] UART4 error occurred - recovering\r\n";
        HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
        
        // 清除错误标志（STM32F4兼容版本）
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_PE | UART_FLAG_FE);
        
        // 清除错误状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;
        
        // 重新启动接收
        HAL_UART_Receive_IT(&huart4, &g_uart_rx_buffer, 1);
    }
    else if (huart->Instance == UART5) {
        // 发送错误消息
        //const char* error_msg = "[ERROR] UART5 error occurred - recovering\r\n";
        //HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100)

        // 清除错误标志
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_ORE | UART_FLAG_NE | UART_FLAG_PE | UART_FLAG_FE);

        // 清除错误状态
        huart->ErrorCode = HAL_UART_ERROR_NONE;

        // 重置接收状态
        uart5_rx_index = 0;
        uart5_packet_ready = 0;

        // 重新启动接收
        HAL_UART_Receive_IT(&huart5, &g_uart5_rx_buffer, 1);
    }
}

/**
  ******************************************************************************
  * @brief  解析并执行Python端发送的命令
  * @param  None
  * @retval None
  * 
  * @details
  * 阶段1支持的命令：
  * - MODE,0  : 停止机器人
  * - MODE,1  : PID前进模式
  * - MODE,2  : PID后退模式
  * - MODE,3  : 左转模式
  * - MODE,4  : 右转模式
  * 
  * 此函数在主循环中被周期性调用，检查是否有完整命令待处理
  ******************************************************************************
  */
static void Python_ParseCommand(void)
{
    // 检查是否有完整命令
    if (!rx_cmd_complete) {
        return;
    }
    
    // 通信看门狗已移除
    
    // 打印接收到的命令（调试用）- 立即反馈给Python端
    char debug_msg[128];
    snprintf(debug_msg, sizeof(debug_msg), "[Python CMD] Received: %s\r\n", rx_command_buffer);
    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
    
    // 阶段1验证：发送确认消息（确保Python端能看到）
    const char* ack_msg = "ACK\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)ack_msg, strlen(ack_msg), 100);
    
    // ========== CMD命令解析（带序列号，阶段3） ==========
    if (strncmp(rx_command_buffer, "CMD,", 4) == 0) {
        uint16_t seq = 0;
        uint8_t mode = 0;
        
        // 调试：显示接收到的原始命令
        char raw_debug[128];
        snprintf(raw_debug, sizeof(raw_debug), 
                "[DEBUG] Raw buffer: '%s' (len=%d)\r\n", rx_command_buffer, strlen(rx_command_buffer));
        HAL_UART_Transmit(&huart4, (uint8_t*)raw_debug, strlen(raw_debug), 100);
        
        // ⭐ 手动解析（避免sscanf问题）
        char *ptr = rx_command_buffer + 4;  // 跳过"CMD,"

        // 解析seq
        seq = (uint16_t)atoi(ptr);

        // 跳到下一个逗号
        ptr = strchr(ptr, ',');
        if (ptr != NULL) {
            ptr++;  // 跳过逗号
            mode = (uint8_t)atoi(ptr);
        }
        
        // 调试：显示解析结果
        char parse_debug[80];
        snprintf(parse_debug, sizeof(parse_debug), 
                "[DEBUG] Manual parse: seq=%u mode=%u\r\n", seq, mode);
        HAL_UART_Transmit(&huart4, (uint8_t*)parse_debug, strlen(parse_debug), 100);
        
        // 重复命令检测
        if (seq == g_last_command_seq && g_action_in_progress) {
            char warn[64];
            snprintf(warn, sizeof(warn), "[WARN] Duplicate CMD seq=%u, ignored\r\n", seq);
            HAL_UART_Transmit(&huart4, (uint8_t*)warn, strlen(warn), 100);
            
            // 清除命令标志
            rx_cmd_complete = 0;
            rx_cmd_index = 0;
            return;
        }

        // 更新序列号和动作状态
        g_last_command_seq = seq;
        g_action_in_progress = 1;
        g_action_start_time = HAL_GetTick();

        char response[80];
        snprintf(response, sizeof(response), "[CMD] seq=%u mode=%u -> ", seq, mode);

            // 执行新的Motor_Command API
            switch(mode) {
                case 0:  // 停止
                    Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
                    Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
                    snprintf(response + strlen(response), sizeof(response) - strlen(response), "STOP\r\n");
                    
                    // 停止命令立即完成
                    g_action_in_progress = 0;
                    char stop_ack[32];
                    snprintf(stop_ack, sizeof(stop_ack), "ACK,%u\r\n", seq);
                    HAL_UART_Transmit(&huart4, (uint8_t*)stop_ack, strlen(stop_ack), 100);
                    break;
                
                case 1:  // PID前进 600mm
                    Motor_Command_MoveForward(60.0f, 30000);  // 前进600mm，超时30秒
                    snprintf(response + strlen(response), sizeof(response) - strlen(response), 
                            "FORWARD 600mm started\r\n");
                    break;
                
                case 2:  // PID后退 600mm
                    Motor_Command_MoveForward(-60.0f, 30000);  // 后退600mm，超时30秒
                    snprintf(response + strlen(response), sizeof(response) - strlen(response), 
                            "BACKWARD 600mm started\r\n");
                    break;
                
                case 3:  // 左转90度
                    Motor_Command_TurnLeft(90.0f, 10000);  // 左转90度，超时10秒
                    snprintf(response + strlen(response), sizeof(response) - strlen(response), 
                            "TURN_LEFT 90deg started\r\n");
                    break;
                
                case 4:  // 右转90度
                    Motor_Command_TurnRight(90.0f, 10000);  // 右转90度，超时10秒
                    snprintf(response + strlen(response), sizeof(response) - strlen(response), 
                            "TURN_RIGHT 90deg started\r\n");
                    break;
                
                default:
                    snprintf(response + strlen(response), sizeof(response) - strlen(response),
                            "Unknown mode: %u\r\n", mode);
                    g_action_in_progress = 0;  // 无效命令不算动作
                    break;
            }
            
            HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
        }
        else {
            const char* error = "[ERROR] CMD parse failed\r\n";
            HAL_UART_Transmit(&huart4, (uint8_t*)error, strlen(error), 100);
        }
    //}
//    // ========== MODE命令解析（保留兼容性） ==========
//    else if (strncmp(rx_command_buffer, "MODE,", 5) == 0) {
//        int mode = atoi(rx_command_buffer + 5);
//
//        char response[80];
//        snprintf(response, sizeof(response), "[Python CMD] MODE,%d -> ", mode);
//
//        switch(mode) {
//
//            default:
//                snprintf(response + strlen(response), sizeof(response) - strlen(response),
//                        "Unknown mode: %d\r\n", mode);
//        }
//
//        HAL_UART_Transmit(&huart4, (uint8_t*)response, strlen(response), 100);
//    }
//    // ========== 单字符命令支持（'A'雷达扫描等） ==========
//    else if (strlen(rx_command_buffer) == 1) {
//        // 单字符命令，直接调用原有处理函数
//        uint8_t single_char = rx_command_buffer[0];
//
//        char msg[64];
//        snprintf(msg, sizeof(msg), "[Python CMD] Single char: '%c'\r\n", single_char);
//        HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
//
//        // 调用原有的单字符命令处理
//        Command_ProcessUserInput(single_char);
//
//        // 如果是雷达命令，扫描后清空可能堆积的缓冲区
//        if (single_char == 'A' || single_char == 'a') {
//            // 雷达扫描可能阻塞了几秒，期间收到的命令可能不完整
//            // 清空缓冲区，避免解析错误的命令
//            rx_cmd_index = 0;
//            rx_cmd_complete = 0;
//            memset(rx_command_buffer, 0, RX_BUFFER_SIZE);
//
//            // 通信看门狗已移除
//
//            HAL_UART_Transmit(&huart4, (uint8_t*)"[DEBUG] Radar scan done, buffer cleared\r\n", 41, 100);
//        }
//    }
//    // ========== 其他命令（阶段2-5会添加） ==========
//    else {
//        char error_msg[128];
//        snprintf(error_msg, sizeof(error_msg),
//                "[Python CMD] Unknown command: %s\r\n", rx_command_buffer);
//        HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
//    }
    
    // 清除完成标志，准备接收下一个命令
    rx_cmd_complete = 0;
}
/**
  ******************************************************************************
  * @brief  处理UART5传感器数据
  * @param  None
  * @retval None
  *
  * @details
  * 检查是否有完整的数据包需要处理，如果有则调用解析函数
  ******************************************************************************
  */
static void UART5_ProcessSensorData(void)
{
    if (uart5_packet_ready) {
        // 解析数据包（从包头开始的位置）
        UART5_ParseDataPacket(&uart5_rx_data[uart5_packet_start], 11);

        // 重置状态
        uart5_packet_ready = 0;
        uart5_header_found = 0;
        uart5_rx_index = 0;
        uart5_packet_start = 0;
    }
}

/**
  ******************************************************************************
  * @brief  解析UART5传感器数据包
  * @param  data: 数据包指针
  * @param  length: 数据包长度
  * @retval None
  *
  * @details
  * 数据包格式：0x55 0x53 RollL RollH PitchL PitchH YawL YawH VL VH SUM
  * 校验和：SUM = 0x55 + 0x53 + RollH + RollL + PitchH + PitchL + YawH + YawL + VH + VL
  * 偏航角：Z = ((YawH<<8)|YawL)/32768*180(°)
  ******************************************************************************
  */
static void UART5_ParseDataPacket(uint8_t* data, uint16_t length)
{
    // 检查数据包长度和包头
    if (length < 11 || data[0] != 0x55 || data[1] != 0x53) {
        return;  // 不是我们要处理的数据包
    }

    // 计算校验和
    uint8_t calculated_sum = 0;
    for (int i = 0; i < 10; i++) {  // 前10个字节
        calculated_sum += data[i];
    }
    uint8_t received_sum = data[10];

    // 校验和验证
    if (calculated_sum != received_sum) {
        char error_msg[80];
        snprintf(error_msg, sizeof(error_msg),
                "[UART5] Checksum error: calc=0x%02X, recv=0x%02X\r\n",
                calculated_sum, received_sum);
        HAL_UART_Transmit(&huart4, (uint8_t*)error_msg, strlen(error_msg), 100);
        return;
    }

    // 提取YawL和YawH数据（索引6和7）
    uint8_t yaw_l = data[6];  // YawL
    uint8_t yaw_h = data[7];  // YawH

    // 拼接成16位数据（小端序：低字节在前）
    g_yaw_data = (int16_t)((yaw_h << 8) | yaw_l);

    // 计算偏航角：Z = ((YawH<<8)|YawL)/32768*180(°)
    g_yaw_angle = (float)g_yaw_data / 32768.0f * 180.0f;

    // 更新电机控制模块的偏航角数据
   Motor_UpdateYawAngle(g_yaw_angle);

    // 发送解析结果
//    char debug_msg[120];
//    snprintf(debug_msg, sizeof(debug_msg),
//            "[UART5] Yaw: %d (0x%04X), Angle: %.2f°\r\n",
//            g_yaw_data, (unsigned int)g_yaw_data, g_yaw_angle);
//    HAL_UART_Transmit(&huart4, (uint8_t*)debug_msg, strlen(debug_msg), 100);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void){
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
