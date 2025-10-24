/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Smart Car Main Control Program
  * @version        : v1.0.0
  * @date           : 2025-01-XX
  ******************************************************************************
  * @description
  * 
  * This program implements a smart car control system based on STM32F446RET6, integrating the following functional modules:
  * 
  * ┌─────────────────────────────────────────────────────────────────┐
  * │                    Smart Car System Architecture                    │
  * ├─────────────────────────────────────────────────────────────────┤
  * │ • Motor Control System (TIM1 PWM + PID Speed Control)          │
  * │ • Encoder Feedback System (TIM2/TIM3 Quadrature Encoders)      │
  * │ • (Removed) MPU6500 Attitude Sensor                            │
  * │ • LiDAR System (UART3 Communication)                          │
  * │ • Bluetooth Communication Module (UART4 Wireless Control)     │
  * │ • Debug Serial Port (UART2 Data Output)                        │
  * └─────────────────────────────────────────────────────────────────┘
  * 
  * @hardware_connections
  * ┌─────────────┬──────────────┬─────────────────────────────────┐
  * │   Module     │   STM32 Pin   │           Description           │
  * ├─────────────┼──────────────┼─────────────────────────────────┤
  * │ Left Motor PWM│ PA8 (TIM1_CH1)│ L298N Driver PWM Input          │
  * │ Right Motor PWM│ PA10(TIM1_CH3)│ L298N Driver PWM Input         │
  * │ Left Encoder │ PA15/PB3     │ TIM3 Quadrature Encoder Input   │
  * │ Right Encoder│ PA0/PA1      │ TIM2 Quadrature Encoder Input   │
  * │ I2C1         │ PB8/PB9      │ Available I2C Bus (No IMU)     │
  * │ LiDAR        │ PC10/PC5     │ UART3 Radar Communication       │
  * │ Bluetooth    │ PA0/PA1      │ UART4 Wireless Control          │
  * │ Debug Serial │ PA2/PA3      │ UART2 PC Debug Interface        │
  * └─────────────┴──────────────┴─────────────────────────────────┘
  * 
  * @author         : Ye Jin
  * @organization   : Group 12
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // Phase 1: Add atoi function support
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
  * @defgroup PERIPHERAL_HANDLES Peripheral Handle Definitions
  * @brief All peripheral handles used in the system
  * @{
  */
UART_HandleTypeDef huart2;  ///< USART2: PC debug serial port (PA2-TX, PA3-RX)
UART_HandleTypeDef huart3;  ///< USART3: LiDAR communication (PC10-TX, PC5-RX)  
UART_HandleTypeDef huart4;  ///< USART4: Bluetooth wireless control (PA0-TX, PA1-RX)
UART_HandleTypeDef huart5;  ///< UART5: Sensor data reception (PC12-TX, PD2-RX)
TIM_HandleTypeDef htim1;    ///< TIM1: Motor PWM output timer (PA8, PA10)
TIM_HandleTypeDef htim2;    ///< TIM2: Right wheel encoder timer (PA0, PA1)
TIM_HandleTypeDef htim3;    ///< TIM3: Left wheel encoder timer (PA15, PB3)
TIM_HandleTypeDef htim6;    ///< TIM6: System timer (interrupt service)
I2C_HandleTypeDef hi2c1;    ///< I2C1: Available bus (currently no IMU)
/**
  * @}
  */

/**
  * @defgroup COMMUNICATION_BUFFERS Communication Buffers
  * @brief Data buffers for various communication interfaces
  * @{
  */
uint8_t g_uart_rx_buffer;          ///< Bluetooth command receive buffer (single byte)
uint8_t g_debug_rx_buffer;         ///< Debug serial port receive buffer  
uint8_t g_radar_rx_buffer;         ///< Radar data receive buffer
uint8_t g_uart5_rx_buffer;         ///< UART5 sensor data receive buffer (single byte)

// Python command receive buffer (Phase 1 addition)
#define RX_BUFFER_SIZE 256
char rx_command_buffer[RX_BUFFER_SIZE];     ///< Python command buffer
volatile uint8_t rx_cmd_index = 0;          ///< Command buffer index
volatile uint8_t rx_cmd_complete = 0;       ///< Command receive completion flag
uint8_t CTL = 0;       ///< Command receive completion flag
// UART5 sensor data packet parsing
#define UART5_RX_BUFFER_SIZE 64             ///< UART5 receive buffer size
uint8_t uart5_rx_data[UART5_RX_BUFFER_SIZE]; ///< UART5 data receive buffer
volatile uint16_t uart5_rx_index = 0;       ///< UART5 receive buffer index
volatile uint8_t uart5_packet_ready = 0;    ///< UART5 data packet receive completion flag
volatile uint8_t uart5_header_found = 0;    ///< UART5 packet header detection flag
volatile uint16_t uart5_packet_start = 0;   ///< UART5 data packet start position
int16_t g_yaw_data = 0;                     ///< Parsed Yaw data (YawL + YawH)
float g_yaw_angle = 0.0f;                   ///< Parsed yaw angle (degrees)


// ODO data transmission (Phase 1: PWM calibration support)
uint32_t last_odo_time = 0;                 ///< Last ODO data transmission time
#define ODO_SEND_INTERVAL 100               ///< ODO data transmission interval (100ms = 10Hz)
#define LEFT_ENCODER_PPR   1560             ///< Left wheel encoder pulses per revolution
#define RIGHT_ENCODER_PPR  1560              ///< Right wheel encoder pulses per revolution

/**
  * @}
  */

/**
  * @defgroup SYSTEM_STATE_VARIABLES System State Variables
  * @brief System running state and control parameters
  * @{
  */
static SystemState_t g_system_state = SYSTEM_STATE_INIT;
// Removed: g_motor_mode, g_pid_control_enabled (now using Motor_Command_XXX API)
static float g_target_speed_rps = DEFAULT_TARGET_SPEED;

// Command sequence number mechanism (Phase 3: maze exploration)
static uint16_t g_last_command_seq = 0;      // Last received command sequence number
static uint8_t g_action_in_progress = 0;     // Action in progress flag
static uint32_t g_action_start_time = 0;     // Action start time (ms)

// Removed MPU6500 related global variables
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
  MX_GPIO_Init();           // GPIO configuration initialization
  MX_I2C1_Init();           // I2C1 bus initialization
  MX_I2C2_Init();           // I2C2 bus initialization (backup)
  MX_SPI1_Init();           // SPI1 interface initialization (backup)
  MX_TIM1_Init();           // Motor PWM timer initialization
  MX_TIM2_Init();           // Right wheel encoder timer initialization
  MX_TIM3_Init();           // Left wheel encoder timer initialization
  MX_TIM6_Init();           // System reference timer initialization
  MX_UART4_Init();          // Bluetooth communication serial port initialization
  MX_USART2_UART_Init();    // Debug serial port initialization
  MX_USART3_UART_Init();    // LiDAR serial port initialization
  MX_UART5_Init();          // Sensor data serial port initialization
  /* ========================================================================== */
  /*                        FUNCTIONAL MODULES SETUP                           */
  /* ========================================================================== */
  
  // Start encoder timers (quadrature encoder mode)
  if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK ||
      HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK) {
      System_ErrorHandler("Encoder timer startup failed");
  }

  // Initialize motor control system
  Motor_Init(&htim1);
  
  // Initialize PID controller
  Motor_PID_Init();

  // Initialize LiDAR module (continuous scanning mode)
  if (Radar_Init(&huart3) != 0) {
      System_ErrorHandler("Radar module initialization failed");
  }
  
  // Send radar initialization success message
  const char* radar_msg = "[INFO] Radar continuous scan mode initialized\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)radar_msg, strlen(radar_msg), 100);
  
  // System startup prompt
  Command_ShowSystemInfo();
  
  /* ======================================================================== */
  /*                    PYTHON COMMAND INTERFACE SETUP (阶段1)                 */
  /* ======================================================================== */
  
  // Send ready message to Python side
  const char* ready_msg = "STM32 Ready - Waiting for Python commands...\r\n";
  HAL_UART_Transmit(&huart4, (uint8_t*)ready_msg, strlen(ready_msg), 100);
  
  // Start UART4 receive interrupt (for receiving Python commands)
  HAL_UART_Receive_IT(&huart4, &g_uart_rx_buffer, 1);
  // Start UART5 receive interrupt (for receiving sensor data)
  HAL_StatusTypeDef uart5_status = HAL_UART_Receive_IT(&huart5, &g_uart5_rx_buffer, 1);

  // Send UART5 initialization status
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
  static PackPoint_t points_buf[LIDAR_MAX_POINTS]; // Static allocation to avoid stack overflow
  
  while (1)
  {
      loop_counter++;
      
      // ⭐ Step 1: Process radar data in ring buffer (state machine + sliding window parsing)
      LIDAR_ProcessPendingData();
      
      // ⭐ Step 2: If complete rotation data is available and it's time to send, pack and send to host computer
      // New solution: Send one rotation every 500ms (2Hz frequency), raw 5-byte data pass-through
      uint16_t n = LIDAR_GetCompletedRotation((LidarPoint_t*)points_buf, LIDAR_MAX_POINTS);
      if (n > 0 && LIDAR_ShouldSendFrame()) {
          // Use new raw data pass-through API
          int r = LIDAR_PackAndSendRawFrame(points_buf, n);
          if (r != 0) {
              // Send failed to start (UART busy or other error), try again next loop
              // r == -1: UART busy
              // r == -3: Zero points (theoretically won't reach here)
          }
      }
      
      // System status update
      System_StatusUpdate();
//      // Process UART5 sensor data
      UART5_ProcessSensorData();
//      // PID control loop (execute every 20ms)
      Motor_PID_Control(&htim1, &htim3, &htim2);
//      // Phase 1: Process Python commands (from interrupt buffer)
     Python_ParseCommand();
//
      // ========== Phase 3: Action completion detection and ACK reply ==========
      // Note: Motor_Command_XXX functions are non-blocking and will automatically stop when target is reached
      // We detect action completion by checking if speed is close to 0
      if (g_action_in_progress) {
          uint32_t elapsed = HAL_GetTick() - g_action_start_time;
          float left_rps, right_rps;
          Motor_GetActualSpeed(&left_rps, &right_rps);
          
          // Action completion detection: speed close to 0 (entering PID dead zone)
          float speed_threshold = 0.08f;  // RPS, smaller threshold ensures true stop
          uint32_t min_time = 1000;  // Wait at least 1 second to avoid misjudgment
          
          // Detect action completion: time > 1 second and speed close to 0
          if (elapsed > min_time && 
              fabsf(left_rps) < speed_threshold && 
              fabsf(right_rps) < speed_threshold) {
              
              // Debug: Display sequence number about to be sent
              char pre_ack_debug[64];
              snprintf(pre_ack_debug, sizeof(pre_ack_debug), 
                      "[DEBUG] Sending ACK: g_last_command_seq=%u\r\n", g_last_command_seq);
              HAL_UART_Transmit(&huart4, (uint8_t*)pre_ack_debug, strlen(pre_ack_debug), 100);
              
              // Send completion reply: ACK,seq\n
              char ack[48];
              snprintf(ack, sizeof(ack), "ACK,%u,OK,%.2fs\r\n", 
                      g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)ack, strlen(ack), 100);
              
              g_action_in_progress = 0;
              
              // Debug log
              char log[64];
              snprintf(log, sizeof(log), "[DEBUG] Action completed: seq=%u time=%.2fs\r\n",
                      g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)log, strlen(log), 100);
          }
          
          // Timeout protection (35 seconds, give Motor_Command enough time)
          if (elapsed > 35000) {
              char timeout_msg[64];
              snprintf(timeout_msg, sizeof(timeout_msg), 
                      "ACK,%u,TIMEOUT,%.2fs\r\n", g_last_command_seq, elapsed / 1000.0f);
              HAL_UART_Transmit(&huart4, (uint8_t*)timeout_msg, strlen(timeout_msg), 100);
              g_action_in_progress = 0;
              
              // Force stop motors
              Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
              Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
          }
      }
//
      // Phase 1: Send odometry data (10Hz, provide speed data for PWM calibration)
      //send_odometry_data();
  }
}

/* ============================================================================ */
/*                         PRIVATE FUNCTION DEFINITIONS                        */
/* ============================================================================ */

/**
  ******************************************************************************
  * @brief  Process user command input
  * @param  command: Received command character
  * @retval None
  * 
  * @details
  * Command categories:
  * - Motion control: 0-9 (stop, forward, turn, etc.)
  * - Speed adjustment: +/- (increase/decrease target speed)
  * - System test: E,Q,L,R,D,P,p,Z (encoder test)
  * - Sensor: (IMU related commands removed)
  * - Radar: A,a,r,B (LiDAR operations)
  * - Debug: ?,H,C (help and diagnostics)
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

        case '5': // Check current speed
            {
                char speed_report[120];
                
                // Read speed directly from Motor.c (Motor_Command API already running in background)
                Motor_GetActualSpeed(&current_speed_left, &current_speed_right);
                snprintf(speed_report, sizeof(speed_report), 
                        "[SPEED] L: %.2f RPS | R: %.2f RPS | Target: %.1f RPS\r\n",
                        current_speed_left, current_speed_right, g_target_speed_rps);
                
                HAL_UART_Transmit(&huart2, (uint8_t*)speed_report, strlen(speed_report), 100);
            }
            break;
            
        case '?': // Display help information
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
  * @brief  Display system information and startup prompt
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
  * @brief  System status update
  * @param  None
  * @retval None
  * 
  * @details
  * Regularly update system status and monitor key parameters
  ******************************************************************************
  */
static void System_StatusUpdate(void)
{
    static uint32_t last_status_update = 0;
    uint32_t current_time = HAL_GetTick();
    
    // Update status every 1 second (adjustable as needed)
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
  * @brief  System error handling
  * @param  error_msg: Error message string
  * @retval None
  * 
  * @details
  * Unified error handling function, record errors and enter safe mode
  ******************************************************************************
  */
static void System_ErrorHandler(const char* error_msg)
{
    char full_error_msg[100];
    
    // Set system to error state
    g_system_state = SYSTEM_STATE_ERROR;
    // Removed: g_motor_mode, g_pid_control_enabled
    
    // Emergency stop all motors
    Motor_SetSpeed(&htim1, MOTOR_LEFT, 0.0f);
    Motor_SetSpeed(&htim1, MOTOR_RIGHT, 0.0f);
    
    // Send error message
    snprintf(full_error_msg, sizeof(full_error_msg), 
            "[CRITICAL ERROR] %s\r\n[SYSTEM] Entering safe mode...\r\n", error_msg);
    HAL_UART_Transmit(&huart4, (uint8_t*)full_error_msg, strlen(full_error_msg), HAL_MAX_DELAY);
    
    // Enter infinite loop waiting for reset (consider soft reset in production)
    while(1) {
        HAL_Delay(1000);
        // Can add error LED blinking and other prompts here
    }
}

/**
  ******************************************************************************
  * @brief  Send odometry (ODO) data to Python side
  * @param  None
  * @retval None
  * 
  * @details
  * Phase 1 function: Provide real-time speed data for PWM calibration script
  * - Transmission frequency: 10Hz (100ms interval)
  * - Data format: ODO,timestamp,left_rps,right_rps,left_count,right_count
  * - Purpose: pwm_calibration.py can collect speed data to establish PWM-speed relationship
  * 
  * Data description:
  * - left_rps/right_rps: Rotation speed (revolutions per second)
  * - left_count/right_count: Encoder accumulated pulse count
  * 
  * @note Must be called after TIM3 (left wheel) and TIM2 (right wheel) encoders are started
  ******************************************************************************
  */
static void send_odometry_data(void)
{
    uint32_t now = HAL_GetTick();
    if (now - last_odo_time < ODO_SEND_INTERVAL) return;
    last_odo_time = now;
    
    float left_rps = 0.0f;
    float right_rps = 0.0f;
    
    // ⭐ Motor_Command API already running PID control in background
    // Directly use speed from Motor.c (already updated in real-time through PID)
    Motor_GetActualSpeed(&left_rps, &right_rps);
    
    // Apply direction (forward is positive, backward is negative)
    if (target_direction < 0) {
        // Backward mode: invert speed
        left_rps = -left_rps;
        right_rps = -right_rps;
    }
    
    // Removed manual PWM mode code (now using Motor_Command unified management)
    if (0) {  // 保留代码结构但不执行
        // Manual PWM mode (TURN commands, etc.): directly read encoder count
        // In this mode encoder will not be cleared by PID, can accumulate normally
        
        // Read encoder count
        int32_t left_count = (int32_t)TIM3->CNT;   // 左轮：TIM3
        int32_t right_count = (int32_t)TIM2->CNT;  // 右轮：TIM2
        
        // Calculate speed (RPS - revolutions per second)
        static int32_t last_left_count = 0;
        static int32_t last_right_count = 0;
        static uint8_t first_run = 1;
        
        if (!first_run) {
            float dt = ODO_SEND_INTERVAL / 1000.0f;  // Convert to seconds (0.1 second)
            
            // Calculate pulse count change
            int32_t left_delta = left_count - last_left_count;
            int32_t right_delta = right_count - last_right_count;
            
            // Convert to RPS (revolutions per second)
            // Left wheel: 1560 PPR (normal direction)
            left_rps = left_delta / (LEFT_ENCODER_PPR * dt);
            
            // Right wheel: 780 PPR (opposite direction, need to invert)
            // Note: Right wheel encoder counts negative when moving forward, need to invert to match left wheel direction
            // Goal: Both wheels positive when forward, both negative when backward
            right_rps = -right_delta / (RIGHT_ENCODER_PPR * dt);
        }
        
        // Update historical values
        last_left_count = left_count;
        last_right_count = right_count;
        first_run = 0;
    }
    
    // Read encoder accumulated count (for odometry)
    int32_t left_count = (int32_t)TIM3->CNT;   // 左轮：TIM3
    int32_t right_count = (int32_t)TIM2->CNT;  // 右轮：TIM2
    
    // 🔧 Fix: Right wheel invert (consistent with rps_right)
    right_count = -right_count;
   
    // Send ODO data (CSV format)
    // Format: ODO,timestamp,left_rps,right_rps,left_count,right_count
    char odo_msg[100];
    snprintf(odo_msg, sizeof(odo_msg), 
            "ODO,%lu,%.2f,%.2f,%ld,%ld\r\n",
            now, left_rps, right_rps, (long)left_count, (long)right_count);
    
    HAL_UART_Transmit(&huart4, (uint8_t*)odo_msg, strlen(odo_msg), 100);
}

// Redirect printf to USART2 (debug output)
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
/* ============================================================================ */
/* ============================================================================ */

/**
  ******************************************************************************
  * @brief  UART receive completion interrupt callback function
  * @param  huart: UART handle pointer
  * @retval None
  * 
  * @details
  * This function is automatically called after each byte is received:
  * - Build command string byte by byte
  * - Detect newline character '\n' as command end flag
  * - Set command completion flag for main loop processing
  * - Automatically restart receiving next byte
  ******************************************************************************
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4) {
        uint8_t received_byte = g_uart_rx_buffer;
        
        // Newline character indicates command end
        if (received_byte == '\n') {
            rx_command_buffer[rx_cmd_index] = '\0';  // String terminator
            rx_cmd_complete = 1;                      // Set completion flag
            rx_cmd_index = 0;                         // Reset index
        }
        // Carriage return ignored (Windows system sends \r\n)
        else if (received_byte == '\r') {
            // Skip, wait for \n
        }
        // Normal character: store in buffer
        else if (rx_cmd_index < RX_BUFFER_SIZE - 1) {
            rx_command_buffer[rx_cmd_index++] = received_byte;
        }
        // Buffer overflow protection - force clear and report
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
