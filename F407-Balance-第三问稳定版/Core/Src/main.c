/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Balance_Control.h"
#include "Camera.h"
#include "Motor_Calibration.h"
#include "Motor_Command.h"
#include "Motor_Feedback.h"
#include "OLED_Display.h"
#include "Telemetry.h"
#include "UART_DMA.h"
#include "LineTrack_Control.h"
#include "UI_Menu.h"
#include "tim.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/*
 * ===================== 串口资源与接线约定 =====================
 *
 * 1. USART1：张大头闭环步进电机总线，115200 8N1。
 *    PA9  / USART1_TX -> 电机 RX/R
 *    PA10 / USART1_RX <- 电机 TX/T
 *
 * 2. USART2：摄像头坐标链路，115200 8N1。
 *    PA2 / USART2_TX，PA3 / USART2_RX。
 *
 * 3. USART3：电脑遥测输出，115200 8N1。
 *    PB10 / USART3_TX -> USB-TTL RX；PB11 不参与当前单向遥测。
 *
 * 4. OLED：0.96 英寸 SSD1306 128x64 四针 I2C，400 kHz。
 *    PB6=I2C1_SCL，PB7=I2C1_SDA；TX 使用低优先级 DMA1_Stream7。
 *
 * ===================== 软件分层与启动顺序 =====================
 * CubeMX 层：MX_DMA_Init()、MX_USARTx_UART_Init() 只配置时钟、GPIO、DMA 句柄和中断。
 * BSP 层：Camera/Motor_Feedback 先清空软件状态，UART_DMA_Init() 再启动
 * USART1/2 RX DMA；USART3 TX DMA 由 Telemetry 按 20 Hz 非阻塞发送。
 * APP 层：Motor_Command 统一管理 USART1；Balance_Control 只在 PA0/Watch 安全
  * 门控、零位查询/清零/验证和使能 ACK 全部通过后执行 O 点稳球闭环。
 *
 * 系统时钟：8 MHz HSE / PLLM 4 = 2 MHz，* PLLN 168 = 336 MHz VCO，
 * / PLLP 2 = 168 MHz SYSCLK；APB1=42 MHz，APB2=84 MHz。
 *
  * 初始化阶段不发送运动命令；PA0 已松开且小球在 O 点稳定 3 帧后，才自动按安全
  * 状态机把当前水平位置建为零点、使能电机并进入 O 点稳球。
 */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* 复位原因不旁路视觉、PA0、通信 ACK、零点验证和机械硬限位。 */
  __HAL_RCC_CLEAR_RESET_FLAGS();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  /* 业务解析器必须先清零，再启动 USART1/2 的 ReceiveToIdle DMA。 */
  Camera_Init();
  Motor_FeedbackInit();

  /* 启动电机回包和摄像头两路 RX；USART3 保留给电脑端遥测 TX。 */
  UART_DMA_Init();

  /*
   * Motor_Command 先建立 USART1 唯一所有权和不可变 -340/+330 pulse 硬门。
   * 标定模块的 false 参数明确关闭历史自动清零/自动往返；两个 APP 初始化均只
   * 清状态，不发送任何串口命令。Balance 只有通过 PA0、评分页 PE2 或 Watch
   * 完整门控后才启动。
   */
  Motor_CommandInit();
  Motor_CalibrationInit(false);
  Balance_ControlInit();
  Telemetry_Init();
  OLED_DisplayInit();

  /*
   * 循迹底盘只新增独立 TIM/GPIO/PID 路径，不接管稳定版 UART、I2C、PA0、PE2
   * 或张大头 Motor_Command。初始化时 PWM 保持为 0。
   */
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  LineTrack_Init();
  UI_MenuInit();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /*
     * UART_DMA_Process：监视/补挂 USART1/2 RX DMA；
     * Camera_Process：维护摄像头超时在线状态；
     * Motor_CommandProcess：推进 USART1 TX、ACK、超时和一次重试；
     * UI_MenuProcess：统一扫描 PE3/PE2，短按/长按事件只分发一次；
     * Balance_ControlLoop：确认 PA0 已松开且球稳定在 O 点后，上电自动把当前水平
     * 位置建零并持续闭环回 O 点；PA0、评分页 PE2 和 Watch 保留人工启动入口；
     * Motor_CalibrationProcess：只处理人工 Watch 命令并服从 USART1 所有权。
     * Telemetry_Process：只读控制快照，经 USART3 TX DMA 以 20 Hz 发往电脑。
     * OLED_DisplayProcess：最后只读控制/UI 快照，以 10 Hz 生成 128x64 画面；
     *                     I2C1 DMA 非阻塞发送，故障时自动释放并重建总线。
     * 上述函数均为非阻塞设计，可在裸机 while(1) 中高频轮询。
     */
    UART_DMA_Process();
    Camera_Process();
    Motor_CommandProcess();
    UI_MenuProcess();
    Balance_ControlLoop();
    Motor_CalibrationProcess();
    LineTrack_Process();
    Telemetry_Process();
    OLED_DisplayProcess();
  }
  /* USER CODE END 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
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
