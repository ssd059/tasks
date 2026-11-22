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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* 三档速度：z=65%，x=80%，c=98% */
#define SPEED_LEVEL_1 65U
#define SPEED_LEVEL_2 80U
#define SPEED_LEVEL_3 98U

/* 10° 转向参数（履带式双电机，开环：线速度 × 延时）
 * 车体：履带中心距 B ≈ 245 - 33 = 212 mm，驱动轮直径 D = 39 mm，
 *       电机 1:30、额定 333 r/min（12 V 空载）。
 *
 * 1) 原地转向（车停着，a/d 直接转）：
 *    两履带反向，65% 占空比，ω = 2v/B ≈ 182 °/s，
 *    10° 理论 ≈ 55 ms，加起步死区/滑移取 100 ms。
 *
 * 2) 行进中转向（不打断前进/后退）：
 *    外履带 = min(当前档位 + TURN_OUTER_BOOST, 100)，
 *    内履带 = 外履带 - TURN_INNER_DROP，两履带整体抬高，
 *    既保持平均速度、避免内履带因占空比过低而停转拖死，
 *    差速仍为 30 个点：65 档 80/50、80 档 95/65、98 档 100/70。
 *    差速约 30 个点对应 Δv ≈ 0.156 m/s、ω ≈ 42 °/s，10° ≈ 240 ms。
 *    履带打滑较大，实际角度请实测后按 10°/实测角度 缩放 TURN_MOVING_MS。 */
#define TURN_DUTY_PERCENT  65U   /* 原地转向占空比 */
#define TURN_PIVOT_MS      100U  /* 原地转 10° 的时间 */
#define TURN_INNER_DROP    30U   /* 行进转向时内履带比外履带低的点数 */
#define TURN_OUTER_BOOST   15U   /* 行进转向时两履带整体抬高的点数 */
#define TURN_MOVING_MS     240U  /* 行进中转 10° 的时间 */

/* 右电机镜像安装标志：
 *   1 = 右电机与左电机对装（常见小车结构），前进时 IN3/IN4 取反；
 *   0 = 两电机同向安装。
 * 若按 w 后小车原地打转而不是前进，把这里改成 0，
 * 或改为对调右电机接 L298N 的 M+/M- 两根线（两种方法只用一个）。 */
#define MOTOR_B_INVERT 1U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
static uint8_t rx_byte;                  /* 中断接收的单字节缓冲 */
static volatile uint8_t key_ready;       /* 1：收到一个有效单字符命令 */
static volatile char    key_char;        /* 收到的命令字符 */
static uint8_t speed_percent = SPEED_LEVEL_1;  /* 当前速度档位，默认一档 65% */
static uint8_t car_moving;               /* 1：小车正在行驶（用于实时改速度） */
static uint8_t car_dir;                  /* 0=停，1=前进，2=后退 */
static uint8_t turning;                  /* 1：正在执行 10° 转向 */
static uint8_t turn_inner_left;          /* 行进转向时内履带是否为左 */
static uint32_t turn_end_tick;           /* 转向结束的 SysTick 时刻 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
static void UART_SendString(const char *str);
static void LED_On(void);
static void LED_Off(void);
static void Motor_SetSpeed(uint8_t percent);
static void Motor_SetSpeeds(uint8_t left_percent, uint8_t right_percent);
static void Motor_SetLeft(uint8_t forward);
static void Motor_SetRight(uint8_t forward);
static void Car_Forward(void);
static void Car_Backward(void);
static void Car_Turn(char cmd);
static void Car_ApplyTurnSpeeds(void);
static void Car_Stop(void);
static void Car_SetSpeedLevel(uint8_t percent);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/**
  * @brief  串口接收完成回调：单字符遥控
  * @note   w/a/s/d = 前/左/后/右，z/x/c = 65%/80%/98%，
  *         q = 停车，n/f = LED 开/关（f 同时停车）。
  *         收到有效字符后暂停接收，等主循环处理完再重新开启。
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    char c = (char)rx_byte;

    if ((c >= 'A') && (c <= 'Z'))
    {
      c = (char)(c - 'A' + 'a');       /* 兼容大写字母 */
    }

    if ((c == 'w') || (c == 'a') || (c == 's') || (c == 'd') ||
        (c == 'z') || (c == 'x') || (c == 'c') ||
        (c == 'q') || (c == 'n') || (c == 'f'))
    {
      key_char  = c;
      key_ready = 1U;
    }

    if (key_ready == 0U)
    {
      (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
  }
}

/**
  * @brief  串口错误回调：清溢出标志并恢复接收
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    __HAL_UART_CLEAR_OREFLAG(huart);

    if (key_ready == 0U)
    {
      (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
  }
}
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  LED_Off();    /* 上电默认熄灭（PC13 低电平点亮） */

  /* 启动 TIM3 两路 PWM：CH1(PA6)=左电机 ENA，CH2(PA7)=右电机 ENB */
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  Car_Stop();   /* 上电两电机都不转 */

  /* 开启单字节中断接收，等待 JDY-31 转发过来的手机数据 */
  (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* 10° 转向时间到：原地转向则停车；行进中转向则恢复两履带同速，继续前进/后退 */
    if ((turning != 0U) && ((int32_t)(HAL_GetTick() - turn_end_tick) >= 0))
    {
      if (car_dir == 0U)
      {
        Car_Stop();
      }
      else
      {
        turning = 0U;
        Motor_SetSpeed(speed_percent);   /* 转向结束，继续原方向行驶 */
      }
    }

    if (key_ready != 0U)
    {
      switch (key_char)
      {
        case 'w':   /* 前进 */
          Car_Forward();
          UART_SendString("FWD\r\n");
          break;

        case 's':   /* 后退 */
          Car_Backward();
          UART_SendString("BACK\r\n");
          break;

        case 'a':   /* 左转 10°：行驶中只降低内履带速度，不打断前进/后退 */
          Car_Turn('a');
          UART_SendString("LEFT\r\n");
          break;

        case 'd':   /* 右转 10°：行驶中只降低内履带速度，不打断前进/后退 */
          Car_Turn('d');
          UART_SendString("RIGHT\r\n");
          break;

        case 'z':   /* 一档 65% */
          Car_SetSpeedLevel(SPEED_LEVEL_1);
          UART_SendString("SPEED 65%\r\n");
          break;

        case 'x':   /* 二档 80% */
          Car_SetSpeedLevel(SPEED_LEVEL_2);
          UART_SendString("SPEED 80%\r\n");
          break;

        case 'c':   /* 三档 98% */
          Car_SetSpeedLevel(SPEED_LEVEL_3);
          UART_SendString("SPEED 98%\r\n");
          break;

        case 'q':   /* 停车 */
          Car_Stop();
          UART_SendString("STOP\r\n");
          break;

        case 'n':   /* LED 开，不影响电机 */
          LED_On();
          UART_SendString("LED ON\r\n");
          break;

        case 'f':   /* LED 关，同时停车 */
          LED_Off();
          Car_Stop();
          UART_SendString("LED OFF\r\n");
          break;

        default:
          break;
      }

      key_ready = 0U;

      /* 处理完毕，重新开启接收 */
      (void)HAL_UART_Receive_IT(&huart1, &rx_byte, 1U);
    }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3599;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN3_Pin|MOTOR_IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MOTOR_IN1_Pin MOTOR_IN2_Pin MOTOR_IN3_Pin MOTOR_IN4_Pin */
  GPIO_InitStruct.Pin = MOTOR_IN1_Pin|MOTOR_IN2_Pin|MOTOR_IN3_Pin|MOTOR_IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
  * @brief  通过 USART1 发送字符串
  * @param  str 以 '\0' 结尾的字符串
  */
static void UART_SendString(const char *str)
{
  (void)HAL_UART_Transmit(&huart1, (uint8_t *)str,
                         (uint16_t)strlen(str), 100U);
}

/**
  * @brief  点亮板载 LED（Blue Pill 的 PC13 为低电平点亮）
  */
static void LED_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
}

/**
  * @brief  熄灭板载 LED
  */
static void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
}

/**
  * @brief  分别设置左右两路电机 PWM 占空比
  * @param  left_percent  左履带 0~100
  * @param  right_percent 右履带 0~100
  */
static void Motor_SetSpeeds(uint8_t left_percent, uint8_t right_percent)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;

  if (left_percent > 100U)
  {
    left_percent = 100U;
  }
  if (right_percent > 100U)
  {
    right_percent = 100U;
  }

  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
                        (period * left_percent) / 100U);   /* 左 ENA */
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
                        (period * right_percent) / 100U);  /* 右 ENB */
}

/**
  * @brief  左右两路电机使用相同占空比
  * @param  percent 0~100，0 停止、100 全速
  */
static void Motor_SetSpeed(uint8_t percent)
{
  Motor_SetSpeeds(percent, percent);
}

/**
  * @brief  设置左电机方向
  * @param  forward 1 = 正转，0 = 反转
  */
static void Motor_SetLeft(uint8_t forward)
{
  if (forward != 0U)
  {
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  设置右电机方向
  * @param  forward 1 = 车体前进方向，0 = 后退方向
  * @note   右电机镜像安装时，通过 MOTOR_B_INVERT 把电平时序取反
  */
static void Motor_SetRight(uint8_t forward)
{
  uint8_t fwd = (uint8_t)(forward ^ (uint8_t)MOTOR_B_INVERT);

  if (fwd != 0U)
  {
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
  }
  else
  {
    HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_SET);
  }
}

/**
  * @brief  小车前进：两电机按前进方向转动
  */
static void Car_Forward(void)
{
  turning = 0U;              /* 取消可能正在进行的转向 */
  car_dir = 1U;
  Motor_SetSpeed(0U);        /* 先降速，避免带载换向 */
  Motor_SetLeft(1U);
  Motor_SetRight(1U);
  Motor_SetSpeed(speed_percent);
  car_moving = 1U;
}

/**
  * @brief  小车后退：两电机按后退方向转动
  */
static void Car_Backward(void)
{
  turning = 0U;              /* 取消可能正在进行的转向 */
  car_dir = 2U;
  Motor_SetSpeed(0U);
  Motor_SetLeft(0U);
  Motor_SetRight(0U);
  Motor_SetSpeed(speed_percent);
  car_moving = 1U;
}

/**
  * @brief  行进中转向：两履带整体抬高，再让内履带比外履带低 TURN_INNER_DROP 个点
  * @note   65 档 -> 外 80 / 内 50；80 档 -> 外 95 / 内 65；98 档 -> 外 100 / 内 70。
  *         这样左右转向的差速完全对称，同时避免内履带占空比过低而停转、
  *         履带被拖着走导致某一侧转向明显变慢。
  */
static void Car_ApplyTurnSpeeds(void)
{
  uint8_t outer = (uint8_t)(speed_percent + TURN_OUTER_BOOST);
  uint8_t inner;
  uint8_t left;
  uint8_t right;

  if (outer > 100U)
  {
    outer = 100U;
  }

  inner = (uint8_t)(outer - TURN_INNER_DROP);

  left  = outer;
  right = outer;

  if (turn_inner_left != 0U)
  {
    left = inner;
  }
  else
  {
    right = inner;
  }

  Motor_SetSpeeds(left, right);
}

/**
  * @brief  转 10°（非阻塞）
  * @param  cmd 'a' = 左转命令，'d' = 右转命令
  * @note   停车时：两履带反向原地转，转完停车；
  *         行进中：不改变前进/后退方向和外侧速度，只降低内履带速度，
  *         转完自动恢复两履带同速，继续原来的前进/后退。
  */
static void Car_Turn(char cmd)
{
  if (car_dir == 0U)
  {
    /* 原地转向：沿用互换后的 a/d 电机逻辑 */
    if (cmd == 'a')
    {
      Motor_SetLeft(1U);
      Motor_SetRight(0U);
    }
    else
    {
      Motor_SetLeft(0U);
      Motor_SetRight(1U);
    }

    Motor_SetSpeeds(TURN_DUTY_PERCENT, TURN_DUTY_PERCENT);
    turning       = 1U;
    turn_end_tick = HAL_GetTick() + TURN_PIVOT_MS;
    return;
  }

  /* 行进中转向：方向脚不动，只做左右履带差速 */
  if (cmd == 'a')
  {
    /* a 对应原右转（顺时针）：前进时内履带在右，后退时在左 */
    turn_inner_left = (car_dir == 2U) ? 1U : 0U;
  }
  else
  {
    /* d 对应原左转（逆时针）：前进时内履带在左，后退时在右 */
    turn_inner_left = (car_dir == 1U) ? 1U : 0U;
  }

  Car_ApplyTurnSpeeds();

  turning       = 1U;
  turn_end_tick = HAL_GetTick() + TURN_MOVING_MS;
}

/**
  * @brief  停车：占空比 0，四个方向脚全部拉低（自由滑行）
  */
static void Car_Stop(void)
{
  Motor_SetSpeed(0U);
  HAL_GPIO_WritePin(MOTOR_IN1_GPIO_Port, MOTOR_IN1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN2_GPIO_Port, MOTOR_IN2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN3_GPIO_Port, MOTOR_IN3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(MOTOR_IN4_GPIO_Port, MOTOR_IN4_Pin, GPIO_PIN_RESET);
  car_moving = 0U;
  car_dir    = 0U;
  turning    = 0U;
}

/**
  * @brief  切换速度档位
  * @param  percent 0~100
  * @note   直行中切换立即生效；行进转向中保持差速不变、外履带跟随新档；
  *         停车时只记录，下次行驶使用
  */
static void Car_SetSpeedLevel(uint8_t percent)
{
  if (percent > 100U)
  {
    percent = 100U;
  }

  speed_percent = percent;

  if (turning != 0U)
  {
    if (car_dir != 0U)
    {
      Car_ApplyTurnSpeeds();
    }
  }
  else if (car_moving != 0U)
  {
    Motor_SetSpeed(speed_percent);
  }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
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
