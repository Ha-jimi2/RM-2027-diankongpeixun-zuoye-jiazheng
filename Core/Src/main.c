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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
  const float *seq;
  uint8_t      len;
  uint8_t      idx;
  uint32_t     tick;
  uint8_t      started;
  uint8_t      done;
} Seq_t;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t      pin;
  uint8_t       raw;
  uint8_t       stable;
  uint32_t      t_change;
} Key_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_CCR_MIN    550u
#define SERVO_CCR_MAX    2450u
#define SERVO_CCR_MID    1500u

#define SEQ_LEN          5u
#define SEQ_INTERVAL_MS  1000u

#define KEY_STEP_DEG     15.0f
#define KEY_DEBOUNCE_MS  20u
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float g_angle = 90.0f;

static const float g_seq[SEQ_LEN] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f};
static Seq_t g_sched = { g_seq, SEQ_LEN, 0, 0, 0, 0 };

static float g_shown_deg = -1.0f;

static Key_t g_key_add = { KEY_ADD_GPIO_Port, KEY_ADD_Pin,  1, 1, 0 };
static Key_t g_key_sub = { KEY_SUB_GPIO_Port, KEY_SUB_Pin,  1, 1, 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static uint16_t Servo_AngleToCCR(float angle)
{
  float ccr;

  if (angle < 0.0f)   angle = 0.0f;
  if (angle > 180.0f) angle = 180.0f;

  ccr = 500.0f + angle / 180.0f * 2000.0f;

  if (ccr < (float)SERVO_CCR_MIN) ccr = (float)SERVO_CCR_MIN;
  if (ccr > (float)SERVO_CCR_MAX) ccr = (float)SERVO_CCR_MAX;

  return (uint16_t)ccr;
}

static void Servo_SetAngle(float angle)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, Servo_AngleToCCR(angle));
}

static void Servo_Init(void)
{
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, SERVO_CCR_MID);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void Servo_ShowUI(float angle)
{
  uint32_t deg = (uint32_t)(angle + 0.5f);
  uint16_t ccr = Servo_AngleToCCR(angle);

  OLED_ShowString(1, 1,  "Target:");
  OLED_ShowNum(1, 9, deg, 3);
  OLED_ShowString(1, 12, "deg");

  OLED_ShowString(2, 1,  "CCR   :");
  OLED_ShowNum(2, 9, ccr, 4);

  OLED_ShowString(3, 1,  "Pulse :");
  OLED_ShowNum(3, 9, ccr / 1000, 1);
  OLED_ShowString(3, 10, ".");
  OLED_ShowNum(3, 11, ccr % 1000, 3);
  OLED_ShowString(3, 14, "ms");
}

static void Servo_Apply(void)
{
  uint32_t deg = (uint32_t)(g_angle + 0.5f);
  uint16_t ccr = Servo_AngleToCCR(g_angle);

  Servo_SetAngle(g_angle);

  printf("Target=%3u  CCR=%4u  Pulse=%u.%03ums\r\n",
         (unsigned)deg, (unsigned)ccr,
         (unsigned)(ccr / 1000), (unsigned)(ccr % 1000));
}

static void Seq_Poll(void)
{
  if (g_sched.done) return;

  if (g_sched.started &&
      (HAL_GetTick() - g_sched.tick) < SEQ_INTERVAL_MS)
    return;

  g_sched.tick    = HAL_GetTick();
  g_sched.started = 1;

  g_angle = g_sched.seq[g_sched.idx];
  Servo_Apply();

  g_sched.idx++;
  if (g_sched.idx >= g_sched.len)
    g_sched.idx = 0;
}

static uint8_t Key_Edge(Key_t *k)
{
  uint8_t raw = (HAL_GPIO_ReadPin(k->port, k->pin) == GPIO_PIN_RESET) ? 0 : 1;

  if (raw != k->raw)
  {
    k->raw = raw;
    k->t_change = HAL_GetTick();
    return 0;
  }

  if (raw != k->stable && (HAL_GetTick() - k->t_change) >= KEY_DEBOUNCE_MS)
  {
    k->stable = raw;
    return (uint8_t)(k->stable == 0);
  }

  return 0;
}

static void Key_Poll(void)
{
  if (Key_Edge(&g_key_add))
  {
    g_sched.done = 1;
    g_angle += KEY_STEP_DEG;
    if (g_angle > 180.0f) g_angle = 180.0f;
    Servo_Apply();
  }

  if (Key_Edge(&g_key_sub))
  {
    g_sched.done = 1;
    g_angle -= KEY_STEP_DEG;
    if (g_angle < 0.0f) g_angle = 0.0f;
    Servo_Apply();
  }
}

static void Ui_Refresh(void)
{
  if (g_shown_deg == g_angle) return;
  g_shown_deg = g_angle;
  Servo_ShowUI(g_angle);
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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  setvbuf(stdout, NULL, _IONBF, 0);

  Servo_Init();

  g_key_add.t_change = HAL_GetTick();
  g_key_sub.t_change = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Seq_Poll();
    Key_Poll();
    Ui_Refresh();
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

/* USER CODE BEGIN 4 */
int _write(int fd, char *ptr, int len)
{
  (void)fd;
  HAL_UART_Transmit(&huart1, (uint8_t *)ptr, (uint16_t)len, 100);
  return len;
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
