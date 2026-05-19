/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for DHT22 + OLED + MQ135 + ESP01
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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fonts.h"
#include "ssd1306.h"
#include "dht22.h"
#include "mq135.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_1

/*
 * MQ135 load-resistor value in kΩ (measured from 3362P trimpot).
 * Adjust if you turn the trimpot.
 */
#define MQ135_RL_KOHM   2.6f

/*
 * Number of ADC samples to average during R0 calibration.
 * More samples = slower but more stable calibration.
 */
#define MQ135_CAL_SAMPLES  50
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* Task Handles */
osThreadId mq135TaskHandle;
osThreadId dht22TaskHandle;
osThreadId displayTaskHandle;
osThreadId telemetryTaskHandle;

/* --- RTOS Sync Objects --- */
osSemaphoreId sem_mq135_start;
osSemaphoreId sem_3v3_start;
osSemaphoreId sem_mq135_done;
osSemaphoreId sem_dht_done;
osSemaphoreId sem_display_start;
osSemaphoreId sem_tx_start;
osSemaphoreId sem_display_done;
osSemaphoreId sem_tx_done;

/* Bug 2 fix: mutex protecting shared sensor data buffers */
osMutexId  mtx_sensor_data;

static DHT22_Data_t        dht_data;
static MQ135_GasReadings_t gas_data;
static char                msg[40];     /* Scratch buffer for display strings */
static uint8_t             oled_ok = 0;

/* Bug 5 fix: RTC backup register index used to persist R0 across Stop-mode cycles.
 * BKP_DR1 holds the integer part (kΩ × 10), BKP_DR2 is a magic-number sentinel. */
#define R0_BKP_MAGIC  0xA5A5u
#define R0_BKP_DR_VAL RTC_BKP_DR1   /* stores R0 × 10 as an integer */
#define R0_BKP_DR_MGC RTC_BKP_DR2   /* magic sentinel */

/*
 * Binary frame sent to ESP-01 via UART3.
 * Layout (34 bytes total):
 *   [0]       SOF  = 0xAA
 *   [1]       LEN  = sizeof(SensorPayload_t) = 30
 *   [2..31]   SensorPayload_t  (7 floats × 4 + uint16_t × 2 = 30 bytes)
 *   [32]      CRC  = XOR of bytes [0..31]
 *   [33]      EOF  = 0x55
 *
 * sizeof(SensorPayload_t) breakdown:
 *   float temperature 4
 *   float humidity    4
 *   float co2         4
 *   float nh3         4
 *   float co          4
 *   float alcohol     4
 *   float toluene     4
 *   uint16_t adc_raw  2
 *   ─────────────────────
 *   Total payload    30 bytes → FRAME_TOTAL = 2 + 30 + 2 = 34
 */
#define FRAME_SOF  0xAAu
#define FRAME_EOF  0x55u

#pragma pack(push, 1)
typedef struct {
    float    temperature;
    float    humidity;
    float    co2;
    float    nh4;   /* Bug 1 fix: renamed nh3→nh4 to match MQ135_GasReadings_t field */
    float    co;
    float    alcohol;
    float    toluene;
    uint16_t adc_raw;
} SensorPayload_t;
#pragma pack(pop)

/* 2(header) + 30(payload) + 2(crc+eof) = 34 bytes */
#define FRAME_TOTAL  (2u + sizeof(SensorPayload_t) + 2u)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */
void MQ135Task(void const * argument);
void DHT22Task(void const * argument);
void DisplayTask(void const * argument);
void TelemetryTask(void const * argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* Stack overflow hook — required by configCHECK_FOR_STACK_OVERFLOW = 2 */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  /* Transmit task name directly — printf may not be safe here */
  HAL_UART_Transmit(&huart1, (uint8_t *)"[FATAL] Stack overflow: ", 24, 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)pcTaskName, strlen(pcTaskName), 100);
  HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, 100);
  __disable_irq();
  for (;;) { }
}

/* Retarget printf to USART1 (HW-417C debug) */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
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
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  /* ---- Initialise sensor drivers (Handles only) ---- */
  DHT22_Init(DHT22_PORT, DHT22_PIN, &htim1);
  MQ135_Init(&hadc2, MQ135_RL_KOHM);

  printf("\r\n=== DHT22 + MQ135 + OLED + ESP01 (RTOS + Deep Sleep) ===\r\n");
  printf("System Initialized. Entering RTOS Scheduler...\r\n");
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* Bug 2 fix: create mutex for shared dht_data / gas_data */
  osMutexDef(mtx_sensor_data);
  mtx_sensor_data = osMutexCreate(osMutex(mtx_sensor_data));
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* Binary semaphores: created with count=1, then immediately taken to start at 0 */
  osSemaphoreDef(sem_mq135_start);  sem_mq135_start  = osSemaphoreCreate(osSemaphore(sem_mq135_start),  1); osSemaphoreWait(sem_mq135_start,  0);
  osSemaphoreDef(sem_3v3_start);    sem_3v3_start    = osSemaphoreCreate(osSemaphore(sem_3v3_start),    1); osSemaphoreWait(sem_3v3_start,    0);
  osSemaphoreDef(sem_mq135_done);   sem_mq135_done   = osSemaphoreCreate(osSemaphore(sem_mq135_done),   1); osSemaphoreWait(sem_mq135_done,   0);
  osSemaphoreDef(sem_dht_done);     sem_dht_done     = osSemaphoreCreate(osSemaphore(sem_dht_done),     1); osSemaphoreWait(sem_dht_done,     0);
  osSemaphoreDef(sem_display_start);sem_display_start= osSemaphoreCreate(osSemaphore(sem_display_start),1); osSemaphoreWait(sem_display_start,0);
  osSemaphoreDef(sem_tx_start);     sem_tx_start     = osSemaphoreCreate(osSemaphore(sem_tx_start),     1); osSemaphoreWait(sem_tx_start,     0);
  osSemaphoreDef(sem_display_done); sem_display_done = osSemaphoreCreate(osSemaphore(sem_display_done), 1); osSemaphoreWait(sem_display_done, 0);
  osSemaphoreDef(sem_tx_done);      sem_tx_done      = osSemaphoreCreate(osSemaphore(sem_tx_done),      1); osSemaphoreWait(sem_tx_done,      0);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  osThreadDef(mq135Task,    MQ135Task,    osPriorityNormal,      0, 384);
  mq135TaskHandle    = osThreadCreate(osThread(mq135Task),    NULL);

  osThreadDef(dht22Task,    DHT22Task,    osPriorityNormal,      0, 384);
  dht22TaskHandle    = osThreadCreate(osThread(dht22Task),    NULL);

  osThreadDef(displayTask,  DisplayTask,  osPriorityBelowNormal, 0, 384);
  displayTaskHandle  = osThreadCreate(osThread(displayTask),  NULL);

  osThreadDef(telemetryTask,TelemetryTask,osPriorityNormal,      0, 384);
  telemetryTaskHandle= osThreadCreate(osThread(telemetryTask),NULL);
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 240;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief  MQ135Task – waits for PA2 (5V) to be ON, then reads all gas values.
 *         Signals PowerManagerTask when complete via sem_mq135_done.
 */
void MQ135Task(void const * argument)
{
  for (;;)
  {
    /* Block until PowerManagerTask enables 5V rail and signals us */
    osSemaphoreWait(sem_mq135_start, osWaitForever);

    printf("[MQ135] Reading gas sensors...\r\n");

    /* Bug 5 fix: restore R0 from RTC backup register, or calibrate once in clean air.
     * BKP_DR2 acts as a magic-number sentinel to detect first-boot vs. warm wake. */
    if (HAL_RTCEx_BKUPRead(&hrtc, R0_BKP_DR_MGC) == R0_BKP_MAGIC)
    {
      /* Warm wake: restore persisted R0 */
      uint32_t r0_x10 = HAL_RTCEx_BKUPRead(&hrtc, R0_BKP_DR_VAL);
      MQ135_SetR0((float)r0_x10 / 10.0f);
      printf("[MQ135] Restored R0 = %.1f kOhm from backup reg\r\n", (float)r0_x10 / 10.0f);
    }
    else
    {
      /* First boot: calibrate R0 in (assumed) clean air and persist it */
      printf("[MQ135] First boot — calibrating R0 in clean air...\r\n");
      float r0 = MQ135_CalibrateR0(MQ135_CAL_SAMPLES);
      MQ135_SetR0(r0);
      HAL_RTCEx_BKUPWrite(&hrtc, R0_BKP_DR_VAL, (uint32_t)(r0 * 10.0f));
      HAL_RTCEx_BKUPWrite(&hrtc, R0_BKP_DR_MGC, R0_BKP_MAGIC);
      printf("[MQ135] R0 calibrated = %.2f kOhm, saved to backup reg\r\n", r0);
    }

    /* Bug 2 fix: take a snapshot of dht_data under mutex for temperature compensation */
    float t_comp, h_comp;
    osMutexWait(mtx_sensor_data, osWaitForever);
    t_comp = dht_data.temperature;
    h_comp = dht_data.humidity;
    osMutexRelease(mtx_sensor_data);

    /* Use 25°C / 50%RH as safe defaults on first boot before DHT22 has run */
    if (t_comp == 0.0f && h_comp == 0.0f) { t_comp = 25.0f; h_comp = 50.0f; }

    MQ135_GasReadings_t local_gas;
    if (!MQ135_ReadAllGases(&local_gas, t_comp, h_comp))
    {
      printf("[WARN][MQ135] Read failed\r\n");
    }
    else
    {
      osMutexWait(mtx_sensor_data, osWaitForever);
      gas_data = local_gas;
      osMutexRelease(mtx_sensor_data);
      printf("[MQ135] CO2: %.1f ppm  NH3: %.1f ppm  CO: %.1f ppm\r\n",
             local_gas.co2, local_gas.nh4, local_gas.co);
    }

    /* Signal PowerManagerTask that MQ135 reading is done */
    osSemaphoreRelease(sem_mq135_done);
  }
}

/**
 * @brief  DHT22Task – waits for 3.3V rail to be ON, then reads temperature/humidity.
 *         Signals PowerManagerTask when complete via sem_dht_done.
 */
void DHT22Task(void const * argument)
{
  for (;;)
  {
    /* Block until PowerManagerTask enables 3.3V rail */
    osSemaphoreWait(sem_3v3_start, osWaitForever);

    /* DHT22 needs at least 2s after power-on to stabilise */
    osDelay(2000);

    printf("[DHT22] Reading temperature & humidity...\r\n");

    /* Bug 2 fix: write into a local buffer, then copy under mutex */
    DHT22_Data_t local_dht;
    if (!DHT22_ReadData(&local_dht))
    {
      printf("[WARN][DHT22] Read failed\r\n");
    }
    else
    {
      osMutexWait(mtx_sensor_data, osWaitForever);
      dht_data = local_dht;
      osMutexRelease(mtx_sensor_data);
      printf("[DHT22] T: %.1f C  H: %.1f %%RH\r\n",
             local_dht.temperature, local_dht.humidity);
    }

    /* Signal PowerManagerTask that DHT22 reading is done */
    osSemaphoreRelease(sem_dht_done);
  }
}

/**
 * @brief  DisplayTask – waits for both sensors to finish, re-inits OLED,
 *         then renders all readings. Signals completion via sem_display_done.
 */
void DisplayTask(void const * argument)
{
  for (;;)
  {
    /* Block until PowerManagerTask has collected all sensor data */
    osSemaphoreWait(sem_display_start, osWaitForever);

    printf("[DISPLAY] Updating OLED...\r\n");

    /* Re-init OLED — it just received power */
    oled_ok = SSD1306_Init();
    if (oled_ok)
    {
      SSD1306_Fill(SSD1306_COLOR_BLACK);

      /* Row 0 – Temperature & Humidity */
      SSD1306_GotoXY(0, 0);
      sprintf(msg, "T:%.1fC H:%.1f%%", dht_data.temperature, dht_data.humidity);
      SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

      /* Row 1 – CO2 */
      SSD1306_GotoXY(0, 12);
      sprintf(msg, "CO2: %.0f ppm", gas_data.co2);
      SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

      /* Row 2 – NH3 */
      SSD1306_GotoXY(0, 24);
      sprintf(msg, "NH3: %.0f ppm", gas_data.nh4);
      SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

      /* Row 3 – CO */
      SSD1306_GotoXY(0, 36);
      sprintf(msg, "CO:  %.1f ppm", gas_data.co);
      SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

      /* Row 4 – Toluene & Acetone */
      SSD1306_GotoXY(0, 48);
      sprintf(msg, "Tol:%.0f Ace:%.0f", gas_data.toluene, gas_data.acetone);
      SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

      SSD1306_UpdateScreen();
    }
    else
    {
      printf("[WARN][DISPLAY] SSD1306 init failed\r\n");
    }

    osSemaphoreRelease(sem_display_done);
  }
}

/**
 * @brief  TelemetryTask – waits for all sensor data to be ready, then prints
 *         readings to debug UART and sends the binary frame to ESP-01.
 *         Signals completion via sem_tx_done.
 */
void TelemetryTask(void const * argument)
{
  for (;;)
  {
    /* Block until PowerManagerTask gives the green light */
    osSemaphoreWait(sem_tx_start, osWaitForever);

    /* Bug 2 fix: take a consistent snapshot of shared data under mutex */
    DHT22_Data_t        snap_dht;
    MQ135_GasReadings_t snap_gas;
    osMutexWait(mtx_sensor_data, osWaitForever);
    snap_dht = dht_data;
    snap_gas = gas_data;
    osMutexRelease(mtx_sensor_data);

    /* ---- Debug print to USART1 ---- */
    printf("--- Sensor Readings ---\r\n");
    printf("DHT22  T: %.1f C   H: %.1f %%RH\r\n",
           snap_dht.temperature, snap_dht.humidity);
    printf("MQ135  ADC: %u   Rs: %.2f kOhm   R0: %.2f kOhm\r\n",
           snap_gas.adc_raw, snap_gas.rs, MQ135_GetR0());
    printf("  CO2:     %.1f ppm\r\n", snap_gas.co2);
    printf("  NH3:     %.1f ppm\r\n", snap_gas.nh4);
    printf("  CO:      %.1f ppm\r\n", snap_gas.co);
    printf("  Alcohol: %.1f ppm\r\n", snap_gas.alcohol);
    printf("  Toluene: %.1f ppm\r\n", snap_gas.toluene);
    printf("-----------------------\r\n");

    /* ---- Send binary frame to ESP-01 via USART3 ---- */
    uint8_t frame[FRAME_TOTAL];
    uint8_t idx = 0;
    SensorPayload_t p;
    p.temperature = snap_dht.temperature;
    p.humidity    = snap_dht.humidity;
    p.co2         = snap_gas.co2;
    p.nh4         = snap_gas.nh4;  /* Bug 1 fix: field renamed nh3→nh4 */
    p.co          = snap_gas.co;
    p.alcohol     = snap_gas.alcohol;
    p.toluene     = snap_gas.toluene;
    p.adc_raw     = snap_gas.adc_raw;
    frame[idx++] = FRAME_SOF;
    frame[idx++] = (uint8_t)sizeof(SensorPayload_t);
    memcpy(&frame[idx], &p, sizeof(SensorPayload_t));
    idx += sizeof(SensorPayload_t);
    uint8_t crc = 0;
    for (uint8_t i = 0; i < idx; i++) crc ^= frame[i];
    frame[idx++] = crc;
    frame[idx++] = FRAME_EOF;
    HAL_UART_Transmit(&huart3, frame, (uint16_t)idx, 100);
    printf("[TELEMETRY] Frame sent to ESP-01 (%u bytes)\r\n", (unsigned)idx);

    osSemaphoreRelease(sem_tx_done);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
  /*
   * PowerManagerTask – the master orchestrator.
   *
   * Cycle (every ~5 minutes):
   *  1. Turn ON 5V  (PA2) → signal MQ135Task to start reading
   *  2. Wait 20 s for MQ135 to heat up
   *  3. Turn ON 3.3V (PA3) → signal DHT22Task to start reading
   *  4. Wait for MQ135Task  done  (sem_mq135_done)
   *  5. Wait for DHT22Task  done  (sem_dht_done)
   *  6. Signal DisplayTask  (sem_display_start)
   *  7. Signal TelemetryTask (sem_tx_start)
   *  8. Wait for both to finish
   *  9. Cut power to all modules (PA2 + PA3 LOW)
   * 10. Arm RTC alarm for T+5 min → enter Stop mode → sleep
   * 11. RTC fires → wake → restore clocks → repeat
   */
  for (;;)
  {
    /* ── STEP 1: Enable 5V rail for MQ135 ── */
    printf("\r\n[PWR] Cycle start — enabling 5V (MQ135)\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);

    /* Signal MQ135Task that it can proceed (power is up) */
    osSemaphoreRelease(sem_mq135_start);

    /* ── STEP 2: Wait 20 s for MQ135 heater ── */
    osDelay(20000);

    /* ── STEP 3: Enable 3.3V rail (OLED, DHT22, ESP-01) ── */
    printf("[PWR] Enabling 3.3V (DHT22 / OLED / ESP-01)\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);

    /* Signal DHT22Task that power is on (it will delay internally) */
    osSemaphoreRelease(sem_3v3_start);

    /* ── STEP 4 & 5: Wait for both sensor tasks ── */
    printf("[PWR] Waiting for sensor tasks...\r\n");
    osSemaphoreWait(sem_mq135_done, osWaitForever);
    osSemaphoreWait(sem_dht_done,   osWaitForever);
    printf("[PWR] All sensors ready.\r\n");

    /* ── STEP 6 & 7: Signal output tasks ── */
    osSemaphoreRelease(sem_display_start);
    osSemaphoreRelease(sem_tx_start);

    /* ── STEP 8: Wait for output tasks ── */
    osSemaphoreWait(sem_display_done, osWaitForever);
    osSemaphoreWait(sem_tx_done,      osWaitForever);

    /* ── STEP 9: Cut power to all modules ── */
    printf("[PWR] Cutting power to all modules.\r\n");
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2 | GPIO_PIN_3, GPIO_PIN_RESET);

    /* ── STEP 10: Arm RTC alarm for T+5 min ── */
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN); /* Must read date to unlock shadow regs */

    RTC_AlarmTypeDef sAlarm = {0};
    sAlarm.AlarmTime.Hours   = sTime.Hours;
    sAlarm.AlarmTime.Minutes = sTime.Minutes + 5;
    sAlarm.AlarmTime.Seconds = sTime.Seconds;
    if (sAlarm.AlarmTime.Minutes >= 60) {
      sAlarm.AlarmTime.Minutes -= 60;
      sAlarm.AlarmTime.Hours    = (sAlarm.AlarmTime.Hours + 1) % 24;
    }
    sAlarm.Alarm = RTC_ALARM_A;
    if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BIN) != HAL_OK) {
      printf("[ERR][PWR] RTC Alarm set failed!\r\n");
    }

    printf("[PWR] Entering Stop Mode (5-min sleep)... ZZZ\r\n");

    /* Bug 3 fix: suspend the FreeRTOS scheduler and SysTick before entering
     * Stop mode so no other task or tick ISR fires while the CPU is halted.
     * On wake, resume the scheduler before any RTOS API call is made.
     * NOTE: This is the safest pragmatic approach on CMSIS-RTOS v1 / FreeRTOS
     * without a dedicated low-power port. All tasks simply see a longer osDelay
     * period, which is acceptable for a 5-minute duty cycle. */
    vTaskSuspendAll();
    HAL_SuspendTick();

    /* ── STEP 10 cont.: Enter Stop mode — CPU halts here ── */
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);

    /* ════════════════════════════════════════════════════
     *  <<< RTC ALARM FIRES — CPU RESUMES HERE >>>
     * ════════════════════════════════════════════════════ */

    /* ── STEP 11: Restore PLL clock (Stop mode falls back to HSI) ── */
    SystemClock_Config();
    HAL_ResumeTick();
    xTaskResumeAll();
    printf("[PWR] Woke from Stop mode. Starting new cycle.\r\n");
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
