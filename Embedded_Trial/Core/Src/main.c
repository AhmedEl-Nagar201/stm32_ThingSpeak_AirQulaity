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
TIM_HandleTypeDef htim1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
static DHT22_Data_t        dht_data;
static MQ135_GasReadings_t gas_data;
static char                msg[40];     /* Scratch buffer for display strings */
static uint8_t             oled_ok = 0;

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
    float    nh3;
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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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

/**
 * @brief  Update the OLED with the latest sensor readings.
 */
static void UpdateOLED(void)
{
  if (!oled_ok) return;

  SSD1306_Fill(SSD1306_COLOR_BLACK);

  /* Row 0 – Temperature & Humidity */
  SSD1306_GotoXY(0, 0);
  sprintf(msg, "T:%.1fC H:%.1f%%", dht_data.temperature, dht_data.humidity);
  SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

  /* Row 1 – CO2 */
  SSD1306_GotoXY(0, 12);
  sprintf(msg, "CO2: %.0f ppm", gas_data.co2);
  SSD1306_Puts(msg, &Font_7x10, SSD1306_COLOR_WHITE);

  /* Row 2 – NH3 (Ammonia) */
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

/**
 * @brief  Print all sensor readings to the debug UART.
 */
static void PrintReadings(void)
{
  printf("--- Sensor Readings ---\r\n");
  printf("DHT22  T: %.1f C   H: %.1f %%RH\r\n",
         dht_data.temperature, dht_data.humidity);
  printf("MQ135  ADC: %u   Rs: %.2f kOhm   Rs/R0: %.3f   R0: %.2f kOhm\r\n",
         gas_data.adc_raw, gas_data.rs, gas_data.rs_ro, MQ135_GetR0());
  printf("  CO2:     %.1f ppm\r\n", gas_data.co2);
  printf("  NH3:     %.1f ppm\r\n", gas_data.nh4);
  printf("  CO:      %.1f ppm\r\n", gas_data.co);
  printf("  Alcohol: %.1f ppm\r\n", gas_data.alcohol);
  printf("  Toluene: %.1f ppm\r\n", gas_data.toluene);
  printf("  Acetone: %.1f ppm\r\n", gas_data.acetone);
  printf("-----------------------\r\n");
}

/**
 * @brief  Build and transmit a 32-byte binary sensor frame to ESP-01 (UART3).
 *
 * Frame layout:
 *   [0]      0xAA  (SOF)
 *   [1]      payload length
 *   [2..N-2] SensorPayload_t
 *   [N-2]    XOR checksum of all preceding bytes
 *   [N-1]    0x55  (EOF)
 */
static void SendTelemetry(void)
{
  uint8_t frame[FRAME_TOTAL];
  uint8_t idx = 0;

  /* Fill payload struct */
  SensorPayload_t p;
  p.temperature = dht_data.temperature;
  p.humidity    = dht_data.humidity;
  p.co2         = gas_data.co2;
  p.nh3         = gas_data.nh4;
  p.co          = gas_data.co;
  p.alcohol     = gas_data.alcohol;
  p.toluene     = gas_data.toluene;
  p.adc_raw     = gas_data.adc_raw;

  /* Build frame */
  frame[idx++] = FRAME_SOF;
  frame[idx++] = (uint8_t)sizeof(SensorPayload_t);
  memcpy(&frame[idx], &p, sizeof(SensorPayload_t));
  idx += sizeof(SensorPayload_t);

  /* XOR checksum over all bytes so far */
  uint8_t crc = 0;
  for (uint8_t i = 0; i < idx; i++)
    crc ^= frame[i];
  frame[idx++] = crc;
  frame[idx++] = FRAME_EOF;

  HAL_UART_Transmit(&huart3, frame, (uint16_t)idx, 50);
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
  HAL_Init();
  SystemClock_Config();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC2_Init();
  MX_USART3_UART_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();

  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);

  /* ---- Initialise sensor drivers ---- */
  DHT22_Init(DHT22_PORT, DHT22_PIN, &htim1);
  MQ135_Init(&hadc2, MQ135_RL_KOHM);

  /* Small power-on delay for OLED capacitor charge */
  HAL_Delay(100);

  oled_ok = SSD1306_Init();
  if (!oled_ok) {
    printf("\r\n[WARN] SSD1306 not found on I2C (addr=0x78). Check wiring!\r\n");
  } else {
    SSD1306_Fill(SSD1306_COLOR_BLACK);
    SSD1306_GotoXY(10, 10);
    SSD1306_Puts("Air Quality", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(10, 26);
    SSD1306_Puts("Monitor v2.0", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_GotoXY(10, 42);
    SSD1306_Puts("Calibrating...", &Font_7x10, SSD1306_COLOR_WHITE);
    SSD1306_UpdateScreen();
  }

  printf("\r\n=== DHT22 + MQ135 + OLED + ESP01 ===\r\n");
  printf("USART1 @115200 -> HW-417C (PA9 TX / PA10 RX)\r\n");
  printf("USART3 @115200 -> ESP-01  (PB10 TX / PB11 RX)\r\n");
  printf("I2C1   @400kHz -> SSD1306 (PB6 SCL / PB7 SDA)\r\n");
  printf("ADC2 CH5       -> MQ135   (PA5)\r\n");
  printf("DHT22          -> PA1\r\n");
  printf("====================================\r\n");

  /* ---- Calibrate MQ135 R0 in clean air ----
   * NOTE: For accurate results, run this in clean outdoor air after
   * the sensor has preheated for 24+ hours on first use.
   * Once you know your R0, you can hardcode it with MQ135_SetR0()
   * instead of calibrating every boot.
   */
  printf("[MQ135] Calibrating R0 (%d samples)...\r\n", MQ135_CAL_SAMPLES);
  float r0 = MQ135_CalibrateR0(MQ135_CAL_SAMPLES);
  if (r0 > 0.0f) {
    printf("[MQ135] R0 = %.2f kOhm (calibrated)\r\n", r0);
  } else {
    printf("[MQ135] Calibration FAILED. Using default R0.\r\n");
    MQ135_SetR0(10.0f);  /* Fallback default */
  }

  /* Wait for DHT22 to stabilise after power-on */
  HAL_Delay(2000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    /* ---- 1. Read DHT22 ---- */
    if (!DHT22_ReadData(&dht_data)) {
      printf("DHT22: Read failed\r\n");
    }

    /* ---- 2. Read MQ135 (all gases, with temp/humidity compensation) ---- */
    if (!MQ135_ReadAllGases(&gas_data, dht_data.temperature, dht_data.humidity)) {
      printf("MQ135: Read failed\r\n");
    }

    /* ---- 3. Output results ---- */
    PrintReadings();
    UpdateOLED();
    SendTelemetry();

    /* DHT22 requires minimum 2 s between readings */
    HAL_Delay(2000);
  }
  /* USER CODE END 3 */
}

/* -------------------------------------------------------------------------- */
/* System Clock Configuration (unchanged)                                    */
/* -------------------------------------------------------------------------- */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators */
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

  /** Initializes the CPU, AHB and APB buses clocks */
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* -------------------------------------------------------------------------- */
/* Peripheral Initialization Functions (from CubeMX, kept as is)             */
/* -------------------------------------------------------------------------- */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};
  hadc2.Instance = ADC2;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
    Error_Handler();
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;  /* Long sample for MQ135 source impedance */
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
    Error_Handler();
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    Error_Handler();
}

static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    Error_Handler();
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    Error_Handler();
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    Error_Handler();
}

static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
    Error_Handler();
}

static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
    Error_Handler();
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* PA1 = DHT22 input (start as default input with pull-up) */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* PB2 = Output (e.g., onboard LED or other) */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
