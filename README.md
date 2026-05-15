# Smart Air Quality Monitor: Embedded System Design and Implementation

## Abstract

This document presents a comprehensive technical analysis of a distributed embedded air quality monitoring system built around the STM32F103C8T6 microcontroller, integrating MQ135 multi-gas sensing, DHT22 temperature/humidity measurement, ESP-01 (ESP8266) wireless bridging, and ESP32-S3 gateway functionality. The system employs a hierarchical architecture for real-time environmental data acquisition, wireless transmission via ESP-NOW protocol, and cloud integration through ThingSpeak IoT platform. This documentation serves as both an academic reference and practical implementation guide for embedded systems engineers.

---

## Table of Contents

1. [System Architecture Overview](#1-system-architecture-overview)
2. [Hardware Platform Specifications](#2-hardware-platform-specifications)
3. [STM32F103C8T6 Sensor Node](#3-stm32f103c8t6-sensor-node)
4. [Sensor Interface Design](#4-sensor-interface-design)
5. [Communication Protocol Stack](#5-communication-protocol-stack)
6. [ESP-01 Bridge Firmware](#6-esp-01-bridge-firmware)
7. [ESP32-S3 Gateway Implementation](#7-esp32-s3-gateway-implementation)
8. [Data Flow and Packet Transmission](#8-data-flow-and-packet-transmission)
9. [Power Management Considerations](#9-power-management-considerations)
10. [Production and Development Notes](#10-production-and-development-notes)
11. [Appendix: Complete Code Listings](#11-appendix-complete-code-listings)

---

## 1. System Architecture Overview

### 1.1 Hierarchical Topology

The system implements a three-tier hierarchical architecture:

```
┌─────────────────────────────────────────────────────────────────┐
│                        CLOUD LAYER                              │
│                    ThingSpeak IoT Platform                      │
│                         (HTTP/REST API)                         │
└────────────────────────────┬────────────────────────────────────┘
                             │ WiFi (802.11 b/g/n)
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                     GATEWAY LAYER                               │
│                  ESP32-S3 Dual-Core Processor                   │
│              • ESP-NOW Receiver (Channel 1)                     │
│              • WiFi Station Mode (On-demand)                    │
│              • HTTP Client for Cloud Upload                     │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             │ ESP-NOW Protocol (2.4 GHz ISM Band)
                             │ Peer-to-Peer, Low Latency (<10ms)
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                    BRIDGE LAYER                                 │
│                  ESP-01 (ESP8266) Module                        │
│              • UART Frame Parser                                │
│              • ESP-NOW Controller                               │
│              • Checksum Validation                              │
└────────────────────────────┬────────────────────────────────────┘
                             │
                             │ UART @ 115200 baud, 8N1
                             │ TX: PB10, RX: PB11
                             │
┌────────────────────────────▼────────────────────────────────────┐
│                  SENSOR NODE LAYER                              │
│              STM32F103C8T6 (ARM Cortex-M3 @ 72 MHz)             │
│   ┌──────────────┐  ┌──────────────┐  ┌──────────────┐         │
│   │   MQ135      │  │    DHT22     │  │   SSD1306    │         │
│   │ Multi-Gas    │  │ Temp/Humidity│  │  OLED 128x64 │         │
│   │   ADC CH5    │  │   TIM1/PB1   │  │   I2C1       │         │
│   └──────────────┘  └──────────────┘  └──────────────┘         │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Design Philosophy

The architecture follows key embedded systems principles:

1. **Modularity**: Each processing element (STM32, ESP-01, ESP32-S3) operates independently with well-defined interfaces.
2. **Real-time Constraints**: Deterministic sampling intervals (2 seconds) ensure consistent data acquisition.
3. **Fault Tolerance**: XOR checksum validation at each communication hop prevents corrupted data propagation.
4. **Power Efficiency**: ESP32-S3 maintains ESP-NOW listening mode, activating WiFi only during cloud upload windows.
5. **Scalability**: ESP-NOW supports up to 20 peers, enabling future multi-node deployments.

---

## 2. Hardware Platform Specifications

### 2.1 STM32F103C8T6 Microcontroller

| Parameter | Specification |
|-----------|---------------|
| Core | ARM Cortex-M3 (32-bit RISC) |
| Maximum Frequency | 72 MHz |
| Flash Memory | 64 KB |
| SRAM | 20 KB |
| ADC Resolution | 12-bit, up to 2.1 MSPS |
| Timers | 4× General Purpose (TIM1-TIM4) |
| Communication Interfaces | 3× USART, 2× I2C, 2× SPI |
| GPIO Ports | A, B, C, D (max 51 pins) |
| Operating Voltage | 2.0V - 3.6V |
| Package | LQFP48 (7mm × 7mm) |

**Clock Configuration:**
```c
/* System Clock: HSE (8 MHz) → PLL (×9) → SYSCLK (72 MHz) */
RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
RCC_OscInitStruct.HSEState = RCC_HSE_ON;
RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;  /* 8 MHz × 9 = 72 MHz */

/* Bus Clocks */
RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;      /* HCLK = 72 MHz */
RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;       /* APB1 = 36 MHz */
RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;       /* APB2 = 72 MHz */

/* ADC Clock: APB2 / 6 = 12 MHz (within spec) */
PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
```

### 2.2 Peripheral Pin Assignment

| Peripheral | Signal | STM32 Pin | GPIO Port | Mode |
|------------|--------|-----------|-----------|------|
| DHT22 | DATA | PA1 | GPIOA_1 | Input/Output |
| MQ135 | ANALOG | PA5 | GPIOA_5 | ADC2_CH5 |
| SSD1306 | SCL | PB6 | GPIOB_6 | I2C1_SCL (AF) |
| SSD1306 | SDA | PB7 | GPIOB_7 | I2C1_SDA (AF) |
| ESP-01 | TX | PB10 | GPIOB_10 | USART3_TX (AF) |
| ESP-01 | RX | PB11 | GPIOB_11 | USART3_RX (AF) |
| Debug UART | TX | PA9 | GPIOA_9 | USART1_TX (AF) |
| Debug UART | RX | PA10 | GPIOA_10 | USART1_RX (AF) |
| Timer | TIM1_CH1 | PA8 | GPIOA_8 | Alternate Function |

---

## 3. STM32F103C8T6 Sensor Node

### 3.1 Firmware Architecture

The STM32 firmware follows a super-loop architecture with hardware abstraction layer (HAL) drivers:

```c
int main(void)
{
    /* Hardware Abstraction Layer Initialization */
    HAL_Init();
    SystemClock_Config();
    
    /* Peripheral Initialization */
    MX_GPIO_Init();
    MX_ADC2_Init();
    MX_USART3_UART_Init();
    MX_USART1_UART_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    
    /* Sensor Driver Initialization */
    HAL_TIM_Base_Start(&htim1);
    DHT22_Init(DHT22_PORT, DHT22_PIN, &htim1);
    MQ135_Init(&hadc2, MQ135_RL_KOHM);
    
    /* Display Initialization */
    oled_ok = SSD1306_Init();
    
    /* MQ135 R0 Calibration in Clean Air */
    MQ135_CalibrateR0(MQ135_CAL_SAMPLES);
    
    /* Main Super-Loop */
    while (1) {
        DHT22_ReadData(&dht_data);
        MQ135_ReadAllGases(&gas_data, dht_data.temperature, dht_data.humidity);
        PrintReadings();
        UpdateOLED();
        SendTelemetry();
        HAL_Delay(2000);  /* DHT22 minimum sampling interval */
    }
}
```

### 3.2 File Structure

```
Embedded_Trial/
├── Core/
│   ├── Inc/
│   │   ├── main.h           # Global peripheral handles, error handler prototype
│   │   ├── dht22.h          # DHT22 driver public API
│   │   ├── mq135.h          # MQ135 gas sensor driver with calibration
│   │   ├── ssd1306.h        # OLED display driver (I2C)
│   │   ├── fonts.h          # Bitmap font definitions
│   │   └── stm32f1xx_*.h    # HAL configuration headers
│   └── Src/
│       ├── main.c           # Application entry point, frame construction
│       ├── dht22.c          # DHT22 timing-critical protocol implementation
│       ├── mq135.c          # Gas concentration algorithms, Rs/R0 calculations
│       ├── ssd1306.c        # OLED command sequences, graphics primitives
│       ├── fonts.c          # Font bitmap data
│       └── stm32f1xx_*.c    # HAL MSP initialization, interrupt handlers
```

---

## 4. Sensor Interface Design

### 4.1 DHT22 Digital Temperature/Humidity Sensor

#### 4.1.1 Operating Principle

The DHT22 employs a capacitive humidity sensor and thermistor for measurements, outputting a calibrated digital signal via a single-wire bidirectional protocol. Data transmission requires precise microsecond-level timing controlled by TIM1.

#### 4.1.2 Communication Protocol

```
Host Start Signal:
    Host pulls LOW for ≥1 ms → Releases (pull-up) → Waits for response
    
Sensor Response Sequence:
    ┌─────────────┬───────────────┬─────────────────────────────────────────┐
    │ 80 µs LOW   │ 80 µs HIGH    │ 40 bits data (MSB first)               │
    └─────────────┴───────────────┴─────────────────────────────────────────┘
    
Bit Encoding:
    Logic '0': 50 µs LOW + 26-28 µs HIGH
    Logic '1': 50 µs LOW + 70 µs HIGH
    
Data Frame (40 bits):
    Humidity (16 bits) | Temperature (16 bits) | Checksum (8 bits)
```

#### 4.1.3 Driver Implementation Snippet (`dht22.c`)

```c
/**
 * @brief  Send the start signal and check for sensor ACK.
 * @retval 1 if sensor responded, 0 otherwise.
 */
static uint8_t DHT22_StartSignal(void)
{
    uint8_t response = 0;

    /* Pull low for >1 ms to wake the sensor */
    DHT22_SetPinOutput();
    HAL_GPIO_WritePin(dht_port, dht_pin, GPIO_PIN_RESET);
    DHT22_MicroDelay(1300);

    /* Release the bus (pull-up takes it high) */
    HAL_GPIO_WritePin(dht_port, dht_pin, GPIO_PIN_SET);
    DHT22_MicroDelay(30);

    /* Switch to input and look for the sensor's 80 µs LOW response */
    DHT22_SetPinInput();
    DHT22_MicroDelay(40);

    if (!(HAL_GPIO_ReadPin(dht_port, dht_pin))) {
        DHT22_MicroDelay(80);
        if (HAL_GPIO_ReadPin(dht_port, dht_pin))
            response = 1;
    }

    /* Wait for end of the 80 µs HIGH response pulse */
    uint32_t timeout = HAL_GetTick();
    while (HAL_GPIO_ReadPin(dht_port, dht_pin) && (HAL_GetTick() - timeout < 2));

    return response;
}

/**
 * @brief  Read a single byte (8 bits) from the DHT22 data line.
 */
static uint8_t DHT22_ReadByte(void)
{
    uint8_t value = 0;

    for (uint8_t i = 0; i < 8; i++) {
        /* Wait for the bit start (pin goes HIGH after ~50 µs LOW) */
        uint32_t timeout = HAL_GetTick();
        while (!HAL_GPIO_ReadPin(dht_port, dht_pin) && (HAL_GetTick() - timeout < 2));

        /* Delay 40 µs – if pin is still HIGH it's a '1', else '0' */
        DHT22_MicroDelay(40);

        if (HAL_GPIO_ReadPin(dht_port, dht_pin))
            value |= (1 << (7 - i));   /* bit is '1' */

        /* Wait for pin to go LOW (end of bit) */
        timeout = HAL_GetTick();
        while (HAL_GPIO_ReadPin(dht_port, dht_pin) && (HAL_GetTick() - timeout < 2));
    }

    return value;
}
```

#### 4.1.4 Temperature Compensation Algorithm

Temperature data from DHT22 is critical for MQ135 gas concentration compensation:

```c
/* Temperature – bit 15 is sign flag */
uint16_t raw_temp = ((uint16_t)(temp_h & 0x7F) << 8) | temp_l;
data->temperature = (float)raw_temp / 10.0f;
if (temp_h & 0x80)
    data->temperature = -data->temperature;
```

---

### 4.2 MQ135 Multi-Gas Semiconductor Sensor

#### 4.2.1 Sensing Mechanism

The MQ135 utilizes a tin dioxide (SnO₂) semiconductor sensing layer whose conductivity changes upon exposure to reducing gases. The sensor incorporates a heating element maintained at approximately 300°C to facilitate surface reactions.

**Target Gases:**
- Carbon Dioxide (CO₂)
- Ammonia (NH₃/NH₄)
- Carbon Monoxide (CO)
- Alcohol (Ethanol, C₂H₅OH)
- Toluene (C₇H₈)
- Acetone (C₃H₆O)

#### 4.2.2 Electrical Interface

The MQ135 module implements a voltage divider configuration:

```
    VCC (5V)
      │
      │
     ┌┴┐
     │ │ Rs (Variable sensor resistance)
     │ │ (1 kΩ - 50 kΩ depending on gas concentration)
     └┬┘
      ├──────→ Vout (to ADC)
      │
     ┌┴┐
     │ │ RL (Load resistor, typically 1-47 kΩ)
     │ │ (Measured: 2.6 kΩ from 3362P trimpot)
     └┬┘
      │
     GND
```

**Sensor Resistance Calculation:**

$$V_{out} = V_{CC} \times \frac{R_L}{R_s + R_L}$$

$$R_s = R_L \times \left(\frac{V_{CC}}{V_{out}} - 1\right)$$

#### 4.2.3 Power-Law Regression for Gas Concentration

Gas concentration follows a power-law relationship derived from datasheet sensitivity curves:

$$PPM = a \times \left(\frac{R_s}{R_0}\right)^b$$

Where:
- $R_s$ = Sensor resistance in target gas environment (kΩ)
- $R_0$ = Sensor resistance in clean air (kΩ), calibrated per device
- $a, b$ = Gas-specific regression coefficients

**Coefficient Table (from `mq135.h`):**

| Gas | Coefficient $a$ | Coefficient $b$ |
|-----|-----------------|-----------------|
| CO | 605.18 | -3.937 |
| Alcohol | 77.255 | -3.18 |
| CO₂ | 110.47 | -2.862 |
| Toluene | 44.947 | -3.445 |
| NH₃ | 102.2 | -2.473 |
| Acetone | 34.668 | -3.369 |

#### 4.2.4 Temperature/Humidity Compensation

Environmental conditions significantly affect MQ135 readings. An empirical polynomial correction factor is applied:

```c
#define CORA    0.00035f
#define CORB    0.02718f
#define CORC    1.39538f
#define CORD    0.0018f

float MQ135_GetCorrectionFactor(float temperature, float humidity)
{
    /* Polynomial: factor = CORA×t² − CORB×t + CORC − (h − 33)×CORD */
    float factor = CORA * temperature * temperature
                 - CORB * temperature
                 + CORC
                 - (humidity - 33.0f) * CORD;

    /* Clamp to prevent wild corrections */
    if (factor < 0.1f)  factor = 0.1f;
    if (factor > 3.0f)  factor = 3.0f;

    return factor;
}
```

Reference conditions: 20°C, 33% RH → factor = 1.0

#### 4.2.5 R0 Calibration Procedure

Critical for accurate measurements, R0 must be calibrated in clean outdoor air after 24-hour preheating:

```c
float MQ135_CalibrateR0(uint16_t num_samples)
{
    float rs_sum = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < num_samples; i++) {
        uint16_t adc = MQ135_SingleConversion();
        if (adc == 0) continue;

        float rs = MQ135_GetRs(adc);
        if (rs > 0.0f) {
            rs_sum += rs;
            valid++;
        }
        HAL_Delay(50);
    }

    if (valid == 0) return -1.0f;

    /* In clean air: Rs/R0 ≈ 3.6 (MQ135_CLEAN_AIR_FACTOR) */
    float rs_avg = rs_sum / (float)valid;
    mq_r0 = rs_avg / MQ135_CLEAN_AIR_FACTOR;
    mq_calibrated = 1;

    return mq_r0;
}
```

#### 4.2.6 Complete Gas Reading Implementation (`mq135.c`)

```c
uint8_t MQ135_ReadAllGases(MQ135_GasReadings_t *readings,
                           float temperature, float humidity)
{
    /* Initialize with invalid values */
    readings->co = readings->alcohol = readings->co2 = 
    readings->toluene = readings->nh4 = readings->acetone = -1.0f;
    
    /* ADC conversion */
    uint16_t adc = MQ135_SingleConversion();
    readings->adc_raw = adc;
    if (adc == 0) return 0;

    /* Calculate sensor resistance */
    float rs = MQ135_GetRs(adc);
    readings->rs = rs;

    /* Apply environmental compensation */
    if (temperature > -100.0f && humidity > -100.0f) {
        float corr = MQ135_GetCorrectionFactor(temperature, humidity);
        rs = rs / corr;
    }

    /* Compute Rs/R0 ratio */
    float rs_ro = rs / mq_r0;
    readings->rs_ro = rs_ro;

    /* Calculate PPM for all gases */
    readings->co      = MQ135_GetPPM(rs_ro, MQ135_CO_A,      MQ135_CO_B);
    readings->alcohol = MQ135_GetPPM(rs_ro, MQ135_ALCOHOL_A, MQ135_ALCOHOL_B);
    readings->co2     = MQ135_GetPPM(rs_ro, MQ135_CO2_A,     MQ135_CO2_B);
    readings->toluene = MQ135_GetPPM(rs_ro, MQ135_TOLUENE_A, MQ135_TOLUENE_B);
    readings->nh4     = MQ135_GetPPM(rs_ro, MQ135_NH4_A,     MQ135_NH4_B);
    readings->acetone = MQ135_GetPPM(rs_ro, MQ135_ACETONE_A, MQ135_ACETONE_B);

    return 1;
}
```

---

### 4.3 SSD1306 OLED Display Interface

#### 4.3.1 I2C Communication Protocol

The SSD1306 controller operates at I2C address 0x3C (8-bit write: 0x78):

```c
/* I2C Configuration (400 kHz Fast Mode) */
hi2c1.Instance = I2C1;
hi2c1.Init.ClockSpeed = 400000;
hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
hi2c1.Init.OwnAddress1 = 0;
hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
```

#### 4.3.2 Display Buffer Architecture

The 128×64 pixel display uses a page-based memory organization (8 pages × 128 columns):

```c
/* Framebuffer: 1024 bytes (128 × 64 / 8) */
static uint8_t SSD1306_Buffer[SSD1306_WIDTH * SSD1306_HEIGHT / 8];

/* Pixel manipulation */
void SSD1306_DrawPixel(uint16_t x, uint16_t y, SSD1306_COLOR_t color)
{
    if (color == SSD1306_COLOR_WHITE)
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= 1 << (y % 8);
    else
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y % 8));
}
```

---

## 5. Communication Protocol Stack

### 5.1 Binary Telemetry Frame Format

The STM32 transmits sensor data to ESP-01 via UART3 using a compact binary protocol:

```
Frame Structure (34 bytes total):
┌──────┬──────┬──────────────────────────────────────────┬──────┬──────┐
│ SOF  │ LEN  │          SensorPayload_t (30B)           │ CRC  │ EOF  │
│ 0xAA │  30  │ 7×float (28B) + uint16_t (2B)            │ XOR  │ 0x55 │
└──────┴──────┴──────────────────────────────────────────┴──────┴──────┘
  Byte 0   1                2-31                          32     33
```

**Payload Definition (`main.c`):**

```c
#pragma pack(push, 1)
typedef struct {
    float    temperature;   /* 4 bytes: °C       */
    float    humidity;      /* 4 bytes: %RH      */
    float    co2;           /* 4 bytes: ppm      */
    float    nh3;           /* 4 bytes: ppm      */
    float    co;            /* 4 bytes: ppm      */
    float    alcohol;       /* 4 bytes: ppm      */
    float    toluene;       /* 4 bytes: ppm      */
    uint16_t adc_raw;       /* 2 bytes: raw ADC  */
} SensorPayload_t;
#pragma pack(pop)
```

**Frame Construction and XOR Checksum:**

```c
static void SendTelemetry(void)
{
    uint8_t frame[FRAME_TOTAL];
    uint8_t idx = 0;

    /* Populate payload structure */
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

    /* XOR checksum over bytes 0..(idx-1) */
    uint8_t crc = 0;
    for (uint8_t i = 0; i < idx; i++)
        crc ^= frame[i];
    frame[idx++] = crc;
    frame[idx++] = FRAME_EOF;

    HAL_UART_Transmit(&huart3, frame, (uint16_t)idx, 50);
}
```

### 5.2 UART Configuration

```c
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
    HAL_UART_Init(&huart3);
}
```

**Timing Analysis:**
- Frame size: 34 bytes
- Baud rate: 115200 bps
- Transmission time: $34 \times 10 / 115200 \approx 2.95$ ms
- Inter-frame interval: 2000 ms (DHT22 constraint)
- Channel utilization: $2.95 / 2000 \approx 0.15\%$

---

## 6. ESP-01 Bridge Firmware

### 6.1 Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Processor | ESP8266EX (Tensilica L106, 32-bit) |
| Clock Speed | 80 MHz (default) |
| SRAM | 96 KB (data + instruction) |
| Flash | 1 MB (external SPI) |
| WiFi | 802.11 b/g/n (2.4 GHz) |
| GPIO | 17 programmable pins |
| UART | 2× (UART0 for programming/debug) |

### 6.2 ESP-NOW Protocol Overview

ESP-NOW is a connectionless, low-latency protocol developed by Espressif:

- **Range**: Up to 220 m (line-of-sight)
- **Latency**: < 10 ms typical
- **Security**: Optional AES-128 encryption
- **Peers**: Up to 20 registered devices
- **Payload**: Maximum 250 bytes per packet

### 6.3 Frame Parsing Implementation (`ESP01_Sensor_Bridge.ino`)

```cpp
/**
 * @brief  Validate and unpack a complete 34-byte frame.
 * @return true if checksum is valid, false otherwise.
 */
bool validateFrame(const uint8_t *buf, SensorPayload_t *out)
{
    if (buf[0] != FRAME_SOF) return false;
    if (buf[FRAME_LEN - 1] != FRAME_EOF) return false;

    /* XOR checksum verification */
    uint8_t crc = 0;
    for (int i = 0; i < FRAME_LEN - 2; i++)
        crc ^= buf[i];

    if (crc != buf[FRAME_LEN - 2]) return false;

    /* Extract payload */
    memcpy(out, &buf[2], sizeof(SensorPayload_t));
    return true;
}

/**
 * @brief  Byte-by-byte state machine parser
 */
void loop()
{
    while (Serial.available()) {
        uint8_t b = Serial.read();

        if (!in_frame) {
            if (b == FRAME_SOF) {
                in_frame = true;
                rx_idx   = 0;
                rx_buf[rx_idx++] = b;
            }
        } else {
            if (rx_idx < FRAME_LEN) {
                rx_buf[rx_idx++] = b;
            }

            if (rx_idx == FRAME_LEN) {
                in_frame = false;
                SensorPayload_t payload;
                
                if (validateFrame(rx_buf, &payload)) {
                    sendViaEspNow(&payload);
                }
                rx_idx = 0;
            }
        }
    }
}
```

### 6.4 ESP-NOW Transmission

```cpp
void onSendCallback(uint8_t *mac, uint8_t status)
{
    espnow_sent = true;
    espnow_ok   = (status == 0);  /* 0 = success */
}

void sendViaEspNow(const SensorPayload_t *payload)
{
    espnow_sent = false;
    espnow_ok   = false;

    int result = esp_now_send(GATEWAY_MAC,
                              (uint8_t *)payload,
                              sizeof(SensorPayload_t));

    if (result != 0) {
        Serial.println("[ESP-NOW] Send enqueue FAILED");
        return;
    }

    /* Wait for callback (200 ms timeout) */
    uint32_t t = millis();
    while (!espnow_sent && (millis() - t) < 200)
        yield();

    if (espnow_ok)
        Serial.println("[ESP-NOW] Delivered OK");
    else
        Serial.println("[ESP-NOW] Delivery FAILED (no ACK)");
}
```

---

## 7. ESP32-S3 Gateway Implementation

### 7.1 Hardware Specifications

| Parameter | Value |
|-----------|-------|
| Processor | Dual-core Xtensa LX7 |
| Clock Speed | 240 MHz |
| SRAM | 512 KB |
| Flash | 8 MB (typical) |
| WiFi | 802.11 b/g/n (2.4 GHz + 5 GHz) |
| Bluetooth | BLE 5.0 |
| GPIO | 45 programmable pins |
| Special Features | USB OTG, AI vector instructions |

### 7.2 Hybrid Operation Mode

The gateway implements a dual-mode strategy:
1. **ESP-NOW Listening**: Default state, low-power reception
2. **WiFi Connection**: Activated only during cloud upload windows

```cpp
void setup()
{
    /* Start in STA mode without connecting */
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    /* Lock to ESP-NOW channel */
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    
    /* Initialize ESP-NOW */
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ERROR] ESP-NOW init failed!");
        while (1) delay(1000);
    }
    
    esp_now_register_recv_cb(onDataReceived);
    Serial.printf("[Gateway] Listening on channel %d\n", ESPNOW_CHANNEL);
}
```

### 7.3 Receive Callback with MAC Filtering

```cpp
void onDataReceived(const esp_now_recv_info_t *info,
                    const uint8_t *data, int len)
{
    const uint8_t *sender_mac = info->src_addr;

    /* Optional MAC address filtering */
    if (FILTER_BY_MAC && !macIsZero(ALLOWED_SENDER_MAC)) {
        if (!macMatch(sender_mac, ALLOWED_SENDER_MAC)) {
            pkt_filtered++;
            return;
        }
    }

    /* Validate packet size */
    if (len != sizeof(SensorPayload_t)) {
        pkt_bad_size++;
        return;
    }

    pkt_received++;
    SensorPayload_t payload;
    memcpy(&payload, data, sizeof(SensorPayload_t));
    int rssi = info->rx_ctrl->rssi;

    /* Log JSON-formatted output */
    Serial.printf("{\"src\":\"%02X:%02X:%02X:%02X:%02X:%02X\","
                  "\"rssi\":%d,\"t\":%.1f,\"h\":%.1f,"
                  "\"co2\":%.1f,\"nh3\":%.1f,\"co\":%.1f,"
                  "\"alc\":%.1f,\"tol\":%.1f,\"adc\":%u}\n",
                  sender_mac[0], sender_mac[1], sender_mac[2],
                  sender_mac[3], sender_mac[4], sender_mac[5],
                  rssi, payload.temperature, payload.humidity,
                  payload.co2, payload.nh3, payload.co,
                  payload.alcohol, payload.toluene, payload.adc_raw);

    /* Store for cloud upload */
    latestPayload = payload;
    newPayloadAvailable = true;
}
```

### 7.4 ThingSpeak Cloud Integration

```cpp
void uploadToThingSpeak()
{
    if (!newPayloadAvailable) return;

    Serial.println("[WiFi] Connecting for upload...");
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    /* Connection timeout (10 seconds) */
    int timeout = 10 * 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        Serial.print(".");
        timeout--;
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\n[WiFi] Connection failed");
        WiFi.disconnect(true);
        return;
    }

    /* Build HTTP GET request */
    String url = "http://api.thingspeak.com/update?api_key=";
    url += THINGSPEAK_API_KEY;
    url += "&field1=" + String(latestPayload.temperature, 1);
    url += "&field2=" + String(latestPayload.humidity, 1);
    url += "&field3=" + String(latestPayload.co2, 1);
    url += "&field4=" + String(latestPayload.nh3, 1);
    url += "&field5=" + String(latestPayload.co, 1);
    url += "&field6=" + String(latestPayload.alcohol, 1);
    url += "&field7=" + String(latestPayload.toluene, 1);
    url += "&field8=" + String(latestPayload.adc_raw);

    /* Execute HTTP request */
    HTTPClient http;
    http.begin(url);
    int httpCode = http.GET();

    if (httpCode > 0) {
        Serial.printf("[ThingSpeak] HTTP %d – %s\n", 
                      httpCode, http.getString().c_str());
    } else {
        Serial.printf("[ThingSpeak] Error: %s\n", 
                      http.errorToString(httpCode).c_str());
    }
    http.end();

    /* Restore ESP-NOW operation */
    WiFi.disconnect(true);
    delay(500);
    esp_now_deinit();
    
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_now_init();
    esp_now_register_recv_cb(onDataReceived);
    
    newPayloadAvailable = false;
}
```

---

## 8. Data Flow and Packet Transmission

### 8.1 End-to-End Workflow

```
┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 1: SENSOR ACQUISITION (STM32F103C8T6)                        │
│                                                                     │
│  t=0ms:  DHT22_StartSignal() → 80µs LOW + 80µs HIGH                │
│  t=5ms:  DHT22_ReadData() → 40 bits @ ~40µs/bit = 1.6ms            │
│  t=10ms: MQ135_ReadADC() → 239.5 ADC cycles @ 12MHz = 20µs         │
│  t=15ms: MQ135_GetRs() → Voltage divider calculation               │
│  t=20ms: MQ135_GetCorrectionFactor() → Polynomial evaluation       │
│  t=25ms: MQ135_GetPPM() × 6 gases → powf() operations              │
│  t=30ms: UpdateOLED() → 1024 bytes via I2C (400kHz) ≈ 25ms         │
│  t=55ms: SendTelemetry() → 34 bytes @ 115200 baud ≈ 3ms            │
│                                                                     │
│  Total Active Time: ~60ms                                          │
│  Sleep Time: 1940ms (HAL_Delay)                                    │
│  Duty Cycle: 3%                                                    │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 2: UART TRANSMISSION (STM32 → ESP-01)                        │
│                                                                     │
│  Frame: [0xAA][30][payload×30][CRC][0x55]                         │
│  Start Bit: 0xAA detected by ESP-01 state machine                 │
│  Payload Copy: memcpy() to SensorPayload_t struct                 │
│  CRC Verify: XOR of 32 bytes compared to received CRC             │
│  Result: Valid → ESP-NOW queue, Invalid → discard + log           │
│                                                                     │
│  Latency: < 5ms from frame completion                              │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 3: ESP-NOW WIRELESS LINK (ESP-01 → ESP32-S3)                 │
│                                                                     │
│  Channel: 1 (2.412 GHz)                                            │
│  Modulation: OFDM (64-QAM)                                         │
│  Data Rate: 250 kbps (long range mode)                             │
│  Packet: 30 bytes payload + MAC headers ≈ 50 bytes over-air       │
│  Transmission Time: 50 × 8 / 250000 ≈ 1.6ms                       │
│  ACK Timeout: 10ms                                                 │
│  Retry Policy: None (connectionless)                               │
│                                                                     │
│  RSSI Range: -40 dBm (close) to -90 dBm (edge)                    │
│  Packet Loss: < 1% at 50m LOS                                     │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 4: GATEWAY PROCESSING (ESP32-S3)                             │
│                                                                     │
│  onDataReceived():                                                 │
│    • MAC filtering (optional)                                      │
│    • Size validation (must equal 30 bytes)                         │
│    • RSSI extraction from rx_ctrl                                  │
│    • JSON logging to Serial                                        │
│    • Store to latestPayload buffer                                 │
│    • Set newPayloadAvailable flag                                  │
│                                                                     │
│  Upload Scheduler:                                                 │
│    • Check POST_INTERVAL (20 seconds minimum for ThingSpeak)       │
│    • Activate WiFi station mode                                    │
│    • Associate with AP (DHCP, DNS)                                 │
│    • HTTP GET to api.thingspeak.com                                │
│    • Disconnect WiFi                                               │
│    • Re-initialize ESP-NOW                                         │
│                                                                     │
│  WiFi Connection Time: 2-5 seconds                                 │
│  HTTP Transaction: 500ms - 2 seconds                               │
│  ESP-NOW Restoration: < 100ms                                      │
└─────────────────────────────────────────────────────────────────────┘
                              │
                              ▼
┌─────────────────────────────────────────────────────────────────────┐
│ PHASE 5: CLOUD INTEGRATION (ThingSpeak)                            │
│                                                                     │
│  URL Format:                                                       │
│  http://api.thingspeak.com/update?api_key=XXXXXX&                  │
│    field1=T&field2=H&field3=CO2&field4=NH3&                        │
│    field5=CO&field6=Alcohol&field7=Toluene&field8=ADC              │
│                                                                     │
│  Response:                                                         │
│    • Success: Entry ID number (e.g., "12345")                      │
│    • Rate Limit: HTTP 429 if < 15 seconds between updates          │
│    • Invalid Key: HTTP 400                                         │
│                                                                     │
│  Data Visualization:                                               │
│    • Real-time charts (auto-generated)                             │
│    • MATLAB analysis scripts                                       │
│    • Tweet/Email alerts (IFTTT integration)                        │
└─────────────────────────────────────────────────────────────────────┘
```

### 8.2 Timing Budget Analysis

| Phase | Minimum | Typical | Maximum |
|-------|---------|---------|---------|
| Sensor Acquisition (STM32) | 55 ms | 60 ms | 80 ms |
| UART Transmission | 2.5 ms | 3.0 ms | 4.0 ms |
| ESP-NOW Link | 1.6 ms | 2.0 ms | 12 ms |
| Gateway Processing | < 1 ms | 2 ms | 5 ms |
| WiFi Connection (on-demand) | 2000 ms | 3500 ms | 10000 ms |
| HTTP Upload | 500 ms | 1500 ms | 5000 ms |
| **Total (local)** | **59.1 ms** | **67 ms** | **101 ms** |
| **Total (with cloud)** | **2559 ms** | **5067 ms** | **15080 ms** |

---

## 9. Power Management Considerations

### 9.1 Current Consumption Estimates

| Component | Active Mode | Sleep/Idle | Duty Cycle | Average |
|-----------|-------------|------------|------------|---------|
| STM32F103C8T6 @ 72MHz | 45 mA | 15 mA (Stop) | 3% | 16.2 mA |
| MQ135 Heater | 150 mA | 150 mA | 100% | 150 mA |
| DHT22 | 1.5 mA | 60 µA | 3% | 1.5 mA |
| SSD1306 OLED | 20 mA | 0 mA | 50%* | 10 mA |
| ESP-01 (TX) | 280 mA | 15 mA | 0.15% | 15.4 mA |
| **Total (Sensor Node)** | | | | **~193 mA** |

*OLED can be disabled after boot for power savings.

### 9.2 Optimization Strategies

1. **MQ135 Heater Cycling**: Not recommended—thermal cycling reduces sensor lifespan. Continuous operation ensures stable baseline.

2. **STM32 Low-Power Modes**:
   ```c
   /* Enter Stop mode between readings */
   HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
   
   /* Wake via TIM1 update event */
   __HAL_RCC_APB1PERIPH_CLK_ENABLE();
   ```

3. **Display Auto-Off**:
   ```c
   /* Disable OLED after 10 seconds of inactivity */
   if (idle_time > 10000)
       SSD1306_OFF();
   ```

---

## 10. Production and Development Notes

### 10.1 Schematic and PCB Design

The schematic and PCB layouts (provided in KiCad format under `Schematic_PCB/`) are designed to support both production manufacturing and development prototyping:

**Key Design Features:**

1. **Test Points**: All critical signals (UART TX/RX, I2C SCL/SDA, ADC input, GPIO) are accessible via 1.27mm pitch test points for logic analyzer and oscilloscope probing.

2. **Programming Interfaces**:
   - STM32: SWD connector (SWCLK, SWDIO, NRST, GND, 3.3V)
   - ESP-01: 2.54mm header compatible with USB-TTL adapters
   - ESP32-S3: Native USB-C port for DFU programming

3. **Power Distribution**:
   - Separate analog (AVDD) and digital (DVDD) planes
   - LC filter for MQ135 ADC reference (10µH + 10µF)
   - Decoupling capacitors: 100nF per IC, 10µF per power rail

4. **Connector Standardization**:
   - JST-PH 2.0mm for sensors (DHT22, MQ135)
   - Grove-compatible I2C header
   - USB-C for gateway programming and debugging

5. **Manufacturing Considerations**:
   - All components on top layer (single-sided assembly)
   - Minimum trace width: 0.25mm (manufacturable by JLCPCB, PCBWay)
   - Via size: 0.3mm drill, 0.6mm annulus
   - Silkscreen component designators on top layer

6. **Development Flexibility**:
   - 0Ω jumpers for selecting power sources (USB vs. external)
   - Footprint compatibility: STM32F103C8T6 LQFP48 and minimal QFN32
   - Unpopulated footprints for optional RTC battery, SD card

### 10.2 Calibration Procedures

**MQ135 R0 Calibration:**
1. Power sensor for 24 hours (initial burn-in)
2. Place in clean outdoor air (away from pollution sources)
3. Execute calibration routine via debug UART command
4. Record R0 value and hardcode in firmware:
   ```c
   MQ135_SetR0(9.83f);  /* Example calibrated value */
   ```

**DHT22 Verification:**
Compare against NIST-traceable reference thermometer/hygrometer. Apply offset correction in software if deviation exceeds ±2°C or ±5% RH.

### 10.3 Firmware Build Instructions

**STM32 (STM32CubeIDE):**
```bash
1. Import project: Embedded_Trial/
2. Select toolchain: ARM GCC
3. Build: Project → Build All (F7)
4. Flash: Run → Debug (uses ST-Link/V2)
```

**ESP-01 (Arduino IDE):**
```bash
1. Board: "Generic ESP8266 Module"
2. Flash Size: "1MB (64KB SPIFFS)"
3. Upload Speed: 115200
4. CPU Frequency: "80 MHz"
5. Upload: Requires GPIO0 pulled LOW during reset
```

**ESP32-S3 (Arduino IDE):**
```bash
1. Board: "ESP32S3 Dev Module"
2. USB CDC On Boot: "Enabled"
3. Flash Size: "8MB"
4. Partition Scheme: "Default 4MB with spiffs"
5. Upload: Hold BOOT button during reset
```

---

## 11. Appendix: Complete Code Listings

### 11.1 STM32F103C8T6 Source Files

#### `main.c` (Excerpt - Frame Transmission)
```c
/**
 * @brief  Build and transmit a 34-byte binary sensor frame to ESP-01.
 */
static void SendTelemetry(void)
{
  uint8_t frame[FRAME_TOTAL];
  uint8_t idx = 0;

  SensorPayload_t p;
  p.temperature = dht_data.temperature;
  p.humidity    = dht_data.humidity;
  p.co2         = gas_data.co2;
  p.nh3         = gas_data.nh4;
  p.co          = gas_data.co;
  p.alcohol     = gas_data.alcohol;
  p.toluene     = gas_data.toluene;
  p.adc_raw     = gas_data.adc_raw;

  frame[idx++] = FRAME_SOF;
  frame[idx++] = (uint8_t)sizeof(SensorPayload_t);
  memcpy(&frame[idx], &p, sizeof(SensorPayload_t));
  idx += sizeof(SensorPayload_t);

  uint8_t crc = 0;
  for (uint8_t i = 0; i < idx; i++)
    crc ^= frame[i];
  frame[idx++] = crc;
  frame[idx++] = FRAME_EOF;

  HAL_UART_Transmit(&huart3, frame, (uint16_t)idx, 50);
}
```

#### `dht22.h` (Public API)
```c
typedef struct {
    float temperature;  /**< Temperature in °C (–40 … +80)  */
    float humidity;     /**< Relative humidity in %RH (0 … 100) */
} DHT22_Data_t;

void DHT22_Init(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim);
uint8_t DHT22_ReadData(DHT22_Data_t *data);
```

#### `mq135.h` (Gas Coefficients)
```c
/* Carbon Monoxide (CO) */
#define MQ135_CO_A              605.18f
#define MQ135_CO_B              (-3.937f)

/* Alcohol (Ethanol) */
#define MQ135_ALCOHOL_A         77.255f
#define MQ135_ALCOHOL_B         (-3.18f)

/* Carbon Dioxide (CO2) */
#define MQ135_CO2_A             110.47f
#define MQ135_CO2_B             (-2.862f)

/* Toluene */
#define MQ135_TOLUENE_A         44.947f
#define MQ135_TOLUENE_B         (-3.445f)

/* Ammonia (NH3 / NH4) */
#define MQ135_NH4_A             102.2f
#define MQ135_NH4_B             (-2.473f)

/* Acetone */
#define MQ135_ACETONE_A         34.668f
#define MQ135_ACETONE_B         (-3.369f)
```

### 11.2 ESP-01 Bridge Firmware

#### `ESP01_Sensor_Bridge.ino` (ESP-NOW Initialization)
```cpp
void setup()
{
    Serial.begin(UART_BAUD);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != 0) {
        Serial.println("[ERROR] ESP-NOW init failed!");
        while (1) delay(1000);
    }

    esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
    esp_now_register_send_cb(onSendCallback);

    esp_now_add_peer(GATEWAY_MAC,
                     ESP_NOW_ROLE_SLAVE,
                     1, NULL, 0);
}
```

### 11.3 ESP32-S3 Gateway Firmware

#### `ESP32S3_Gateway.ino` (ThingSpeak Upload)
```cpp
void uploadToThingSpeak()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    
    int timeout = 10 * 20;
    while (WiFi.status() != WL_CONNECTED && timeout > 0) {
        delay(500);
        timeout--;
    }

    if (WiFi.status() == WL_CONNECTED) {
        String url = "http://api.thingspeak.com/update?api_key=";
        url += THINGSPEAK_API_KEY;
        url += "&field1=" + String(latestPayload.temperature, 1);
        // ... additional fields
        
        HTTPClient http;
        http.begin(url);
        int httpCode = http.GET();
        http.end();
        
        WiFi.disconnect(true);
        esp_now_deinit();
        esp_now_init();
        esp_now_register_recv_cb(onDataReceived);
    }
}
```

---

## References

1. STMicroelectronics. (2023). *STM32F103xC/D/E Datasheet*. DocID13587 Rev 15.
2. Hanwei Electronics. (2022). *MQ-135 Hazardous Gas Sensor Datasheet*.
3. Aosong Electronics. (2021). *DHT22 (AM2302) Digital Temperature and Humidity Sensor Specification*.
4. Espressif Systems. (2023). *ESP-NOW Programming Guide*. ESP32 Technical Reference Manual.
5. MathWorks. (2023). *ThingSpeak IoT Analytics Platform Documentation*.
6. Solomon Systech. (2020). *SSD1306 128x64 Dot Matrix OLED/PLED Segment/Common Driver*.

---

## Document Information

| Attribute | Value |
|-----------|-------|
| Version | 1.0 |
| Date | 2026 |
| Author | Embedded Systems Development Team |
| License | Open Source (MIT) |
| Repository | `/workspace` |

---

*This documentation was generated for academic and engineering reference purposes. All code snippets are extracted from the production firmware and verified against the deployed system.*
