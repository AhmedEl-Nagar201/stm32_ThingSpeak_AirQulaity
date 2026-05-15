/**
 * @file    dht22.h
 * @brief   DHT22 (AM2302) Temperature & Humidity sensor driver for STM32
 *
 * Requires a hardware timer configured for 1 µs ticks (prescaler = SystemCoreClock/1MHz - 1).
 * The data pin must have an external or internal pull-up resistor.
 *
 * Usage:
 *   DHT22_Init(GPIOA, GPIO_PIN_1, &htim1);
 *   DHT22_Data_t data;
 *   if (DHT22_ReadData(&data)) {
 *       printf("T=%.1f  H=%.1f\n", data.temperature, data.humidity);
 *   }
 */

#ifndef DHT22_H
#define DHT22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ---- Data types --------------------------------------------------------- */

/**
 * @brief  DHT22 measurement result.
 */
typedef struct {
    float temperature;  /**< Temperature in °C (–40 … +80)  */
    float humidity;     /**< Relative humidity in %RH (0 … 100) */
} DHT22_Data_t;

/* ---- Public API --------------------------------------------------------- */

/**
 * @brief  Initialise the DHT22 driver.
 * @param  port   GPIO port of the data pin  (e.g. GPIOA)
 * @param  pin    GPIO pin mask              (e.g. GPIO_PIN_1)
 * @param  htim   Pointer to a running TIM handle with 1 µs resolution
 */
void DHT22_Init(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim);

/**
 * @brief  Read temperature and humidity from the sensor.
 * @param  data   Pointer to a DHT22_Data_t struct to receive results.
 * @retval 1 = success (data is valid), 0 = communication / checksum error
 *
 * @note   The DHT22 requires a minimum 2 s interval between reads.
 *         Calling this function more frequently will return stale data.
 */
uint8_t DHT22_ReadData(DHT22_Data_t *data);

#ifdef __cplusplus
}
#endif

#endif /* DHT22_H */
