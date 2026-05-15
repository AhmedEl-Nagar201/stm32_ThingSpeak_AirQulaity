/**
 * @file    dht22.c
 * @brief   DHT22 (AM2302) Temperature & Humidity sensor driver implementation.
 */

#include "dht22.h"

/* ---- Private state ------------------------------------------------------ */

static GPIO_TypeDef      *dht_port;
static uint16_t           dht_pin;
static TIM_HandleTypeDef *dht_htim;

/* ---- Private helpers ---------------------------------------------------- */

/** Microsecond delay using the hardware timer. */
static void DHT22_MicroDelay(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(dht_htim, 0);
    while (__HAL_TIM_GET_COUNTER(dht_htim) < us);
}

/** Configure the data pin as push-pull output. */
static void DHT22_SetPinOutput(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = dht_pin;
    gpio.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(dht_port, &gpio);
}

/** Configure the data pin as input with pull-up. */
static void DHT22_SetPinInput(void)
{
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = dht_pin;
    gpio.Mode = GPIO_MODE_INPUT;
    gpio.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(dht_port, &gpio);
}

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

    if (!(HAL_GPIO_ReadPin(dht_port, dht_pin)))
    {
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

    for (uint8_t i = 0; i < 8; i++)
    {
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

/* ---- Public API --------------------------------------------------------- */

void DHT22_Init(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *htim)
{
    dht_port = port;
    dht_pin  = pin;
    dht_htim = htim;
}

uint8_t DHT22_ReadData(DHT22_Data_t *data)
{
    if (!data) return 0;

    data->temperature = 0.0f;
    data->humidity    = 0.0f;

    if (!DHT22_StartSignal())
        return 0;   /* Sensor not responding */

    /* Read 5 bytes: Hum_H, Hum_L, Temp_H, Temp_L, Checksum */
    uint8_t hum_h  = DHT22_ReadByte();
    uint8_t hum_l  = DHT22_ReadByte();
    uint8_t temp_h = DHT22_ReadByte();
    uint8_t temp_l = DHT22_ReadByte();
    uint8_t chksum = DHT22_ReadByte();

    /* Verify checksum */
    uint8_t calc = (uint8_t)(hum_h + hum_l + temp_h + temp_l);
    if (calc != chksum)
        return 0;   /* Checksum mismatch */

    /* Humidity (always positive) */
    data->humidity = (float)(((uint16_t)hum_h << 8) | hum_l) / 10.0f;

    /* Temperature – bit 15 is sign flag */
    uint16_t raw_temp = ((uint16_t)(temp_h & 0x7F) << 8) | temp_l;
    data->temperature = (float)raw_temp / 10.0f;
    if (temp_h & 0x80)
        data->temperature = -data->temperature;

    return 1;   /* Success */
}
