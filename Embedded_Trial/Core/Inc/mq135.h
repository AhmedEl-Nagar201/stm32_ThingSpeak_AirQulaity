/**
 * @file    mq135.h
 * @brief   MQ135 Air Quality Gas Sensor driver for STM32
 *
 * Supports calculating PPM for all gases on the MQ135 sensitivity curves:
 *   - CO   (Carbon Monoxide)
 *   - Alcohol
 *   - CO2  (Carbon Dioxide)
 *   - Toluene
 *   - NH4  (Ammonia)
 *   - Acetone
 *
 * Gas concentration is computed using the power-law regression:
 *     PPM = a × (Rs/R0)^b
 * where a, b are gas-specific coefficients derived from the MQ135 datasheet
 * sensitivity curves.
 *
 * IMPORTANT CALIBRATION NOTES:
 *   1. The sensor heater MUST be preheated for at least 24 hours on first use
 *      (20 min minimum for subsequent power-ons).
 *   2. R0 must be calibrated in clean outdoor air using MQ135_CalibrateR0().
 *   3. The load resistor value (RL) must match your breakout board.
 *      Common modules use 1 kΩ; better ones use 10–47 kΩ.
 *   4. If your MQ135 module is powered at 5 V but the STM32 ADC reference
 *      is 3.3 V, set MQ135_SENSOR_VCC to 5.0 and MQ135_ADC_VREF to 3.3.
 *
 * Usage:
 *   MQ135_Init(&hadc2, 1.0f);            // RL = 1 kΩ
 *   MQ135_CalibrateR0(50);               // Average 50 readings in clean air
 *   MQ135_GasReadings_t gas;
 *   MQ135_ReadAllGases(&gas, temp, hum); // temp/hum from DHT22
 */

#ifndef MQ135_H
#define MQ135_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ---- User-tunable parameters -------------------------------------------- */

/** Voltage powering the MQ135 module's analog voltage divider (Rs + RL).
 *  Set to 5.0 if the module is powered from 5 V, or 3.3 if from 3.3 V. */
#ifndef MQ135_SENSOR_VCC
#define MQ135_SENSOR_VCC        5.0f
#endif

/** ADC reference voltage of the STM32 (usually 3.3 V). */
#ifndef MQ135_ADC_VREF
#define MQ135_ADC_VREF          3.3f
#endif

/** ADC maximum digital value (12-bit = 4095). */
#ifndef MQ135_ADC_RESOLUTION
#define MQ135_ADC_RESOLUTION    4095.0f
#endif

/** Rs/R0 ratio in clean air from the MQ135 datasheet (~3.6). */
#ifndef MQ135_CLEAN_AIR_FACTOR
#define MQ135_CLEAN_AIR_FACTOR  3.6f
#endif

/* ---- Gas-specific curve coefficients ------------------------------------ */
/*
 * PPM = a × (Rs/R0)^b
 * These are derived from power regression on the MQ135 datasheet log-log
 * sensitivity curves.  Adjust if your sensor batch differs significantly.
 */

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

/* ---- Data types --------------------------------------------------------- */

/**
 * @brief  All gas concentration readings from the MQ135 (in PPM).
 *
 * A value of -1.0 indicates an invalid / unreadable measurement
 * (e.g. ADC saturated or R0 not calibrated).
 */
typedef struct {
    float co;         /**< Carbon Monoxide  (ppm) */
    float alcohol;    /**< Alcohol / Ethanol (ppm) */
    float co2;        /**< Carbon Dioxide   (ppm) */
    float toluene;    /**< Toluene          (ppm) */
    float nh4;        /**< Ammonia          (ppm) */
    float acetone;    /**< Acetone          (ppm) */
    /* Diagnostic fields */
    uint16_t adc_raw; /**< Last raw ADC value (0–4095) */
    float    rs;      /**< Sensor resistance Rs (kΩ)   */
    float    rs_ro;   /**< Rs / R0 ratio               */
} MQ135_GasReadings_t;

/* ---- Public API --------------------------------------------------------- */

/**
 * @brief  Initialise the MQ135 driver.
 * @param  hadc      Pointer to the ADC handle connected to the sensor.
 * @param  rl_kohm   Load resistor value in kΩ (check your breakout board).
 */
void MQ135_Init(ADC_HandleTypeDef *hadc, float rl_kohm);

/**
 * @brief  Calibrate R0 in clean air.
 *
 * Call this once after the sensor has been preheated.  It takes multiple
 * ADC samples, averages them, and stores R0 for subsequent gas calculations.
 *
 * @param  num_samples  Number of ADC samples to average (≥10 recommended).
 * @retval The calibrated R0 value in kΩ (also stored internally).
 *         Returns -1.0 if calibration failed.
 */
float MQ135_CalibrateR0(uint16_t num_samples);

/**
 * @brief  Set R0 directly (useful if you have a previously calibrated value).
 * @param  r0_kohm  R0 in kΩ.
 */
void MQ135_SetR0(float r0_kohm);

/**
 * @brief  Get the current R0 value.
 * @retval R0 in kΩ.
 */
float MQ135_GetR0(void);

/**
 * @brief  Read a single raw ADC value from the MQ135.
 * @retval 12-bit ADC value (0–4095), or 0 on failure.
 */
uint16_t MQ135_ReadADC(void);

/**
 * @brief  Calculate the sensor resistance Rs from an ADC reading.
 * @param  adc_raw  Raw ADC value (0–4095).
 * @retval Rs in kΩ, or -1.0 if adc_raw is 0 (open circuit).
 */
float MQ135_GetRs(uint16_t adc_raw);

/**
 * @brief  Get the temperature/humidity correction factor.
 *
 * The MQ135 sensitivity varies with ambient conditions.  This function
 * returns a multiplier to compensate Rs readings.
 *
 * Reference conditions: 20 °C, 33 %RH → factor = 1.0.
 *
 * @param  temperature  Ambient temperature in °C (from DHT22).
 * @param  humidity     Relative humidity in %RH  (from DHT22).
 * @retval Correction factor (multiply Rs by this to get corrected Rs).
 */
float MQ135_GetCorrectionFactor(float temperature, float humidity);

/**
 * @brief  Calculate PPM for a specific gas given the Rs/R0 ratio.
 * @param  rs_ro  The Rs/R0 ratio (optionally temperature-compensated).
 * @param  a      Gas curve coefficient 'a'.
 * @param  b      Gas curve coefficient 'b'.
 * @retval Concentration in PPM.
 */
float MQ135_GetPPM(float rs_ro, float a, float b);

/**
 * @brief  Read all supported gas concentrations in one call.
 *
 * Takes an ADC reading, applies optional temperature/humidity compensation,
 * and fills in the result struct with PPM values for all six gases.
 *
 * @param  readings     Pointer to the result struct.
 * @param  temperature  Ambient temperature in °C for compensation.
 *                      Pass -999.0 to skip compensation.
 * @param  humidity     Ambient humidity in %RH for compensation.
 *                      Pass -999.0 to skip compensation.
 * @retval 1 = success, 0 = ADC read failed or R0 not calibrated.
 */
uint8_t MQ135_ReadAllGases(MQ135_GasReadings_t *readings,
                           float temperature, float humidity);

#ifdef __cplusplus
}
#endif

#endif /* MQ135_H */
