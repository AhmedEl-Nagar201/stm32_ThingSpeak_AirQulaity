/**
 * @file    mq135.c
 * @brief   MQ135 Air Quality Gas Sensor driver implementation.
 *
 * Implements the power-law regression:  PPM = a × (Rs/R0)^b
 * for all six gases on the MQ135 sensitivity curves.
 *
 * Rs is calculated from the ADC reading using the voltage-divider equation:
 *     Vout = VCC × RL / (Rs + RL)
 *     Rs   = RL × (VCC / Vout − 1)
 *
 * Temperature/humidity compensation uses an empirical polynomial derived
 * from the MQ135 datasheet correction curves (reference: 20 °C, 33 %RH).
 */

#include "mq135.h"
#include <math.h>

/* ---- Private state ------------------------------------------------------ */

static ADC_HandleTypeDef *mq_hadc = NULL;
static float              mq_rl   = 2.6f;   /* Load resistance in kΩ */
static float              mq_r0   = 10.0f;  /* Sensor resistance in clean air (kΩ), default estimate */
static uint8_t            mq_calibrated = 0; /* Set to 1 after CalibrateR0 or SetR0 */

/* ---- Temperature/Humidity correction coefficients ----------------------- */
/*
 * Empirical polynomial fit to the MQ135 datasheet temperature/humidity
 * correction curves.  Reference point: 20 °C, 33 %RH → factor = 1.0.
 *
 *   factor = CORA×t² − CORB×t + CORC − (h − 33)×CORD
 *
 * where t = temperature (°C), h = humidity (%RH).
 */
#define CORA    0.00035f
#define CORB    0.02718f
#define CORC    1.39538f
#define CORD    0.0018f

/* ---- Private helpers ---------------------------------------------------- */

/**
 * @brief  Perform a single ADC conversion and return the raw value.
 * @retval 12-bit ADC value, or 0 on timeout.
 */
static uint16_t MQ135_SingleConversion(void)
{
    uint16_t value = 0;

    HAL_ADC_Start(mq_hadc);
    if (HAL_ADC_PollForConversion(mq_hadc, 20) == HAL_OK)
    {
        value = (uint16_t)HAL_ADC_GetValue(mq_hadc);
    }
    HAL_ADC_Stop(mq_hadc);

    return value;
}

/* ---- Public API --------------------------------------------------------- */

void MQ135_Init(ADC_HandleTypeDef *hadc, float rl_kohm)
{
    mq_hadc = hadc;
    mq_rl   = rl_kohm;
    mq_calibrated = 0;
}

float MQ135_CalibrateR0(uint16_t num_samples)
{
    if (!mq_hadc || num_samples == 0)
        return -1.0f;

    float rs_sum = 0.0f;
    uint16_t valid = 0;

    for (uint16_t i = 0; i < num_samples; i++)
    {
        uint16_t adc = MQ135_SingleConversion();
        if (adc == 0) continue;  /* Skip saturated/zero readings */

        float rs = MQ135_GetRs(adc);
        if (rs > 0.0f)
        {
            rs_sum += rs;
            valid++;
        }

        HAL_Delay(50);  /* Brief pause between samples */
    }

    if (valid == 0)
        return -1.0f;

    /* In clean air: Rs/R0 ≈ MQ135_CLEAN_AIR_FACTOR (typically 3.6) */
    float rs_avg = rs_sum / (float)valid;
    mq_r0 = rs_avg / MQ135_CLEAN_AIR_FACTOR;
    mq_calibrated = 1;

    return mq_r0;
}

void MQ135_SetR0(float r0_kohm)
{
    if (r0_kohm > 0.0f)
    {
        mq_r0 = r0_kohm;
        mq_calibrated = 1;
    }
}

float MQ135_GetR0(void)
{
    return mq_r0;
}

uint16_t MQ135_ReadADC(void)
{
    return MQ135_SingleConversion();
}

float MQ135_GetRs(uint16_t adc_raw)
{
    if (adc_raw == 0)
        return -1.0f;  /* Cannot compute – would be division by zero */

    /*
     * Vout = adc_raw × ADC_VREF / ADC_RESOLUTION
     * Rs   = RL × (VCC / Vout − 1)
     *
     * Combining:
     *   Rs = RL × (VCC × ADC_RESOLUTION / (adc_raw × ADC_VREF) − 1)
     */
    float vout = (float)adc_raw * MQ135_ADC_VREF / MQ135_ADC_RESOLUTION;
    float rs   = mq_rl * ((MQ135_SENSOR_VCC / vout) - 1.0f);

    return (rs > 0.0f) ? rs : 0.0f;
}

float MQ135_GetCorrectionFactor(float temperature, float humidity)
{
    /*
     * Polynomial correction from datasheet curves.
     * At 20 °C, 33 %RH the factor should be ≈ 1.0.
     */
    float factor = CORA * temperature * temperature
                 - CORB * temperature
                 + CORC
                 - (humidity - 33.0f) * CORD;

    /* Clamp to a sane range to avoid wild corrections */
    if (factor < 0.1f)  factor = 0.1f;
    if (factor > 3.0f)  factor = 3.0f;

    return factor;
}

float MQ135_GetPPM(float rs_ro, float a, float b)
{
    if (rs_ro <= 0.0f)
        return -1.0f;

    return a * powf(rs_ro, b);
}

uint8_t MQ135_ReadAllGases(MQ135_GasReadings_t *readings,
                           float temperature, float humidity)
{
    if (!readings || !mq_hadc)
        return 0;

    /* Default: invalid */
    readings->co      = -1.0f;
    readings->alcohol = -1.0f;
    readings->co2     = -1.0f;
    readings->toluene = -1.0f;
    readings->nh4     = -1.0f;
    readings->acetone = -1.0f;
    readings->adc_raw = 0;
    readings->rs      = -1.0f;
    readings->rs_ro   = -1.0f;

    /* Take ADC reading */
    uint16_t adc = MQ135_SingleConversion();
    readings->adc_raw = adc;

    if (adc == 0)
        return 0;  /* Sensor disconnected or saturated */

    /* Calculate sensor resistance */
    float rs = MQ135_GetRs(adc);
    if (rs < 0.0f)
        return 0;

    readings->rs = rs;

    /* Apply temperature/humidity compensation if valid data provided */
    if (temperature > -100.0f && humidity > -100.0f)
    {
        float corr = MQ135_GetCorrectionFactor(temperature, humidity);
        rs = rs / corr;  /* Corrected Rs */
    }

    /* If R0 hasn't been calibrated, use the default estimate but flag it */
    float rs_ro = rs / mq_r0;
    readings->rs_ro = rs_ro;

    /* Compute PPM for each gas using datasheet curve coefficients */
    readings->co      = MQ135_GetPPM(rs_ro, MQ135_CO_A,      MQ135_CO_B);
    readings->alcohol = MQ135_GetPPM(rs_ro, MQ135_ALCOHOL_A, MQ135_ALCOHOL_B);
    readings->co2     = MQ135_GetPPM(rs_ro, MQ135_CO2_A,     MQ135_CO2_B);
    readings->toluene = MQ135_GetPPM(rs_ro, MQ135_TOLUENE_A, MQ135_TOLUENE_B);
    readings->nh4     = MQ135_GetPPM(rs_ro, MQ135_NH4_A,     MQ135_NH4_B);
    readings->acetone = MQ135_GetPPM(rs_ro, MQ135_ACETONE_A, MQ135_ACETONE_B);

    return 1;
}
