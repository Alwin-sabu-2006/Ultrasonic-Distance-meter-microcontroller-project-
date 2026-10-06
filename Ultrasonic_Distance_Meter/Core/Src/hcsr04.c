/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hcsr04.c
  * @brief   HC-SR04 Ultrasonic Distance Sensor Driver Implementation
  *
  * Target MCU : STM32F103C8T6 (Blue Pill)
  * System clock: 72 MHz (8 MHz HSE x PLL x9)
  * Timer       : TIM1, Prescaler=71 => 1 us per tick (1 MHz)
  *
  * Pin mapping (set by CubeMX, do NOT change here):
  *   PA9  = TRIG (output, push-pull)
  *   PA8  = ECHO (input, no pull)
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------ */
#include "hcsr04.h"
#include <string.h>   /* memset */

/* Private helpers ----------------------------------------------------------- */

/**
 * @brief  Delay exactly 'us' microseconds using TIM1 counter.
 *         TIM1 must already be started (HAL_TIM_Base_Start called).
 * @note   Maximum reliable delay = 65535 us (16-bit counter).
 */
static void delay_us(TIM_HandleTypeDef *htim, uint32_t us)
{
    /* Reload counter to 0 and wait */
    __HAL_TIM_SET_COUNTER(htim, 0);
    while ((uint32_t)__HAL_TIM_GET_COUNTER(htim) < us)
    {
        /* spin */
    }
}

/**
 * @brief  Wait for ECHO pin to reach 'state' within 'timeout_us'.
 * @retval 1 if pin reached the requested state, 0 on timeout.
 */
static uint8_t wait_for_echo(TIM_HandleTypeDef *htim,
                             GPIO_PinState        state,
                             uint32_t             timeout_us)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    while (HAL_GPIO_ReadPin(HCSR04_ECHO_PORT, HCSR04_ECHO_PIN) != state)
    {
        if ((uint32_t)__HAL_TIM_GET_COUNTER(htim) >= timeout_us)
        {
            return 0U; /* timeout */
        }
    }
    return 1U;
}

/* Public API ---------------------------------------------------------------- */

/**
 * @brief  Initialise the HC-SR04 driver handle.
 */
HAL_StatusTypeDef HCSR04_Init(HCSR04_HandleTypeDef *hdev, TIM_HandleTypeDef *htim)
{
    if ((hdev == NULL) || (htim == NULL))
    {
        return HAL_ERROR;
    }

    memset(hdev, 0, sizeof(HCSR04_HandleTypeDef));
    hdev->htim         = htim;
    hdev->sound_divisor = HCSR04_DEFAULT_DIVISOR;

    /* Ensure TRIG is LOW at start */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* Start the free-running microsecond timer */
    return HAL_TIM_Base_Start(htim);
}

/**
 * @brief  Trigger one measurement and return raw pulse width in microseconds.
 * @retval Pulse duration in us, or 0 on timeout.
 */
uint32_t HCSR04_ReadPulse_us(HCSR04_HandleTypeDef *hdev)
{
    uint32_t pulse_us = 0;

    /* ---- 1. Generate 10 us TRIG pulse ------------------------------------ */
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_SET);
    delay_us(hdev->htim, 10U);
    HAL_GPIO_WritePin(HCSR04_TRIG_PORT, HCSR04_TRIG_PIN, GPIO_PIN_RESET);

    /* ---- 2. Wait for ECHO to go HIGH (up to 38 ms) ----------------------- */
    if (!wait_for_echo(hdev->htim, GPIO_PIN_SET, HCSR04_TIMEOUT_US))
    {
        return 0U; /* No echo detected (sensor not responding) */
    }

    /* ---- 3. Measure ECHO HIGH duration ----------------------------------- */
    __HAL_TIM_SET_COUNTER(hdev->htim, 0);

    if (!wait_for_echo(hdev->htim, GPIO_PIN_RESET, HCSR04_TIMEOUT_US))
    {
        return 0U; /* Echo never fell (object too close / stuck) */
    }

    pulse_us = (uint32_t)__HAL_TIM_GET_COUNTER(hdev->htim);

    return pulse_us;
}

/**
 * @brief  Read distance in cm (single raw measurement, no filter).
 */
float HCSR04_ReadRaw_cm(HCSR04_HandleTypeDef *hdev)
{
    uint32_t pulse = HCSR04_ReadPulse_us(hdev);

    if (pulse == 0U)
    {
        return HCSR04_OUT_OF_RANGE;
    }

    return (float)pulse / hdev->sound_divisor;
}

/**
 * @brief  Read distance in cm with a moving-average filter.
 *         Out-of-range readings are excluded from the average.
 */
float HCSR04_ReadFiltered_cm(HCSR04_HandleTypeDef *hdev)
{
    float raw = HCSR04_ReadRaw_cm(hdev);

    /* Reject out-of-range readings from the filter */
    if (raw == HCSR04_OUT_OF_RANGE)
    {
        return HCSR04_OUT_OF_RANGE;
    }

    /* Store in circular buffer */
    hdev->filter_buf[hdev->filter_idx] = raw;
    hdev->filter_idx = (uint8_t)((hdev->filter_idx + 1U) % HCSR04_FILTER_SAMPLES);

    if (hdev->filter_filled < HCSR04_FILTER_SAMPLES)
    {
        hdev->filter_filled++;
    }

    /* Compute average over valid samples */
    float  sum   = 0.0f;
    uint8_t count = hdev->filter_filled;

    for (uint8_t i = 0U; i < count; i++)
    {
        /* Index from oldest to newest */
        uint8_t idx = (uint8_t)((hdev->filter_idx + (HCSR04_FILTER_SAMPLES - count) + i)
                                % HCSR04_FILTER_SAMPLES);
        sum += hdev->filter_buf[idx];
    }

    return sum / (float)count;
}

/**
 * @brief  Set temperature compensation.
 *         Adjusts internal sound_divisor for the given ambient temperature.
 */
void HCSR04_SetTemperature(HCSR04_HandleTypeDef *hdev, float temp_C)
{
    /* Speed of sound (m/s): v = 331.3 + 0.606 * T */
    float v_sound = 331.3f + (0.606f * temp_C);

    /*
     * Round-trip distance per cm:
     *   1 cm = 0.01 m  =>  round-trip = 0.02 m
     *   time (us) = 0.02 / v_sound * 1e6  = 20000.0 / v_sound
     */
    hdev->sound_divisor = 20000.0f / v_sound;
}
