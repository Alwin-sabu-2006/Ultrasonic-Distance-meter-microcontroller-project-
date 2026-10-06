/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    hcsr04.h
  * @brief   HC-SR04 Ultrasonic Distance Sensor Driver for STM32F103C8T6
  *
  * Hardware connections:
  *   - TRIG  → PA9  (GPIO Output Push-Pull)
  *   - ECHO  → PA8  (GPIO Input, No Pull)
  *
  * Timer:
  *   - TIM1, Prescaler = 71  →  1 µs tick @ 72 MHz system clock
  *   - Period = 65535 (free-running 16-bit counter)
  *
  * Measurement principle:
  *   1. Pull TRIG HIGH for ≥10 µs, then LOW.
  *   2. Sensor sends 8 × 40 kHz bursts and raises ECHO.
  *   3. ECHO pulse width (µs) = round-trip time.
  *   4. Distance (cm) = pulse_width_µs / 58.0
  *      (speed of sound ≈ 343 m/s → 1 cm ≈ 58 µs round-trip)
  *
  * Accuracy enhancements:
  *   - Moving-average filter (configurable window)
  *   - Timeout guard (> 38 ms → out of range)
  *   - Configurable speed-of-sound for temperature compensation
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef __HCSR04_H
#define __HCSR04_H

#ifdef __cplusplus
extern "C" {
#endif

/* --- Includes --------------------------------------------------------------- */
#include "stm32f1xx_hal.h"

/* --- Public constants ------------------------------------------------------- */

/** Number of samples averaged per reading (1 = no filter, max 32) */
#define HCSR04_FILTER_SAMPLES     5U

/** Echo timeout in microseconds (HC-SR04 max range pulse ~38 ms) */
#define HCSR04_TIMEOUT_US         38000UL

/** Out-of-range sentinel value returned on timeout (cm) */
#define HCSR04_OUT_OF_RANGE       (-1.0f)

/** Default speed-of-sound divisor for 20 C (us per cm, round-trip) */
#define HCSR04_DEFAULT_DIVISOR    58.0f

/* --- GPIO / Timer macros (match CubeMX pin assignment) --------------------- */
#define HCSR04_TRIG_PORT          GPIOA
#define HCSR04_TRIG_PIN           GPIO_PIN_9

#define HCSR04_ECHO_PORT          GPIOA
#define HCSR04_ECHO_PIN           GPIO_PIN_8

/* --- Public types ----------------------------------------------------------- */

/**
 * @brief  HC-SR04 driver handle.
 *         Initialise with HCSR04_Init() before use.
 */
typedef struct
{
    TIM_HandleTypeDef *htim;                              /**< 1 us-tick timer handle  */
    float              filter_buf[HCSR04_FILTER_SAMPLES]; /**< Rolling average buffer   */
    uint8_t            filter_idx;                        /**< Current write index      */
    uint8_t            filter_filled;                     /**< Buffer fill flag         */
    float              sound_divisor;                     /**< us / cm (temperature-adj.) */
} HCSR04_HandleTypeDef;

/* --- Public function prototypes -------------------------------------------- */

/**
 * @brief  Initialise the HC-SR04 driver.
 * @param  hdev    Pointer to an HCSR04_HandleTypeDef structure.
 * @param  htim    Pointer to the HAL TIM handle configured for 1 us ticks.
 * @retval HAL_OK on success, HAL_ERROR on failure.
 */
HAL_StatusTypeDef HCSR04_Init(HCSR04_HandleTypeDef *hdev, TIM_HandleTypeDef *htim);

/**
 * @brief  Measure distance once (raw, no filter).
 * @param  hdev    Pointer to initialised HCSR04_HandleTypeDef.
 * @retval Distance in centimetres, or HCSR04_OUT_OF_RANGE on timeout.
 */
float HCSR04_ReadRaw_cm(HCSR04_HandleTypeDef *hdev);

/**
 * @brief  Measure distance with moving-average filter applied.
 * @param  hdev    Pointer to initialised HCSR04_HandleTypeDef.
 * @retval Filtered distance in centimetres, or HCSR04_OUT_OF_RANGE on timeout.
 */
float HCSR04_ReadFiltered_cm(HCSR04_HandleTypeDef *hdev);

/**
 * @brief  Apply temperature compensation to the sound-speed divisor.
 * @param  hdev       Pointer to initialised HCSR04_HandleTypeDef.
 * @param  temp_C     Ambient temperature in degrees C (typically -20 to +60).
 *
 * @note   Formula used: v_sound = 331.3 + 0.606 * T  (m/s)
 *         Divisor (us/cm round-trip) = 20000.0 / v_sound
 */
void HCSR04_SetTemperature(HCSR04_HandleTypeDef *hdev, float temp_C);

/**
 * @brief  Measure raw echo pulse width in microseconds (useful for debugging).
 * @param  hdev    Pointer to initialised HCSR04_HandleTypeDef.
 * @retval Echo pulse width in us, or 0 on timeout.
 */
uint32_t HCSR04_ReadPulse_us(HCSR04_HandleTypeDef *hdev);

#ifdef __cplusplus
}
#endif

#endif /* __HCSR04_H */
