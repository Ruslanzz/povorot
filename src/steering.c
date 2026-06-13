/**
  ******************************************************************************
  * @file    steering.c
  * @brief   Реализация управления катушками бортового поворота.
  *
  *          Профиль тока «рывок-удержание»: при необходимости включить катушку
  *          сначала подаётся полный ШИМ (рывок) на время tim1_chX_pulse, затем
  *          ток снижается до удержания (period_left/period_right). Состояние
  *          фазы хранится в coil.l/coil.r.
  ******************************************************************************
  */

#include "steering.h"
#include "config.h"
#include "bsp.h"

struct coil_status coil = {0, 0};

volatile int period_left  = 0;
volatile int period_right = 0;

uint32_t Steering_CalcPeriod(uint8_t value)
{
  uint32_t percentage = 100 - value;
  return (uint32_t)((PWM_PERIOD / 100) * percentage);
}

/* --------------------------------------------------------------------------
 * Левый борт: TIM4 CH1/CH2, тайминг TIM1 CH1.
 * -------------------------------------------------------------------------- */
static void Steering_UpdateLeft(void)
{
  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) != RESET) {
    return;  /* тайминг катушки уже активен */
  }

  if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2) == 0) &&
      (period_left < PWM_PERIOD) && (coil.l == 0)) {
    /* Старт рывка: полный ток по второй полуобмотке. */
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
    coil.l = 1;
    BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_1, tim1_ch1_pulse);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
  }

  if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2) > 0) && (coil.l == 0)) {
    if (period_left == PWM_PERIOD) {
      /* Рывок в обратную сторону при полном запросе. */
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
      coil.l = 2;
      BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_1, tim1_ch1_pulse);
      HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
    }
    if (period_left < PWM_PERIOD) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period_left);
    }
  }
}

/* --------------------------------------------------------------------------
 * Правый борт: TIM4 CH3/CH4, тайминг TIM1 CH2.
 * -------------------------------------------------------------------------- */
static void Steering_UpdateRight(void)
{
  if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC2) != RESET) {
    return;
  }

  if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4) == 0) &&
      (period_right < PWM_PERIOD) && (coil.r == 0)) {
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, PWM_PERIOD);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
    coil.r = 1;
    BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_2, tim1_ch2_pulse);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
  }

  if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4) > 0) && (coil.r == 0)) {
    if (period_right == PWM_PERIOD) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
      coil.r = 2;
      BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_2, tim1_ch2_pulse);
      HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
    }
    if (period_right < PWM_PERIOD) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period_right);
    }
  }
}

void Steering_Update(void)
{
  Steering_UpdateLeft();
  Steering_UpdateRight();
}

void Steering_OnOcElapsed(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1) {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
    if (coil.l == 1) {
      /* Конец рывка -> переход на удержание. */
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period_left);
      coil.l = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
    } else if (coil.l == 2) {
      coil.l = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
    }
  }

  if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2) {
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
    if (coil.r == 1) {
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period_right);
      coil.r = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
    } else if (coil.r == 2) {
      coil.r = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
    }
  }
}
