/**
  ******************************************************************************
  * @file    inputs.c
  * @brief   Реализация обработки органов управления.
  ******************************************************************************
  */

#include "inputs.h"
#include "config.h"
#include "bsp.h"
#include "io.h"
#include "vesc.h"

unsigned char    Selector       = 'N';
volatile int     switchactivity = 0;

/* Инфраструктура подсчёта нажатий (резерв, измерение по TIM1 CH4). */
volatile int32_t buttonPressCount   = 0;
volatile uint8_t measurementActive  = 0;

/* --------------------------------------------------------------------------
 * Запуск/завершение измерения по TIM1 CH4.
 * -------------------------------------------------------------------------- */
static void Inputs_ArmCh4Window(void)
{
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
  HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
  BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_4, tim1_ch4_pulse);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
}

void Inputs_StartSwitchMeasurement(void)
{
  switchactivity = 1;
  Inputs_ArmCh4Window();
}

void Inputs_StartButtonMeasurement(void)
{
  buttonPressCount   = 1;
  measurementActive  = 1;
  Inputs_ArmCh4Window();
}

void Inputs_OnSwitchTimeout(void)
{
  __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
  HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
  switchactivity = 0;
  erpm = 0;
}

/* --------------------------------------------------------------------------
 * Антидребезг рычагов тормоза.
 *
 * Прим.: в исходном коде «удержание предыдущего состояния» не работало —
 * last_brake_state перезаписывался текущим (нулём) в момент перехода, поэтому
 * выход сразу обнулялся. Здесь подтверждённое состояние хранится отдельно от
 * сырого, что соответствует исходному замыслу (см. комментарии).
 * -------------------------------------------------------------------------- */
uint8_t Inputs_GetBrakeState(void)
{
  static uint8_t  confirmed_state = BRAKE_NONE;
  static uint8_t  last_raw        = BRAKE_NONE;
  static uint32_t raw_change_time = 0;

  uint8_t left  = IO_ReadPin(comp[COMP_BRAKE_LEFT]);
  uint8_t right = IO_ReadPin(comp[COMP_BRAKE_RIGHT]);
  uint8_t raw   = (left ? BRAKE_LEFT : 0) | (right ? BRAKE_RIGHT : 0);

  uint32_t now = HAL_GetTick();
  if (raw != last_raw) {
    raw_change_time = now;
    last_raw = raw;
  }

  if (raw == BRAKE_NONE) {
    /* Подтверждаем нейтраль только после устойчивого таймаута. */
    if (now - raw_change_time >= BRAKE_DEBOUNCE_TIME) {
      confirmed_state = BRAKE_NONE;
    }
    /* иначе удерживаем предыдущее подтверждённое состояние */
  } else {
    /* Любое нажатие — реагируем мгновенно. */
    confirmed_state = raw;
  }

  return confirmed_state;
}

void Inputs_UpdateSelectorNeutral(void)
{
  if (IO_ReadPin(comp[COMP_SEL_D]) == 0 && IO_ReadPin(comp[COMP_SEL_R]) == 0) {
    Selector = 'N';
  }
}

/* --------------------------------------------------------------------------
 * Внешние прерывания концевиков/переключателей.
 * -------------------------------------------------------------------------- */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (!master) {
    return;
  }

  if (GPIO_Pin == COMP_ADC_7_Pin || GPIO_Pin == COMP_ADC_8_Pin) {
    Inputs_StartSwitchMeasurement();
  }
  else if (GPIO_Pin == COMP_ADC_3_Pin) {
    Selector = 'D';
  }
  else if (GPIO_Pin == COMP_ADC_4_Pin) {
    Selector = 'R';
  }
}
