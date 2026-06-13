/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Точка входа и главный цикл прошивки бортоповоротного
  *                   вездехода со складываемой рамой.
  *
  *  Архитектура (см. отдельные модули):
  *    bsp         — инициализация периферии STM32F103 и помощники
  *    io          — дискретные входы/реле
  *    inputs      — рычаги тормоза, селектор, переключатели
  *    vesc        — мотор складывания рамы (по CAN)
  *    steering    — катушки бортового поворота (ШИМ TIM1/TIM4)
  *    can_bus     — обмен по шине CAN
  *    control_fsm — центральный автомат режимов движения
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "bsp.h"
#include "config.h"
#include "io.h"
#include "inputs.h"
#include "vesc.h"
#include "steering.h"
#include "can_bus.h"
#include "control_fsm.h"

/**
  * @brief  The application entry point.
  */
int main(void)
{
  BSP_Init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  /* Вывести трансивер CAN из режима ожидания и запустить шину. */
  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin, GPIO_PIN_RESET);
  CanBus_Start();

  /* Запуск ШИМ катушек поворота и исходное состояние полумостов. */
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_PERIOD);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_B_Pin, GPIO_PIN_SET);
  period_left  = PWM_PERIOD;
  period_right = PWM_PERIOD;

  /* TIM1: база + канал CH3 — периодический тик рассылки CAN. */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);

  /* Запуск непрерывного измерения АЦП по DMA (8 каналов). */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADS_RES_BUFFER, 8);

  /* Общее реле питания. */
  HAL_GPIO_WritePin(GPIOB, EN_RELAY_5_Pin, GPIO_PIN_SET);

  while (1)
  {
    uint16_t adc_angle = (uint16_t)ADS_RES_BUFFER[0];   /* датчик угла рамы */

    if (master) {
      ControlFSM_Update(adc_angle);
    }

    /* Обслуживание профиля тока катушек (на ведущем и ведомом). */
    Steering_Update();
  }
}

/* ==========================================================================
 * Колбэки и обработчики прерываний прикладного уровня.
 * ========================================================================== */

/** Прерывание сравнения TIM1. */
void TIM1_CC_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&htim1);
}

/** Диспетчер событий Output Compare TIM1 по каналам. */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM1) {
    return;
  }

  switch (htim->Channel) {
    case HAL_TIM_ACTIVE_CHANNEL_1:
    case HAL_TIM_ACTIVE_CHANNEL_2:
      Steering_OnOcElapsed(htim);
      break;

    case HAL_TIM_ACTIVE_CHANNEL_3:
      /* Периодическая рассылка состояния и перепланирование следующего тика. */
      CanBus_TxTask();
      BSP_TimerAdvanceCompare(&htim1, TIM_CHANNEL_3, tim1_ch3_pulse);
      break;

    case HAL_TIM_ACTIVE_CHANNEL_4:
      Inputs_OnSwitchTimeout();
      break;

    default:
      break;
  }
}
