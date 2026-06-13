/**
  ******************************************************************************
  * @file    steering.h
  * @brief   Бортовой поворот электромагнитными катушками.
  *
  *          Катушки управляются изменением силы тока через ШИМ на TIM4
  *          (CH1/CH2 — левый борт, CH3/CH4 — правый). TIM1 (CH1/CH2) задаёт
  *          тайминг профиля тока «рывок-удержание» для срабатывания катушки.
  *          period_left/period_right — текущие значения сравнения ШИМ
  *          (0..PWM_PERIOD), которые формирует логика управления.
  ******************************************************************************
  */

#ifndef __STEERING_H
#define __STEERING_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/* Состояние фазы тока катушки: 0 — удержание, 1/2 — фазы рывка. */
struct coil_status {
  int r;
  int l;
};
extern struct coil_status coil;

/* Текущее значение сравнения ШИМ катушек левого/правого борта. */
extern volatile int period_left;
extern volatile int period_right;

/** Перевести интенсивность 0..100 в значение сравнения ШИМ катушки. */
uint32_t Steering_CalcPeriod(uint8_t value);

/** Шаг автомата катушек: обслуживание профиля тока (вызывать из главного цикла). */
void Steering_Update(void);

/** Обработка событий сравнения TIM1 CH1/CH2 (вызывать из колбэка OC). */
void Steering_OnOcElapsed(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* __STEERING_H */
