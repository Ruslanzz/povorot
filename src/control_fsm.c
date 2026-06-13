/**
  ******************************************************************************
  * @file    control_fsm.c
  * @brief   Реализация центрального автомата режимов движения.
  ******************************************************************************
  */

#include "control_fsm.h"
#include "config.h"
#include "io.h"
#include "inputs.h"
#include "vesc.h"
#include "steering.h"

struct control_status control = {0, 0, 0, 0};
DriveState drive_state = DRIVE_DISABLED;

/* Признак активного центрирования рамы (сохраняется между шагами). */
static int steering_centering = 0;

/* --------------------------------------------------------------------------
 * Обработчики режимов. Каждый выставляет интенсивности на борта (control) и,
 * при необходимости, целевые обороты мотора складывания (erpm).
 * -------------------------------------------------------------------------- */

static void State_BrakeBoth(uint16_t adc_angle)
{
  control.f_l = PERIOD_BRAKE;
  control.b_l = PERIOD_BRAKE;
  control.f_r = PERIOD_BRAKE;
  control.b_r = PERIOD_BRAKE;

  if (switchactivity == 0) {
    steering_centering = 1;
    erpm = Vesc_GetErpm(adc_angle, CONTROL_CENTER);
  }
}

static void State_Neutral(uint16_t adc_angle)
{
  int angle_diff = (int)adc_angle - CENTER_ANGLE;

  /* Пока активно центрирование рамы — продолжаем его, иначе мотор стоп. */
  if (steering_centering == 1) {
    erpm = Vesc_GetErpm(adc_angle, CONTROL_CENTER);
  } else {
    erpm = 0;
  }

  /* Дифференциал по бортам в зависимости от отклонения рамы. */
  if (angle_diff > 0) {
    float period_calc = ((float)PERIOD_DRIVE / (float)(RIGHT_ANGLE - CENTER_ANGLE)) *
                        (float)((RIGHT_ANGLE - CENTER_ANGLE) - angle_diff);
    if (period_calc > 100) {
      period_calc = 100;
    }
    control.f_l = PERIOD_DRIVE;
    control.b_l = PERIOD_DRIVE;
    control.f_r = (int)period_calc;
    control.b_r = (int)period_calc;
  }
  else if (angle_diff < 0) {
    float period_calc = ((float)PERIOD_DRIVE / (float)(LEFT_ANGLE - CENTER_ANGLE)) *
                        (float)((LEFT_ANGLE - CENTER_ANGLE) - angle_diff);
    if (period_calc > 100) {
      period_calc = 100;
    }
    control.f_l = (int)period_calc;
    control.b_l = (int)period_calc;
    control.f_r = PERIOD_DRIVE;
    control.b_r = PERIOD_DRIVE;
  }
}

static void State_BortLeft(uint16_t adc_angle)
{
  control.f_l = 0;
  control.b_l = 0;
  control.f_r = PERIOD_BORT;
  control.b_r = PERIOD_BORT;
  steering_centering = 0;

  if (switchactivity == 0) {
    erpm = Vesc_GetErpm(adc_angle, CONTROL_LEFT);
  }
}

static void State_BortRight(uint16_t adc_angle)
{
  control.f_l = PERIOD_BORT;
  control.b_l = PERIOD_BORT;
  control.f_r = 0;
  control.b_r = 0;
  steering_centering = 0;

  if (switchactivity == 0) {
    erpm = Vesc_GetErpm(adc_angle, CONTROL_RIGHT);
  }
}

/* --------------------------------------------------------------------------
 * Один шаг автомата.
 * -------------------------------------------------------------------------- */
void ControlFSM_Update(uint16_t adc_angle)
{
  Inputs_UpdateSelectorNeutral();

  /* Вход deadman: при отсутствии разрешения мотор складывания останавливаем,
   * интенсивности на бортах сохраняем (как в исходной логике). */
  if (IO_ReadPin(comp[COMP_DEADMAN]) == 0) {
    drive_state = DRIVE_DISABLED;
    erpm = 0;
  }
  else {
    switch (Inputs_GetBrakeState()) {
      case BRAKE_BOTH:
        drive_state = DRIVE_BRAKE_BOTH;
        State_BrakeBoth(adc_angle);
        break;

      case BRAKE_NONE:
        drive_state = DRIVE_NEUTRAL;
        State_Neutral(adc_angle);
        break;

      case BRAKE_LEFT:
        drive_state = DRIVE_BORT_LEFT;
        State_BortLeft(adc_angle);
        break;

      case BRAKE_RIGHT:
        drive_state = DRIVE_BORT_RIGHT;
        State_BortRight(adc_angle);
        break;

      default:
        control.f_l = 0;
        control.b_l = 0;
        control.f_r = 0;
        control.b_r = 0;
        break;
    }
  }

  /* Пересчёт значений сравнения ШИМ катушек из интенсивностей бортов. */
  period_left  = Steering_CalcPeriod(control.f_l);
  period_right = Steering_CalcPeriod(control.f_r);
}
