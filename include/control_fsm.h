/**
  ******************************************************************************
  * @file    control_fsm.h
  * @brief   Центральный автомат режимов движения вездехода.
  *
  *          Входы: рычаги тормоза (через inputs), датчик угла рамы (АЦП),
  *          разрешающий вход deadman. Выходы-исполнители: интенсивности на
  *          борта (control -> катушки поворота) и команда мотору складывания
  *          рамы (erpm -> VESC).
  ******************************************************************************
  */

#ifndef __CONTROL_FSM_H
#define __CONTROL_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Интенсивности (0..100) на передний/задний, левый/правый борт. */
struct control_status {
  int f_r;
  int f_l;
  int b_r;
  int b_l;
};
extern struct control_status control;

/* Режимы движения. */
typedef enum {
  DRIVE_DISABLED = 0,  /* вход deadman не активен — мотор складывания стоп   */
  DRIVE_NEUTRAL,       /* рычаги отпущены — движение прямо/дифференциал по углу */
  DRIVE_BORT_LEFT,     /* нажат левый рычаг — бортовой поворот влево         */
  DRIVE_BORT_RIGHT,    /* нажат правый рычаг — бортовой поворот вправо       */
  DRIVE_BRAKE_BOTH     /* оба рычага — торможение и центрирование рамы       */
} DriveState;

extern DriveState drive_state;

/** Один шаг автомата управления. @p adc_angle — показание датчика угла рамы. */
void ControlFSM_Update(uint16_t adc_angle);

#ifdef __cplusplus
}
#endif

#endif /* __CONTROL_FSM_H */
