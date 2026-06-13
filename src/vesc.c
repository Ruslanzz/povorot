/**
  ******************************************************************************
  * @file    vesc.c
  * @brief   Реализация управления мотором складывания рамы (VESC).
  ******************************************************************************
  */

#include "vesc.h"
#include "config.h"
#include "can_bus.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
#include <stdlib.h>

/* Типы команд VESC (используется только SET_RPM). */
typedef enum {
  CAN_PACKET_SET_CURRENT = 1,
  CAN_PACKET_SET_RPM     = 3,
  CAN_PACKET_SET_POS     = 4,
  CAN_PACKET_SET_DUTY    = 5,
} VESC_CMD_t;

volatile int32_t erpm = 0;

/* Состояние плавного изменения оборотов. */
static int32_t        current_erpm        = 0;
static uint32_t       last_update_time    = 0;
static uint32_t       command_change_time = 0;
static ControlCommand last_cmd            = CONTROL_CENTER;

/**
 * @brief  Плавно подвести текущие обороты к целевым.
 *
 *         При смене команды сбрасывает обороты в 0 и на короткое окно
 *         (EMERGENCY_RAMP_MS) включает увеличенный шаг — экстренное торможение.
 *         Не допускает «перескока» через ноль при смене знака.
 *
 * @param  target_erpm целевые обороты (-MAX_RPM, 0, +MAX_RPM)
 * @param  cmd         текущая команда направления
 * @return текущие обороты с учётом плавности
 */
static int32_t Vesc_SmoothErpm(int32_t target_erpm, ControlCommand cmd)
{
  uint32_t current_time = HAL_GetTick();

  if (cmd != last_cmd) {
    last_cmd = cmd;
    command_change_time = current_time;
    current_erpm = 0;
  }

  if (current_time - last_update_time >= RAMP_INTERVAL_MS) {
    last_update_time = current_time;

    uint32_t time_since_change = current_time - command_change_time;
    bool is_emergency = (time_since_change < EMERGENCY_RAMP_MS);

    int32_t diff = target_erpm - current_erpm;
    int32_t step = is_emergency ? RAMP_STEP * 5 : RAMP_STEP;

    if (abs(diff) <= step) {
      current_erpm = target_erpm;
    }
    else if (diff > 0) {
      current_erpm += step;
      if ((target_erpm > 0) && (current_erpm > target_erpm)) current_erpm = target_erpm;
      if ((target_erpm < 0) && (current_erpm > 0)) current_erpm = 0; /* стоп перед сменой знака */
    }
    else {
      current_erpm -= step;
      if ((target_erpm < 0) && (current_erpm < target_erpm)) current_erpm = target_erpm;
      if ((target_erpm > 0) && (current_erpm < 0)) current_erpm = 0; /* стоп перед сменой знака */
    }
  }

  return current_erpm;
}

int32_t Vesc_GetErpm(uint16_t adc_value, ControlCommand cmd)
{
  int32_t result = 0;

  switch (cmd) {
    case CONTROL_CENTER:
      /* Едем к центру: выше центра — назад, ниже — вперёд. */
      if (abs((int)adc_value - CENTER_ANGLE) > ANGLE_TOLERANCE) {
        if (adc_value > CENTER_ANGLE) {
          result = Vesc_SmoothErpm(-MAX_RPM, cmd);
        } else {
          result = Vesc_SmoothErpm(MAX_RPM, cmd);
        }
      } else {
        result = 0;
      }
      break;

    case CONTROL_LEFT:
      /* Складываем влево, пока не достигнут левый упор. */
      if (adc_value >= LEFT_ANGLE) {
        result = 0;   /* достигли упора — стоп */
      } else {
        result = Vesc_SmoothErpm(MAX_RPM, cmd);
      }
      break;

    case CONTROL_RIGHT:
      /* Складываем вправо, пока не достигнут правый упор. */
      if (adc_value <= RIGHT_ANGLE) {
        result = 0;   /* достигли упора — стоп */
      } else {
        result = Vesc_SmoothErpm(-MAX_RPM, cmd);
      }
      break;

    case CONTROL_OFF:
    default:
      result = Vesc_SmoothErpm(0, cmd);
      break;
  }

  return result;
}

void Vesc_SendRpm(void)
{
  uint8_t data_vesc[4];

  /* 32-битные обороты в порядке big-endian. */
  data_vesc[0] = (erpm >> 24) & 0xFF;
  data_vesc[1] = (erpm >> 16) & 0xFF;
  data_vesc[2] = (erpm >> 8)  & 0xFF;
  data_vesc[3] =  erpm        & 0xFF;

  uint32_t can_id = (CAN_PACKET_SET_RPM << 8) | VESC_CAN_ID;
  CanBus_SendExt(can_id, data_vesc, 4);
}
