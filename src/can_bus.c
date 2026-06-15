/**
  ******************************************************************************
  * @file    can_bus.c
  * @brief   Реализация обмена по шине CAN.
  ******************************************************************************
  */

#include "can_bus.h"
#include "config.h"
#include "bsp.h"
#include "io.h"
#include "vesc.h"
#include "inputs.h"
#include "steering.h"
#include "control_fsm.h"

/* Заголовки и буферы передачи/приёма. */
static CAN_TxHeaderTypeDef TxHeader_Std;
static CAN_TxHeaderTypeDef TxHeader_Ext;
static uint8_t  TxData_Std[8];
static uint8_t  TxData_Ext[8];
static CAN_RxHeaderTypeDef RxHeader;
static uint8_t  RxData[8];

volatile CAN_Debug_t can_debug = {0};

/* ========================================================================== */
void CanBus_Start(void)
{
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

uint32_t CanBus_GenerateStdId(uint8_t dev_id, uint8_t base_index,
                              uint8_t parameter_index)
{
  return ((uint32_t)dev_id << 8) | (uint32_t)(base_index + parameter_index);
}

void CanBus_SendStd(uint32_t std_id, uint8_t *data, uint8_t length)
{
  TxHeader_Std.StdId = std_id;
  TxHeader_Std.ExtId = 0x00;
  TxHeader_Std.IDE   = CAN_ID_STD;
  TxHeader_Std.RTR   = CAN_RTR_DATA;
  TxHeader_Std.DLC   = length;

  for (uint8_t i = 0; i < length; i++) {
    TxData_Std[i] = data[i];
  }
  for (uint8_t i = length; i < 8; i++) {
    TxData_Std[i] = 0x00;
  }

  uint32_t mailbox;
  HAL_StatusTypeDef result = HAL_CAN_AddTxMessage(&hcan, &TxHeader_Std, TxData_Std, &mailbox);

  switch (result) {
    case HAL_OK:
      can_debug.hal_ok++;
      break;
    case HAL_ERROR:
      can_debug.hal_error++;
      can_debug.last_error_code = result;
      can_debug.last_error_mailbox = mailbox;
      break;
    case HAL_BUSY:
      can_debug.hal_busy++;
      can_debug.last_error_code = result;
      break;
    case HAL_TIMEOUT:
      can_debug.hal_timeout++;
      can_debug.last_error_code = result;
      break;
  }
}

void CanBus_SendExt(uint32_t ext_id, uint8_t *data, uint8_t length)
{
  TxHeader_Ext.ExtId = ext_id;
  TxHeader_Ext.IDE   = CAN_ID_EXT;
  TxHeader_Ext.RTR   = CAN_RTR_DATA;
  TxHeader_Ext.DLC   = length;
  TxHeader_Ext.TransmitGlobalTime = DISABLE;

  for (uint8_t i = 0; i < length; i++) {
    TxData_Ext[i] = data[i];
  }

  uint32_t mailbox;
  HAL_CAN_AddTxMessage(&hcan, &TxHeader_Ext, TxData_Ext, &mailbox);
}

/* --------------------------------------------------------------------------
 * Периодическая рассылка состояния узла.
 *
 * ВАЖНО: у bxCAN всего 3 почтовых ящика передачи. Если за один вызов поставить
 * в очередь больше 3 кадров, лишние получают HAL_BUSY и теряются. Поэтому
 * рассылка разнесена на две фазы (как и в исходном коде):
 *   фаза 0 — comp (1 кадр);
 *   фаза 1 — vesc + akpp + control (3 кадра, ровно умещаются в ящики).
 * Ведомый узел шлёт только comp (остаётся в фазе 0).
 * -------------------------------------------------------------------------- */
void CanBus_TxTask(void)
{
  static uint8_t send_phase = 0;

  if (send_phase == 0) {
    /* Состояние дискретных входов. */
    uint8_t data_comp[8];
    for (uint8_t i = 0; i < 8; i++) {
      data_comp[i] = IO_ReadPin(comp[i]);
    }
    CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_COMP, COMP_COUNT), data_comp, 8);

    send_phase = master ? 1 : 0;
  }
  else {
    /* Обороты мотора складывания. */
    Vesc_SendRpm();

    /* Положение селектора АКПП. */
    uint8_t data_akpp[1] = { (uint8_t)Selector };
    CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_AKPP, AKPP_COUNT), data_akpp, 1);

    /* Команды интенсивности на борта. */
    uint8_t data_control[4] = {
      (uint8_t)control.f_r,
      (uint8_t)control.f_l,
      (uint8_t)control.b_r,
      (uint8_t)control.b_l
    };
    CanBus_SendStd(CanBus_GenerateStdId(device_id, BASE_CONTROL, CONTROL_COUNT), data_control, 4);

    send_phase = 0;
  }
}

/* --------------------------------------------------------------------------
 * Приём команд в режиме ведомого.
 * -------------------------------------------------------------------------- */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_rx)
{
  if (HAL_CAN_GetRxMessage(hcan_rx, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) {
    return;
  }
  if (RxHeader.IDE == CAN_ID_EXT) {
    return;  /* расширенные кадры (VESC и т.п.) не обрабатываем */
  }

  uint8_t std_device_id  = (RxHeader.StdId >> 8) & 0xFF;
  uint8_t parameter_index =  RxHeader.StdId       & 0xFF;

  if (master) {
    return;  /* ведущий команды не принимает */
  }

  if (std_device_id != 0x01) {
    return;
  }

  if (parameter_index == BASE_CONTROL + 1) {
    period_left  = Steering_CalcPeriod(RxData[2]);
    period_right = Steering_CalcPeriod(RxData[3]);
  }
  else if (parameter_index == BASE_COMP + 1) {
    uint8_t left_turn  = RxData[0];
    uint8_t right_turn = RxData[1];

    HAL_GPIO_WritePin(GPIOB, EN_RELAY_1_Pin, (left_turn  == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, (right_turn == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    /* Аварийная сигнализация: оба поворотника -> стоп-сигнал, поворотники off. */
    if (left_turn && right_turn) {
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_4_Pin, GPIO_PIN_SET);
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_1_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, GPIO_PIN_RESET);
    } else {
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_4_Pin, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_1_Pin, (left_turn  == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, (right_turn == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
  }
  else if (parameter_index == BASE_AKPP + AKPP_COUNT) {
    /* Задний ход. */
    HAL_GPIO_WritePin(GPIOB, EN_RELAY_3_Pin,
                      (RxData[0] == 'R') ? GPIO_PIN_SET : GPIO_PIN_RESET);
  }
}
