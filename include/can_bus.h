/**
  ******************************************************************************
  * @file    can_bus.h
  * @brief   Обмен по шине CAN: отправка прикладных и VESC-кадров, приём команд
  *          в режиме ведомого, периодическая рассылка состояния.
  ******************************************************************************
  */

#ifndef __CAN_BUS_H
#define __CAN_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* Счётчики результатов отправки для диагностики. */
typedef struct {
  uint32_t hal_ok;
  uint32_t hal_error;
  uint32_t hal_busy;
  uint32_t hal_timeout;
  uint32_t last_error_code;
  uint32_t last_error_mailbox;
} CAN_Debug_t;

extern volatile CAN_Debug_t can_debug;

/** Запустить CAN и включить уведомление о приёме (FIFO0). */
void CanBus_Start(void);

/** Сформировать стандартный StdId: (device_id << 8) | (base + index). */
uint32_t CanBus_GenerateStdId(uint8_t dev_id, uint8_t base_index,
                              uint8_t parameter_index);

/** Отправить кадр со стандартным (11-бит) идентификатором. */
void CanBus_SendStd(uint32_t std_id, uint8_t *data, uint8_t length);

/** Отправить кадр с расширенным (29-бит) идентификатором (для VESC). */
void CanBus_SendExt(uint32_t ext_id, uint8_t *data, uint8_t length);

/** Периодическая рассылка состояния узла (вызывается из прерывания TIM1 CC3). */
void CanBus_TxTask(void);

#ifdef __cplusplus
}
#endif

#endif /* __CAN_BUS_H */
