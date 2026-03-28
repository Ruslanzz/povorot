/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#define BASE_RELAY        0x00
#define BASE_ADC1         0x10
#define BASE_ADC2         0x20
#define BASE_CONTROL      0x30
#define BASE_COMP         0x40
#define BASE_RELAY_OUT    0x50
#define BASE_AKPP         0x60

#define RELAY_COUNT       5
#define ADC1_COUNT        2
#define ADC2_COUNT        2
#define CONTROL_COUNT     1
#define COMP_COUNT        1
#define RELAY_OUT_COUNT   1
#define AKPP_COUNT        1

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
CAN_HandleTypeDef hcan;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint32_t ADS_RES_BUFFER[8];
CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8]; // Буфер для данных CAN-сообщени
volatile int32_t buttonPressCount = 0;
volatile uint8_t measurementActive = 0;
volatile uint8_t firstPressDetected = 0; // Флаг первого нажатия
unsigned char Selector = 'N';
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
const int center_angle=652; //638
const int left_angle=1000; //1163
const int right_angle=200; //186
int angle_diff;
int adc_b0=0;
int period_brake=50;
int period_drive=60;
int period_bort=80;
float period_calc;
int period_left;
int period_right;
int period = 20000;
GPIO_PinState left_brake;
GPIO_PinState right_brake;
volatile uint32_t tim1_ch1_pulse = 2000 - 1;
volatile uint32_t tim1_ch2_pulse = 2000 - 1;
volatile uint32_t tim1_ch3_pulse = 2000 - 1;
volatile uint32_t tim1_ch4_pulse = 20000 - 1;
int switchactivity = 0;
int emergency_stop_right = 0;
int emergency_stop_left = 0;
int32_t rpm_calc = 0;
int32_t erpm;

typedef enum {
  BTN_IDLE,
  BTN_PRESSED,
  BTN_DEBOUNCE
} ButtonState;

static ButtonState btn_state = BTN_IDLE;
static uint32_t btn_timestamp = 0;
// Глобальные переменные для антидребезга
const uint32_t DEBOUNCE_DELAY_MS = 40; // Оптимальное время для большинства кнопок
volatile uint32_t last_button_press_time = 0;
volatile uint8_t button_debounce_flag = 0;
volatile uint8_t button_valid_press = 0;

// Типы команд VESC
typedef enum {
  CAN_PACKET_SET_POS = 4,           // Установка позиции
  CAN_PACKET_SET_CURRENT = 1,       // Установка тока
  CAN_PACKET_SET_RPM = 3,           // Установка оборотов
  CAN_PACKET_SET_DUTY = 5,          // Установка заполнения
} VESC_CMD_t;

struct coil_status
{
   int r;
   int l;  
};
struct coil_status coil;

struct control_status
{
   int f_r;
   int f_l;
   int b_r;
   int b_l;   
};
struct control_status control = {0,0,0,0};

typedef struct {
    GPIO_TypeDef* port;  // Указатель на порт GPIO
    uint16_t pin;        // Номер пина
} GPIO_Config;

GPIO_Config comp[] = {
{ GPIOB, COMP_ADC_1_Pin },
{ GPIOB, COMP_ADC_2_Pin },
{ GPIOB, COMP_ADC_3_Pin },
{ GPIOB, COMP_ADC_4_Pin },
{ GPIOC, COMP_ADC_5_Pin },
{ GPIOC, COMP_ADC_6_Pin },
{ GPIOC, COMP_ADC_7_Pin },
{ GPIOA, COMP_ADC_8_Pin }
};

GPIO_Config relay[] = {
{ GPIOB, EN_RELAY_1_Pin },
{ GPIOB, EN_RELAY_2_Pin },
{ GPIOB, EN_RELAY_3_Pin },
{ GPIOB, EN_RELAY_4_Pin },
{ GPIOB, EN_RELAY_5_Pin }
};

bool master = true;
uint8_t device_id = 0x01;


typedef enum {
    FOLD_IDLE = 0,
    FOLD_MOVING_LEFT,
    FOLD_MOVING_RIGHT,
    FOLD_RETURNING_TO_CENTER, 
    FOLD_EMERGENCY_STOP
} SystemState;

typedef struct {
    SystemState state;
    uint8_t left_lever_active;
    uint8_t right_lever_active;
    int current_frame_angle;     // Текущий угол рамы из ADC
    uint32_t center_return_timer; // Таймер для задержки возврата в центр
} FoldingSystem;

FoldingSystem folding_sys = {0};

#define VESC_CAN_ID 60

// Граничные углы для защиты от перескладывания
#define MIN_ANGLE left_angle    // Минимальный угол (186)
#define MAX_ANGLE right_angle   // Максимальный угол (1163)
#define SAFETY_MARGIN 20        // Запас до границ


#define LEFT_LIMIT      186
#define CENTER          638
#define RIGHT_LIMIT     1163
#define DEADZONE        10      // Мертвая зона центра
#define SAFETY_MARGIN   5       // Запас до края

// Параметры скорости
#define MAX_RPM  4000   // Максимальные обороты
#define MIN_RPM_EDGE    0     // Минимальные обороты (при малом отклонении)


/* USER CODE END PFP */
// Функция для чтения состояния пина с использованием структуры
int Read_GPIO_Pin(GPIO_Config config) {
  return (HAL_GPIO_ReadPin(config.port, config.pin) == GPIO_PIN_SET) ? 1 : 0;
}


// Функция для включения реле по индексу
void turn_on_relay(uint8_t relay_index) {
  if (relay_index < RELAY_COUNT) {
      HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_SET);
  }
}

// Функция для выключения реле по индексу
void turn_off_relay(uint8_t relay_index) {
  if (relay_index < RELAY_COUNT) {
      HAL_GPIO_WritePin(relay[relay_index].port, relay[relay_index].pin, GPIO_PIN_RESET);
  }
}

// Генерация stdid
uint32_t generate_stdid(uint8_t device_id, uint8_t base_index, uint8_t parameter_index) {
  return ((uint32_t)device_id << 8) | (uint32_t)(base_index + parameter_index);
}

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Функция запуска измерения
void StartButtonMeasurement()
{
    buttonPressCount = 1;
    measurementActive = 1;
    // Получение текущих значений
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
    uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    uint32_t timer_period = htim1.Instance->ARR;
    
    // Расчет нового значения с защитой от переполнения
    uint32_t new_compare = current_compare + tim1_ch4_pulse;
    if (new_compare > timer_period) {
        new_compare -= timer_period;
        
        // Дополнительная корректировка если нужно
        new_compare = new_compare % (timer_period + 1);
    }
    
    // Установка нового значения
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, new_compare);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4); // Запускаем таймер
}

void StartSwitchMeasurement()
{
    switchactivity = 1;
    // Получение текущих значений
    __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
    HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
    uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4);
    uint32_t timer_period = htim1.Instance->ARR;
    
    // Расчет нового значения с защитой от переполнения
    uint32_t new_compare = current_compare + tim1_ch4_pulse;
    if (new_compare > timer_period) {
        new_compare -= timer_period;
        
        // Дополнительная корректировка если нужно
        new_compare = new_compare % (timer_period + 1);
    }
    
    // Установка нового значения
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, new_compare);
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4); // Запускаем таймер
}

// Обработчик EXTI (считаем нажатия)
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  
  if (master == true) {
    if (GPIO_Pin == COMP_ADC_7_Pin)
    {
      StartSwitchMeasurement();
     // vesc_set_rpm(60, -100);
    }
    if (GPIO_Pin == COMP_ADC_8_Pin)
    {
      StartSwitchMeasurement();
     // vesc_set_rpm(60, 100);
    }
    if (GPIO_Pin == COMP_ADC_3_Pin)
    {
      Selector = 'D';
    //   uint32_t now = HAL_GetTick();

    //    // Быстрая проверка антидребезга (50ms)
    //    if (now - last_button_press_time < DEBOUNCE_DELAY_MS) {
    //     button_debounce_flag = 1;  // Флаг дребезга
    //     return;
    // }
    
    // // Если дошли сюда - нажатие валидное
    // last_button_press_time = now;
    // button_valid_press = 1;
    // button_debounce_flag = 0;
    
    // // Обработка состояний
    // if (!measurementActive) {
    //     if (!firstPressDetected) {
    //         firstPressDetected = 1;
    //         StartButtonMeasurement();
    //     }
    // } else {
    //     // Обработка повторных нажатий
    //     uint32_t new_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_4) + tim1_ch4_pulse;
    //     if (new_compare > htim1.Instance->ARR) {
    //         new_compare = new_compare % (htim1.Instance->ARR + 1);
    //     }
    //     __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, new_compare);
    //     buttonPressCount++;
    // }
    }
    if (GPIO_Pin == COMP_ADC_4_Pin)
    {   
        // if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC4) == RESET) {
        //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
        // }
        // measurementActive = 0; // Останавливаем подсчет
        // firstPressDetected = 0;
        // buttonPressCount = 0;
        Selector = 'R';
    }
  }  
}


uint32_t CalculatePeriod(uint8_t DataValue)
{
    // Вычисляем процентное значение
    uint32_t percentage = 100 - DataValue;
    // Вычисляем period_left
    int period_calc = (period/100) * percentage;

    return period_calc;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{        
  // CAN_RxHeaderTypeDef RxHeader;
  // uint8_t RxData[8]; // Буфер для данных CAN-сообщения

  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    uint32_t stdid = RxHeader.StdId;
    // Извлекаем device_id и parameter_index из stdid
    uint8_t std_device_id = (stdid >> 8) & 0xFF; // Старшие 8 бит
    uint8_t parameter_index = stdid & 0xFF;   // Младшие 8 бит
    
    // if (std_device_id == device_id) {
    //   if (parameter_index >= BASE_RELAY && parameter_index < BASE_RELAY + RELAY_COUNT) {
    //     // Обработка реле
    //     uint8_t relay_index = parameter_index - BASE_RELAY;
    //     if (RxData[0] == 1) {          
    //       turn_on_relay(relay_index);
    //     }
    //     else {
    //       turn_off_relay(relay_index);  
    //     } 
    //   }        
    // }
    if (master == false){
    if (std_device_id == 1 ) {         
      if (parameter_index == BASE_CONTROL + 1) {
        period_left = CalculatePeriod(RxData[2]);  
        period_right = CalculatePeriod(RxData[3]);                          
        // HAL_GPIO_WritePin(GPIOB, EN_RELAY_1_Pin, (RxData[0] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, (RxData[1] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOB, EN_RELAY_3_Pin, (RxData[2] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(GPIOB, EN_RELAY_4_Pin, (RxData[3] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);         
        }
      
      if (parameter_index == BASE_COMP + 1) {
        HAL_GPIO_WritePin(GPIOB, EN_RELAY_1_Pin, (RxData[0] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);//левый поворот
        HAL_GPIO_WritePin(GPIOB, EN_RELAY_2_Pin, (RxData[1] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);//правый поворот
        HAL_GPIO_WritePin(GPIOB, EN_RELAY_3_Pin, (RxData[3] == 1) ? GPIO_PIN_SET : GPIO_PIN_RESET); //задний ход

      } 
    }              
    }
  }
}

void CAN_SendMessage(uint32_t StdId, uint8_t* data, uint8_t dataLength) {
    uint32_t TxMailbox = 0;
    // Заголовок CAN-сообщения
    TxHeader.StdId = StdId;       // Идентификатор сообщения (передаётся как параметр)
    TxHeader.ExtId = 0x00;        // Расширенный идентификатор (не используется)
    TxHeader.IDE = CAN_ID_STD;    // Стандартный идентификатор
    TxHeader.RTR = CAN_RTR_DATA;  // Тип сообщения (данные)
    TxHeader.DLC = dataLength;    // Длина данных (передаётся как параметр)

    // Копирование данных в TxData
    for (uint8_t i = 0; i < dataLength; i++) {
        TxData[i] = data[i];  // Копируем данные из переданного массива
    }

    // Очистка оставшихся байтов (если dataLength < 8)
    if (dataLength < 8){
      for (uint8_t i = dataLength; i < 8; i++) {
          TxData[i] = 0x00;
      }
    }

    // Отправка сообщения
    if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK) {
        // Обработка ошибки отправки
        //Error_Handler();
    }
}
void CAN_SendMessage_VESC() {
  uint8_t data[4];
  
  // Упаковка 32-битного числа в big-endian
  data[0] = (erpm >> 24) & 0xFF;
  data[1] = (erpm >> 16) & 0xFF;
  data[2] = (erpm >> 8) & 0xFF;
  data[3] = erpm & 0xFF;
  
  // Формирование CAN ID: (команда << 8) | ID контроллера
  uint32_t can_id = (CAN_PACKET_SET_RPM << 8) | VESC_CAN_ID;
  
  CAN_TxHeaderTypeDef tx_header;
  uint32_t tx_mailbox;
  
  tx_header.ExtId = can_id;
  tx_header.IDE = CAN_ID_EXT;      // Расширенный ID (29 бит)
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = 4;
  tx_header.TransmitGlobalTime = DISABLE;
  
  HAL_CAN_AddTxMessage(&hcan, &tx_header, data, &tx_mailbox);
}

void CreateCANMessages() {
  uint8_t data_comp[8];
  uint32_t stdid;            
  for (uint8_t i = 0; i < 8; i++) {
      data_comp[i] = Read_GPIO_Pin(comp[i]);
  }            
  stdid = generate_stdid(device_id, BASE_COMP, COMP_COUNT);      
  CAN_SendMessage(stdid, data_comp, 8);

  if (master == true) {
      CAN_SendMessage_VESC();
      // Отправка данных селектора АКПП
      uint8_t data_akpp[1] = {Selector};
      uint32_t stdid = generate_stdid(device_id, BASE_AKPP, AKPP_COUNT);      
      CAN_SendMessage(stdid, data_akpp, 1);  

      // Отправка контрольных данных
      uint8_t data_control[4] = {
          control.f_r,  // TxData[0]
          control.f_l,  // TxData[1]
          control.b_r,  // TxData[2]
          control.b_l   // TxData[3]
      };
      stdid = generate_stdid(device_id, BASE_CONTROL, CONTROL_COUNT);
      CAN_SendMessage(stdid, data_control, 4);
  }
}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)

{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_CAN_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  HAL_GPIO_WritePin(GPIOA, CAN_STB_Pin, GPIO_PIN_RESET);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING); 
  /* USER CODE END 2 */ 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */ 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_A_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_B_Pin, GPIO_PIN_SET);  
  period_left = period;
  period_right = period;
  
  // HAL_TIM_Base_Start(&htim1);
  // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADS_RES_BUFFER, 8);
  

  while (1)
  { 
      __WFI(); // Wait for interrupt (энергоэффективно)
       //  /* USER CODE END WHILE */
      // HAL_ADC_Start_DMA(&hadc1, (uint32_t*)ADS_RES_BUFFER, 8);
      adc_b0 = (ADS_RES_BUFFER[0]);      
      if (master == true) {
          angle_diff = adc_b0 - center_angle;
          if (Read_GPIO_Pin(comp[2]) == 0 && Read_GPIO_Pin(comp[3]) == 0) {              
            Selector = 'N';
          } 

          left_brake = Read_GPIO_Pin(comp[0]); 
          right_brake = Read_GPIO_Pin(comp[1]);
          if (left_brake && right_brake) {
            control.f_l = control.b_l = control.f_r = control.b_r = period_brake;
            if (switchactivity == 0) {
              erpm = 0;
            }
          } 
          else if (!left_brake && !right_brake) {
              if (switchactivity == 0) {
                erpm = 0;
              }
              if(angle_diff > 0) {
                period_calc = ((float)period_drive/(float)(right_angle-center_angle))*(float)((right_angle-center_angle)-angle_diff);
                control.f_l = control.b_l = period_drive;                   
                control.f_r = control.b_r = period_calc;                     
              } else if (angle_diff < 0) {
               period_calc = ((float)period_drive/(float)(left_angle-center_angle))*(float)((left_angle-center_angle)-angle_diff);
                              //(60/1163-638)*(1163-638+0)
                control.f_l = control.b_l = period_calc;                     
                control.f_r = control.b_r = period_drive;                      
              }                 
          } 
              if (!right_brake && left_brake) {   
                  control.f_l = control.b_l = period_bort;
                  control.f_r = control.b_r = 0;
                  if (switchactivity == 0) {
                    if (adc_b0 > center_angle ) {
                      float rpm_mechanical = ((float)MAX_RPM/(float)(left_angle-center_angle))*(float)((left_angle-center_angle)-angle_diff);
                                          //(200/(1087-652))*((1087-652)-435)=  
                      erpm = (int32_t)(rpm_mechanical);

                    }
                    if (adc_b0 >= left_angle)
                    {
                      erpm = 0;
                    }
                    if (adc_b0 < center_angle) {
                      erpm = MAX_RPM;

                    }
                  }
              } else if (!left_brake && right_brake) {   
                  control.f_l = control.b_l = 0;
                  control.f_r = control.b_r = period_bort;
                  if (switchactivity == 0) { 
                    if (adc_b0 < center_angle) {
                      float rpm_mechanical = ((float)MAX_RPM/(float)(right_angle-center_angle))*(float)((right_angle-center_angle)-angle_diff);
                      erpm = (int32_t)(-rpm_mechanical);
                    } 
                    if (adc_b0 <= right_angle)
                    {
                      erpm = 0;
                    }
                    if (adc_b0 > center_angle) {
                      erpm = -MAX_RPM;
                    }
                  }                                
              }          
            } else {
          control = (struct control_status){0};
          period_left = CalculatePeriod(control.f_l);
          period_right = CalculatePeriod(control.f_r);   
      }
     

      if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC1) == RESET) {
            if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2) == 0) && (period_left < period) && (coil.l == 0)) {                    
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, period);
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
              coil.l = 1;  

              // Получение текущих значений
              uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
              uint32_t timer_period = htim1.Instance->ARR;
              
              // Расчет нового значения с защитой от переполнения
              uint32_t new_compare = current_compare + tim1_ch1_pulse;
              if (new_compare > timer_period) {
                  new_compare -= timer_period;
                  
                  // Дополнительная корректировка если нужно
                  new_compare = new_compare % (timer_period + 1);
              }
              
              // Установка нового значения
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_compare);             
              HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);  
                      
            }       
            if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_2) > 0) && (coil.l == 0)) {
              if (period_left == period){ 
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period);                   
                __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);          
                coil.l = 2;
               
              // Получение текущих значений
              uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_1);
              uint32_t timer_period = htim1.Instance->ARR;
              
              // Расчет нового значения с защитой от переполнения
              uint32_t new_compare = current_compare + tim1_ch1_pulse;
              if (new_compare > timer_period) {
                  new_compare -= timer_period;
                  
                  // Дополнительная корректировка если нужно
                  new_compare = new_compare % (timer_period + 1);
              }
              
              // Установка нового значения
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, new_compare);
                HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
              }
              if (period_left < period){
                if (coil.l == 0){
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period_left);        
              }
              }        
            }
          }
      
      if (__HAL_TIM_GET_IT_SOURCE(&htim1, TIM_IT_CC2) == RESET) {
          if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4) == 0) && (period_right < period) && (coil.r == 0)) {                    
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, period);
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
            coil.r = 1; 
            
              // Получение текущих значений
              uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
              uint32_t timer_period = htim1.Instance->ARR;
              
              // Расчет нового значения с защитой от переполнения
              uint32_t new_compare = current_compare + tim1_ch2_pulse;
              if (new_compare > timer_period) {
                  new_compare -= timer_period;
                  
                  // Дополнительная корректировка если нужно
                  new_compare = new_compare % (timer_period + 1);
              }
              
              // Установка нового значения
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, new_compare);
              HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2); 
          }       
          if ((HAL_TIM_ReadCapturedValue(&htim4, TIM_CHANNEL_4) > 0) && (coil.r == 0)) {
            if (period_right == period){
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period);                    
              __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);        
              coil.r = 2;          
              // Получение текущих значений
              uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_2);
              uint32_t timer_period = htim1.Instance->ARR;
              
              // Расчет нового значения с защитой от переполнения
              uint32_t new_compare = current_compare + tim1_ch2_pulse;
              if (new_compare > timer_period) {
                  new_compare -= timer_period;
                  
                  // Дополнительная корректировка если нужно
                  new_compare = new_compare % (timer_period + 1);
              }
              
              // Установка нового значения
              __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, new_compare);  
              HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
            }
            if (period_right < period){
              if (coil.r == 0){
            __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period_right);        
            }
            }        
          }
        }
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */
  CAN_FilterTypeDef  sFilterConfig;
  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 18;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */
  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; 
  sFilterConfig.FilterIdHigh = 0x000;
  sFilterConfig.FilterIdLow = 0;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;

  if(HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
  Error_Handler();
  }
  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */
  __HAL_RCC_TIM1_CLK_ENABLE();
  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7200-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 20000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  sConfigOC.Pulse = tim1_ch1_pulse;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = tim1_ch2_pulse;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = tim1_ch3_pulse;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = tim1_ch4_pulse;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */
  HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 0);  // Высший приоритет
  HAL_NVIC_EnableIRQ(TIM1_CC_IRQn);
  __HAL_RCC_TIM1_CLK_ENABLE();
  // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC1);
  // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC2);
  // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC3);
  // __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_CC4);
  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 72-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 20000-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  { 
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_CC1);
    HAL_TIM_OC_Stop_IT(&htim4, TIM_CHANNEL_1);

    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DRV2_EN_B_Pin|EN_RELAY_1_Pin|EN_RELAY_2_Pin|EN_RELAY_3_Pin
                          |EN_RELAY_4_Pin|EN_RELAY_5_Pin|DRV2_EN_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DRV1_EN_B_Pin|DRV1_EN_A_Pin|CAN_STB_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : COMP_ADC_5_Pin COMP_ADC_6_Pin COMP_ADC_7_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_5_Pin|COMP_ADC_6_Pin|COMP_ADC_7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : COMP_ADC_8_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(COMP_ADC_8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : COMP_ADC_1_Pin COMP_ADC_2_Pin COMP_ADC_3_Pin */
  GPIO_InitStruct.Pin = COMP_ADC_1_Pin|COMP_ADC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);


  GPIO_InitStruct.Pin = COMP_ADC_3_Pin|COMP_ADC_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;  // Убрали лишнюю точку с запятой
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;           // Добавили подтяжку к VCC
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  /*Configure GPIO pins : COMP_ADC_1_Pin COMP_ADC_2_Pin COMP_ADC_3_Pin */
 
  

  /*Configure GPIO pins : DRV2_EN_B_Pin EN_RELAY_1_Pin EN_RELAY_2_Pin EN_RELAY_3_Pin
                           EN_RELAY_4_Pin EN_RELAY_5_Pin DRV2_EN_A_Pin */
  GPIO_InitStruct.Pin = DRV2_EN_B_Pin|EN_RELAY_1_Pin|EN_RELAY_2_Pin|EN_RELAY_3_Pin
                          |EN_RELAY_4_Pin|EN_RELAY_5_Pin|DRV2_EN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DRV1_EN_B_Pin DRV1_EN_A_Pin */
  GPIO_InitStruct.Pin = DRV1_EN_B_Pin|DRV1_EN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_STB_Pin */
  GPIO_InitStruct.Pin = CAN_STB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_STB_GPIO_Port, &GPIO_InitStruct);

  // /*Configure GPIO pin : COMP_ADC_4_Pin */
  // GPIO_InitStruct.Pin = COMP_ADC_4_Pin;
  // GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  // GPIO_InitStruct.Pull = GPIO_PULLUP;
  // HAL_GPIO_Init(COMP_ADC_4_GPIO_Port, &GPIO_InitStruct);
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);
/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TIM1_CC_IRQHandler(void) {
  HAL_TIM_IRQHandler(&htim1);
}

void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {  
    if (htim->Instance == TIM1){
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){     
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC1);
      if (coil.l == 1){                 
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, period_left);
      coil.l = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);      
      } 
      if (coil.l == 2){
      coil.l = 0;   
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
      }       
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC2);
      if (coil.r == 1){                 
      __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, period_right);
      coil.r = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);      
      } 
      if (coil.r == 2){
      coil.r = 0;   
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
      }            
    }
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){   
      //__HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC3);   
      CreateCANMessages();
     // Получение текущих значений
      uint32_t current_compare = __HAL_TIM_GET_COMPARE(&htim1, TIM_CHANNEL_3);
      uint32_t timer_period = htim1.Instance->ARR;
      
      // Расчет нового значения с защитой от переполнения
      uint32_t new_compare = current_compare + tim1_ch3_pulse;
      if (new_compare > timer_period) {
          new_compare -= timer_period;
          
          // Дополнительная корректировка если нужно
          new_compare = new_compare % (timer_period + 1);
      }
      
      // Установка нового значения
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, new_compare);
      }  
     
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
      __HAL_TIM_CLEAR_IT(&htim1, TIM_IT_CC4);
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
      switchactivity = 0;
      erpm = 0;
      // measurementActive = 0; // Останавливаем подсчет
      // firstPressDetected = 0; // Сбрасываем флаг первого нажатия             
      // if (Read_GPIO_Pin(comp[2]) == 0 && Read_GPIO_Pin(comp[3]) == 0) {              
      //   Selector = 'N';
      // }      
      // else if (Read_GPIO_Pin(comp[2]) == 1 && Read_GPIO_Pin(comp[3]) == 0) {
      //   switch(buttonPressCount) {         
      //     case 1:  Selector = 'D'; break;
      //     case 2:  Selector = '2'; break;
      //     case 3: Selector = '1'; break; // Все случаи ≥ 3
      //     default: Selector = 'P'; break; // Все случаи ≥ 3
      //   }        
      // }    
    }
    }  
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
