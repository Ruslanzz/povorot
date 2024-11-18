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
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);

uint16_t ADC_Result=0;
const int period=500;
const int period_drive=500;
const int period_brake=500;
const int period_bort=1000;
const int center_angle=2050;
const int left_angle=400;
const int right_angle=3710;
int angle_diff;
int period_calc;
int period_read;

struct coil_status
{
   int r;
   int l;  
};

struct control_status
{
   int r;
   int l;  
};

struct pwm_value
{
   int r;
   int l;   
};

struct down_timer
{
   int r;
   int l;  
};

struct coil_status coil;
struct control_status control =  {0,0};
struct down_timer dt = {0,0};
struct pwm_value pwm = {0,0};
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc1);

  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
 
  /* USER CODE END 2 */
  
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  MX_TIM1_Init();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  
  while (1)
  {
     /* USER CODE END WHILE */
    HAL_ADC_Start(&hadc1); // запускаем преобразование сигнала АЦП
    HAL_ADC_PollForConversion(&hadc1, 100);
    ADC_Result = HAL_ADC_GetValue(&hadc1);
    period_calc = ADC_Result/10;
    // /* USER CODE END WHILE */
    // if ((HAL_GPIO_ReadPin (GPIOB, LEFT_BRAKE_Pin) == GPIO_PIN_SET) && (HAL_GPIO_ReadPin (GPIOB, RIGHT_BRAKE_Pin) == GPIO_PIN_SET))
    // {
    //     control.r = period_brake;
    //     control.l = period_brake;
    // } 
    // else 
    // {  
    //       if ((HAL_GPIO_ReadPin (GPIOB, LEFT_BRAKE_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin (GPIOB, RIGHT_BRAKE_Pin) == GPIO_PIN_RESET))
    //       {
    //           angle_diff = ADC_Result-center_angle;
    //           if(angle_diff > 0){
    //             period_calc = (period_drive/(right_angle-center_angle))*((right_angle-center_angle)-angle_diff);
    //             control.f_l = period_drive;
    //             control.b_l = period_drive;
    //             control.f_r = period_calc;
    //             control.b_r = period_calc;
    //           }
    //           if(angle_diff < 0) {
    //             period_calc = (period_drive/(left_angle-center_angle))*((left_angle-center_angle)-angle_diff);
    //             control.f_l = period_calc;
    //             control.b_l = period_calc;
    //             control.f_r = period_drive;
    //             control.b_r = period_drive;
    //           }
    //           // else {
    //           //   control.f_l = period_drive;
    //           //   control.b_l = period_drive;
    //           //   control.f_r = period_drive;
    //           //   control.b_r = period_drive;
    //           // }

             
    //       }
    //       else 
    //       {
    //           if ((HAL_GPIO_ReadPin (GPIOB, RIGHT_BRAKE_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin (GPIOB, LEFT_BRAKE_Pin) == GPIO_PIN_SET))// R
    //           {   
    //               control.f_l = period_bort;
    //               control.f_r = 0;
    //               control.b_l = period_bort;
    //               control.b_r = 0;  
    //           }
    //           // else
    //           // {
    //           //     control.f_r = 0;
    //           //     control.b_r = 0;
    //           // }

    //           if ((HAL_GPIO_ReadPin (GPIOB, LEFT_BRAKE_Pin) == GPIO_PIN_RESET) && (HAL_GPIO_ReadPin (GPIOB, RIGHT_BRAKE_Pin) == GPIO_PIN_SET))// L
    //           {   
    //               control.f_l = 0;
    //               control.f_r = period_bort;
    //               control.b_l = 0;
    //               control.b_r = period_bort;         
    //           }
    //           // else
    //           // {
    //           //     control.f_l = 0;
    //           //     control.b_l = 0;
    //           // }
    //       }
    // }

  if (HAL_GPIO_ReadPin (GPIOB, LEFT_BRAKE_Pin) == GPIO_PIN_RESET){
    control.l = 0;
  }
  else {
    control.l = 500;
  }
  

  if (TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_1) == HAL_TIM_CHANNEL_STATE_READY) {
        if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3) == 0) && (control.l > 0) && (coil.l == 0)) {                    
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
          coil.l = 1;          
          __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); 
          HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);          
        }       
        if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3) > 0) && (coil.l == 0)) {
          if (control.l == 0){                  
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
          coil.l = 2;          
          __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  
          HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
          }
          else {
            if((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4) != control.l) && (coil.l == 0)){
          __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, period_calc);        
            }
          }        
        }
      }

  // if (TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_2) == HAL_TIM_CHANNEL_STATE_READY) {
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) == 0) && (control.f_r > 0) && (coil.f_r == 0)) {                    
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, period);
  //         coil.f_r = 1;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); 
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);          
  //       }       
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) > 0) && (coil.f_r == 0)) {
  //         if (control.f_r == 0){                  
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
  //         coil.f_r = 2;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
  //         }
  //         if (control.f_r > 0) {
  //           if((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_2) != control.f_r) && (coil.f_r == 0)){
  //             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, control.f_r);        
  //           }
  //         }        
  //       }
  //     }

  // if (TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_3) == HAL_TIM_CHANNEL_STATE_READY) {
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3) == 0) && (control.b_r > 0) && (coil.b_r == 0)) {                    
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, period);
  //         coil.b_r = 1;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); 
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);          
  //       }       
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3) > 0) && (coil.b_r == 0)) {
  //         if (control.b_r == 0){                  
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
  //         coil.b_r = 2;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);
  //         }
  //         if (control.b_r > 0) {
  //           if((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_3) != control.b_r) && (coil.b_r == 0)){
  //             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, control.b_r);        
  //           }
  //         }        
  //       }
  //     }
  
  // if (TIM_CHANNEL_STATE_GET(&htim1, TIM_CHANNEL_4) == HAL_TIM_CHANNEL_STATE_READY) {
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4) == 0) && (control.b_l > 0) && (coil.b_l == 0)) {                    
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, period);
  //         coil.b_l = 1;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE); 
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);          
  //       }       
  //       if ((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4) > 0) && (coil.b_l == 0)) {
  //         if (control.b_l == 0){                  
  //         __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 0);
  //         coil.b_l = 2;          
  //         __HAL_TIM_ENABLE_IT(&htim1, TIM_IT_UPDATE);  
  //         HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
  //         }
  //         if (control.b_l > 0) {
  //           if((HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_4) != control.b_l) && (coil.b_l == 0)){
  //             __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, control.b_l);        
  //           }
  //         }        
  //       }
  //     }
 //     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 2000);

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 100-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 50000-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 10;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 10;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = period-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : LEFT_BRAKE_Pin RIGHT_BRAKE_Pin ENABLE_FRONT_Pin ENABLE_BACK_Pin */
  GPIO_InitStruct.Pin = LEFT_BRAKE_Pin|RIGHT_BRAKE_Pin|ENABLE_FRONT_Pin|ENABLE_BACK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {  
    if (htim->Instance == TIM1){
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
      if (coil.l == 1){                 
      __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, period_calc);
      coil.l = 0;
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);      
      } 
      if (coil.l == 2){
      coil.l = 0;   
      HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
      }       
    } 
    // if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
    //   if (coil.f_r == 1){                 
    //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, control.f_r);
    //   coil.f_r = 0;
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);      
    //   } 
    //   if (coil.f_r == 2){
    //   coil.f_r = 0;   
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
    //   }      
    // } 
    // if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3){
    //   if (coil.b_r == 1){                 
    //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, control.b_r);
    //   coil.b_r = 0;
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);      
    //   } 
    //   if (coil.b_r == 2){
    //   coil.b_r = 0;   
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
    //   }    
    // } 
    // if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4){
    //   if (coil.b_l == 1){                 
    //   __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, control.b_l);
    //   coil.b_l = 0;
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);      
    //   } 
    //   if (coil.b_l == 2){
    //   coil.b_l = 0;   
    //   HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
    //   }  
    // } 
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
