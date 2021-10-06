/**
  ******************************************************************************
  * @file    sdadc.c
  * @brief   This file provides code for the configuration
  *          of the SDADC instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "sdadc.h"

/* USER CODE BEGIN 0 */
#include "tim.h"
volatile int16_t sdadc1_dma_buff[SDADC1_DMA_BUFFSIZE];
volatile int16_t sdadc2_dma_buff[SDADC2_DMA_BUFFSIZE];
uint32_t counter=0;
/* USER CODE END 0 */

SDADC_HandleTypeDef hsdadc1;
SDADC_HandleTypeDef hsdadc2;
DMA_HandleTypeDef hdma_sdadc1;
DMA_HandleTypeDef hdma_sdadc2;

/* SDADC1 init function */
void MX_SDADC1_Init(void)
{

  /* USER CODE BEGIN SDADC1_Init 0 */

  /* USER CODE END SDADC1_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC1_Init 1 */

  /* USER CODE END SDADC1_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc1.Instance = SDADC1;
  hsdadc1.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc1.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc1.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc1.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc1.InjectedTrigger = SDADC_EXTERNAL_TRIGGER;
  hsdadc1.ExtTriggerEdge = SDADC_EXT_TRIG_RISING_EDGE;
  if (HAL_SDADC_Init(&hsdadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc1, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedExtTrigger(&hsdadc1, SDADC_EXT_TRIG_TIM19_CC2, SDADC_EXT_TRIG_RISING_EDGE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc1, SDADC_EXTERNAL_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc1, SDADC_CHANNEL_0|SDADC_CHANNEL_1
                              |SDADC_CHANNEL_2, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc1, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_0, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_1, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc1, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC1_Init 2 */

  /* USER CODE END SDADC1_Init 2 */

}
/* SDADC2 init function */
void MX_SDADC2_Init(void)
{

  /* USER CODE BEGIN SDADC2_Init 0 */

  /* USER CODE END SDADC2_Init 0 */

  SDADC_ConfParamTypeDef ConfParamStruct = {0};

  /* USER CODE BEGIN SDADC2_Init 1 */

  /* USER CODE END SDADC2_Init 1 */
  /** Configure the SDADC low power mode, fast conversion mode,
  slow clock mode and SDADC1 reference voltage
  */
  hsdadc2.Instance = SDADC2;
  hsdadc2.Init.IdleLowPowerMode = SDADC_LOWPOWER_NONE;
  hsdadc2.Init.FastConversionMode = SDADC_FAST_CONV_DISABLE;
  hsdadc2.Init.SlowClockMode = SDADC_SLOW_CLOCK_DISABLE;
  hsdadc2.Init.ReferenceVoltage = SDADC_VREF_EXT;
  hsdadc2.InjectedTrigger = SDADC_EXTERNAL_TRIGGER;
  hsdadc2.ExtTriggerEdge = SDADC_EXT_TRIG_RISING_EDGE;
  if (HAL_SDADC_Init(&hsdadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Mode
  */
  if (HAL_SDADC_SelectInjectedDelay(&hsdadc2, SDADC_INJECTED_DELAY_NONE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedExtTrigger(&hsdadc2, SDADC_EXT_TRIG_TIM19_CC3, SDADC_EXT_TRIG_RISING_EDGE) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_SelectInjectedTrigger(&hsdadc2, SDADC_EXTERNAL_TRIGGER) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_SDADC_InjectedConfigChannel(&hsdadc2, SDADC_CHANNEL_0|SDADC_CHANNEL_1
                              |SDADC_CHANNEL_2, SDADC_CONTINUOUS_CONV_OFF) != HAL_OK)
  {
    Error_Handler();
  }
  /** Set parameters for SDADC configuration 0 Register
  */
  ConfParamStruct.InputMode = SDADC_INPUT_MODE_SE_ZERO_REFERENCE;
  ConfParamStruct.Gain = SDADC_GAIN_1;
  ConfParamStruct.CommonMode = SDADC_COMMON_MODE_VSSA;
  ConfParamStruct.Offset = 0;
  if (HAL_SDADC_PrepareChannelConfig(&hsdadc2, SDADC_CONF_INDEX_0, &ConfParamStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_0, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_1, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the Injected Channel
  */
  if (HAL_SDADC_AssociateChannelConfig(&hsdadc2, SDADC_CHANNEL_2, SDADC_CONF_INDEX_0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SDADC2_Init 2 */

  /* USER CODE END SDADC2_Init 2 */

}

void HAL_SDADC_MspInit(SDADC_HandleTypeDef* sdadcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(sdadcHandle->Instance==SDADC1)
  {
  /* USER CODE BEGIN SDADC1_MspInit 0 */

  /* USER CODE END SDADC1_MspInit 0 */
    /* SDADC1 clock enable */
    __HAL_RCC_SDADC1_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC1 GPIO Configuration
    PE10     ------> SDADC1_AIN2P
    PE11     ------> SDADC1_AIN1P
    PE12     ------> SDADC1_AIN0P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SDADC1 DMA Init */
    /* SDADC1 Init */
    hdma_sdadc1.Instance = DMA2_Channel3;
    hdma_sdadc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sdadc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdadc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdadc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sdadc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sdadc1.Init.Mode = DMA_CIRCULAR;
    hdma_sdadc1.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_sdadc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(sdadcHandle,hdma,hdma_sdadc1);

  /* USER CODE BEGIN SDADC1_MspInit 1 */

  /* USER CODE END SDADC1_MspInit 1 */
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
  /* USER CODE BEGIN SDADC2_MspInit 0 */

  /* USER CODE END SDADC2_MspInit 0 */
    /* SDADC2 clock enable */
    __HAL_RCC_SDADC2_CLK_ENABLE();

    __HAL_RCC_GPIOE_CLK_ENABLE();
    /**SDADC2 GPIO Configuration
    PE13     ------> SDADC2_AIN2P
    PE14     ------> SDADC2_AIN1P
    PE15     ------> SDADC2_AIN0P
    */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /* SDADC2 DMA Init */
    /* SDADC2 Init */
    hdma_sdadc2.Instance = DMA2_Channel4;
    hdma_sdadc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_sdadc2.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_sdadc2.Init.MemInc = DMA_MINC_ENABLE;
    hdma_sdadc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_sdadc2.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_sdadc2.Init.Mode = DMA_CIRCULAR;
    hdma_sdadc2.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_sdadc2) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(sdadcHandle,hdma,hdma_sdadc2);

  /* USER CODE BEGIN SDADC2_MspInit 1 */

  /* USER CODE END SDADC2_MspInit 1 */
  }
}

void HAL_SDADC_MspDeInit(SDADC_HandleTypeDef* sdadcHandle)
{

  if(sdadcHandle->Instance==SDADC1)
  {
  /* USER CODE BEGIN SDADC1_MspDeInit 0 */

  /* USER CODE END SDADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDADC1_CLK_DISABLE();

    /**SDADC1 GPIO Configuration
    PE10     ------> SDADC1_AIN2P
    PE11     ------> SDADC1_AIN1P
    PE12     ------> SDADC1_AIN0P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);

    /* SDADC1 DMA DeInit */
    HAL_DMA_DeInit(sdadcHandle->hdma);
  /* USER CODE BEGIN SDADC1_MspDeInit 1 */

  /* USER CODE END SDADC1_MspDeInit 1 */
  }
  else if(sdadcHandle->Instance==SDADC2)
  {
  /* USER CODE BEGIN SDADC2_MspDeInit 0 */

  /* USER CODE END SDADC2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_SDADC2_CLK_DISABLE();

    /**SDADC2 GPIO Configuration
    PE13     ------> SDADC2_AIN2P
    PE14     ------> SDADC2_AIN1P
    PE15     ------> SDADC2_AIN0P
    */
    HAL_GPIO_DeInit(GPIOE, GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    /* SDADC2 DMA DeInit */
    HAL_DMA_DeInit(sdadcHandle->hdma);
  /* USER CODE BEGIN SDADC2_MspDeInit 1 */

  /* USER CODE END SDADC2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_SDADC_InjectedConvHalfCpltCallback(SDADC_HandleTypeDef* hadc)
{
//    if(hadc == &hsdadc1){
//        counter +=1;
//        if(counter>10){
//            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
//            counter = 1;
//        }
//
//    }
    //if(hadc == &hadc1) adc_conv_halfcplt_flag = true;
}

void HAL_SDADC_InjectedConvCpltCallback(SDADC_HandleTypeDef* hadc)
{
//    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    if(hadc == &hsdadc1) {
//        HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    }
    if(hadc == &hsdadc2) {
        counter +=1;
        if(counter>=50){
            HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
            counter = 0;
        }
    }
}

void start_SDADCs (void) {

    HAL_SDADC_InjectedStop_DMA(&hsdadc1);
    HAL_SDADC_InjectedStop_DMA(&hsdadc2);
    HAL_Delay(100);
    HAL_TIM_OC_Start(&htim19, TIM_CHANNEL_2);
    HAL_TIM_OC_Start(&htim19, TIM_CHANNEL_3);
    HAL_Delay(100);

    HAL_SDADC_InjectedStart_DMA(&hsdadc1, (uint32_t*)sdadc1_dma_buff, SDADC1_DMA_BUFFSIZE);
    HAL_SDADC_InjectedStart_DMA(&hsdadc2, (uint32_t*)sdadc2_dma_buff, SDADC2_DMA_BUFFSIZE);

}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
