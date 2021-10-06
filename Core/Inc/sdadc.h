/**
  ******************************************************************************
  * @file    sdadc.h
  * @brief   This file contains all the function prototypes for
  *          the sdadc.c file
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDADC_H__
#define __SDADC_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SDADC_HandleTypeDef hsdadc1;
extern SDADC_HandleTypeDef hsdadc2;

/* USER CODE BEGIN Private defines */
#define NUMSAMPLES 50 // Number of times to sample per cycle -- make sure this is an even number
#define NUMCHANNELS 3 // The number of channels to measure
#define SDADC1_DMA_BUFFSIZE (NUMSAMPLES * NUMCHANNELS) // Set this so the DMA buffer is full after one cycle
#define SDADC2_DMA_BUFFSIZE SDADC1_DMA_BUFFSIZE // Set this to the same value as for SDADC1

/* USER CODE END Private defines */

void MX_SDADC1_Init(void);
void MX_SDADC2_Init(void);

/* USER CODE BEGIN Prototypes */
extern volatile int16_t sdadc1_dma_buff[SDADC1_DMA_BUFFSIZE];
extern volatile int16_t sdadc2_dma_buff[SDADC2_DMA_BUFFSIZE];

void start_SDADCs (void);
void pllcalcs( uint16_t offset);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SDADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
