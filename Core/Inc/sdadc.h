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

/* USER CODE BEGIN Private defines */
#define SDADC_DMA_BUFFSIZE 6    // must be integer multiple of number of channels?
/* USER CODE END Private defines */

void MX_SDADC1_Init(void);

/* USER CODE BEGIN Prototypes */
void start_SDADCs (void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SDADC_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
