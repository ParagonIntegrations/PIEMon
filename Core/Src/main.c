/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "sdadc.h"
#include "tim.h"
#include "gpio.h"
#include "math.h"
#include "stdbool.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define CLOCKFREQUENCY 72000000 // Clock speed of the STM32F373 in Hz
#define SUPPLYFREQUENCY 50 // Frequency of the supply in Hz
#define PLLLOCKCOUNT 20 // Number of samples for PLL to be considered locked ~ 4 seconds. N.B--Less than 255
#define PIDKP 3 // PID KP
#define PIDKI 1 // PID KI
#define PIDKD 0 // PID KD
#define ADCOFFSET 32768 // Half of the 16 bits range that the sdadc has
#define LOOPCYCLES 250 // Cycles to complete before sending data

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//#define max(a,b) \
//   ({ __typeof__ (a) _a = (a); \
//       __typeof__ (b) _b = (b); \
//     _a > _b ? _a : _b; })
//#define min(a,b) \
//   ({ __typeof__ (a) _a = (a); \
//       __typeof__ (b) _b = (b); \
//     _a < _b ? _a : _b; })


// Calculated constants (at compile time)
#define PLLTIMERAVG (CLOCKFREQUENCY/(SAMPLESPERCYCLE*SUPPLYFREQUENCY*2)) // The Timer19 reload value, the two is because the output compare toggles
#define PLLTIMERMIN (PLLTIMERAVG*0.8) // Minimum Timer 1 value to start next set of measurements
#define PLLTIMERMAX (PLLTIMERAVG*1.2) // Minimum Timer 1 value to start next set of measurements
#define PLLLOCKRANGE 0.5 // The PLL needs to be within 0.5 samples from the setpoint (+/- 1% timing accuracy @ 50 samples per cycle)

#define PIDK1 (PIDKP + PIDKI + PIDKD) // PID K1
#define PIDK2 (-1*PIDKP - 2*PIDKD) // PID K2
#define PIDK3 (PIDKD) // PID K3


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef struct channel_
{
    int64_t sum_P_import;
    int64_t sum_P_export;
    uint64_t sum_V_sq;
    uint64_t sum_I_sq;
    int32_t sum_V;
    int32_t sum_I;
    uint32_t count;
} channel_t;

channel_t channels[NUMCHANNELS];
channel_t channels_cycle[NUMCHANNELS];
channel_t channels_set[NUMCHANNELS];

uint8_t pllunlocked = PLLLOCKCOUNT;
uint16_t timercount = PLLTIMERAVG;
bool newcycle = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void pllcalcs(uint16_t offset){

    // PID Controller for the phase locked loop, only update on completion of cycle
    static float e0, e1, e2;
    if (offset == SDADC1_DMA_BUFFSIZE/2){
        e2 = e1;
        e1 = e0;
        int32_t last_reading = sdadc1_dma_buff[SDADC1_DMA_BUFFSIZE-1];
        int32_t second_last_reading = sdadc1_dma_buff[SDADC1_DMA_BUFFSIZE-2];
        // Do interpolation or extrapolation to determine how much the timer needs to adjust
        e0 = (last_reading - ADCOFFSET) / ((float)second_last_reading - last_reading);
        e0 = (last_reading > second_last_reading) ? e0 : e0 + (SAMPLESPERCYCLE / 2);
        e0 = constrain(e0, -SAMPLESPERCYCLE, SAMPLESPERCYCLE);

        // Calculate the new timer value
        timercount += (e0 * PIDK1 + e1 * PIDK2 + e2 * PIDK3);
        timercount = constrain(timercount, PLLTIMERMIN, PLLTIMERMAX);

        // Check if PLL is in lock range and decrement the counter if it is, otherwise set counter to max
        if (e0 > PLLLOCKRANGE){
            pllunlocked = PLLLOCKCOUNT;
        } else if (pllunlocked) {
            pllunlocked--;
        }
        // Update the timer
        __HAL_TIM_SET_AUTORELOAD(&htim19, timercount);
    }

    // Loop through the buffer and save the data
    // Assign variables needed for the calculations
    int32_t sample_V, sample_I, signed_V, signed_I;
    // Cycle through the channels
    for (int n = 0; n < NUMCHANNELS; n++){
        // Cycle through all the data in the buffer for the current channel
        for (int i = 0; i < SDADC1_DMA_BUFFSIZE/2 ; i += NUMCHANNELS ){

            // ----------------------------------------
            // Voltage
            sample_V = sdadc1_dma_buff[offset + i + n];
            signed_V = sample_V - ADCOFFSET;
            channels[n].sum_V += signed_V;
            channels[n].sum_V_sq += signed_V * signed_V;
            // ----------------------------------------
            // Current
            sample_I = sdadc2_dma_buff[offset + i + n];
            signed_I = sample_I - ADCOFFSET;
            channels[n].sum_I += signed_I;
            channels[n].sum_I_sq += signed_I * signed_I;
            // ----------------------------------------
            // Power
            channels[n].sum_P_import += signed_V * signed_I;
            // ----------------------------------------
            // Sample Count
            channels[n].count++;
        }
    }

    // If this is the second half of the buffer the data needs to be copied to the cycle stuct
    if (offset == SDADC1_DMA_BUFFSIZE/2){
        // Copy accumulators for use in main loop
        memcpy(&channels_cycle, &channels, sizeof(channels));
        // Reset the accumulators
        memset(&channels, 0, sizeof(channels));
        newcycle = true;
    }


}

void addcycle () {

    newcycle = false;
    for (int n = 0; n < NUMCHANNELS; n++){
        channels_set[n].sum_V += channels_cycle[n].sum_V;
        channels_set[n].sum_V_sq += channels_cycle[n].sum_V_sq;
        channels_set[n].sum_I += channels_cycle[n].sum_I;
        channels_set[n].sum_I_sq += channels_cycle[n].sum_I_sq;
        if (channels_cycle[n].sum_P_import > 0){
            channels_set[n].sum_P_import += channels_cycle[n].sum_P_import;
        } else {
            channels_set[n].sum_P_export -= channels_cycle[n].sum_P_import;
        }
        channels_set[n].count += channels_cycle[n].count;
    }
    // Reset the cycle accumulators
    memset(&channels_cycle, 0, sizeof(channels_cycle));

/*
    TotalV1Squared += CycleV1Squared;
    TotalV2Squared += CycleV2Squared;
    TotalV3Squared += CycleV3Squared;
    TotalI1Squared += CycleI1Squared;
    TotalI2Squared += CycleI2Squared;
    TotalI3Squared += CycleI3Squared;
    if (CycleP1>=0){
        TotalP1Import += CycleP1;
    } else {
        TotalP1Export -= CycleP1;
    }
    if (CycleP2>=0){
        TotalP2Import += CycleP2;
    } else {
        TotalP2Export -= CycleP2;
    }
    if (CycleP3>=0){
        TotalP3Import += CycleP3;
    } else {
        TotalP3Export -= CycleP3;
    }
    SumTimerCount += (OCR1A+1);

    CycleCount++;
    NewCycle = false;
*/
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
  MX_SDADC1_Init();
  MX_TIM19_Init();
  MX_SDADC2_Init();
  /* USER CODE BEGIN 2 */
    start_SDADCs();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      for( int i = 1000; i < 1500; i += 500 ){
          HAL_Delay(i);
          HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
      }
    /* USER CODE END WHILE */

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_SDADC;
  PeriphClkInit.SdadcClockSelection = RCC_SDADCSYSCLK_DIV12;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG1);
  HAL_PWREx_EnableSDADC(PWR_SDADC_ANALOG2);
}

/* USER CODE BEGIN 4 */

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
