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
#include "fatfs.h"
#include "sdadc.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdbool.h"
#include "string.h"
#include <stdio.h>
#include <stdarg.h> //for va_list var arg functions
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define ADCTIMERFREQUENCY 72000000 // Clock speed of the STM32F373 in Hz
#define SUPPLYFREQUENCY 50 // Frequency of the supply in Hz
#define PLLLOCKCOUNT 20 // Number of samples for PLL to be considered locked ~ 4 seconds. N.B--Less than 255
#define PIDKP 3 // PID KP
#define PIDKI 1 // PID KI
#define PIDKD 0 // PID KD
#define ADCOFFSET 32768 // Half of the 16 bits range that the sdadc has
#define ADCVOLTS 3.3
#define LOOPCYCLES 250 // Cycles to complete before sending data
#define VCAL 271 // Calculated value is 2 710 000 / 10 000 = 271 for resistor divider
#define ICAL 90.65 // Calculated value is 100A:0.1A for transformer / 11 Ohms for resistor = 91

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
#define PLLTIMERAVG (ADCTIMERFREQUENCY/(SAMPLESPERCYCLE*SUPPLYFREQUENCY*2)) // The Timer19 reload value, the two is because the output compare toggles
#define PLLTIMERMIN (PLLTIMERAVG*0.8) // Minimum Timer 1 value to start next set of measurements
#define PLLTIMERMAX (PLLTIMERAVG*1.2) // Minimum Timer 1 value to start next set of measurements
#define PLLLOCKRANGE 0.5 // The PLL needs to be within 0.5 samples from the setpoint (+/- 1% timing accuracy @ 50 samples per cycle)

#define PIDK1 (PIDKP + PIDKI + PIDKD) // PID K1
#define PIDK2 (-1*PIDKP - 2*PIDKD) // PID K2
#define PIDK3 (PIDKD) // PID K3

#define VRATIO ((VCAL*ADCVOLTS)/(2^16))
#define IRATIO ((ICAL*ADCVOLTS)/(2^16))
#define LOOPSAMPLES (LOOPCYCLES*SAMPLESPERCYCLE) // Number of samples per transmit cycle.


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef struct channel_
{
    int64_t sum_P_import; // In W
    int64_t sum_P_export; // In W
    uint64_t sum_V_sq; // In V2
    uint64_t sum_I_sq; // In A2
    int32_t sum_V; // In V
    int32_t sum_I; // In A
    uint32_t sum_timer; // In counts
} channel_t;

typedef struct vipf_
{
    float Vrms; // In V
    float Irms; // In A
    float realpower_import; // In W
    float realpower_export; // In W
    float apparentpower; // In VA
    float power_factor; // Factor
    float frequency; // In Hz
    float time; // In s
    float units_import; // In Wh
    float units_export; // In Wh
    float import_counter; // In Wh
    float export_counter; // In Wh

} vipf_t;

channel_t channels[NUMCHANNELS];
channel_t channels_cycle[NUMCHANNELS];
channel_t channels_loop[NUMCHANNELS];
vipf_t channels_vipf[NUMCHANNELS];

uint8_t pllunlocked = PLLLOCKCOUNT;
uint16_t timercount = PLLTIMERAVG;
uint16_t prevtimercount = PLLTIMERAVG;
bool newcycle = false;
uint16_t cyclecount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void myprintf(const char *fmt, ...) {
    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart1, (uint8_t*)buffer, len, -1);

}

void pllcalcs(uint16_t offset){

    // PID Controller for the phase locked loop, only update on completion of cycle
    // This is first so that the timer can be updated as soon as possible
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
        prevtimercount = timercount;
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
            channels[n].sum_timer += prevtimercount;
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
    for (int n = 0; n < NUMCHANNELS; n++) {
        channels_loop[n].sum_V += channels_cycle[n].sum_V;
        channels_loop[n].sum_V_sq += channels_cycle[n].sum_V_sq;
        channels_loop[n].sum_I += channels_cycle[n].sum_I;
        channels_loop[n].sum_I_sq += channels_cycle[n].sum_I_sq;
        if (channels_cycle[n].sum_P_import > 0) {
            channels_loop[n].sum_P_import += channels_cycle[n].sum_P_import;
        } else {
            channels_loop[n].sum_P_export -= channels_cycle[n].sum_P_import;
        }
        channels_loop[n].sum_timer += channels_cycle[n].sum_timer;
    }
    // Reset the cycle accumulators
    memset(&channels_cycle, 0, sizeof(channels_cycle));
    cyclecount++;
}

void calculateVIPF(){
    float old_cyclecount = cyclecount;
    cyclecount = 0;
    for (int n = 0; n < NUMCHANNELS; n++) {
        channels_vipf[n].Vrms = VRATIO * sqrtf(((float)channels_loop[n].sum_V_sq) / LOOPSAMPLES);
        channels_vipf[n].Irms = IRATIO * sqrtf(((float)channels_loop[n].sum_I_sq) / LOOPSAMPLES);
        channels_vipf[n].realpower_import = VRATIO * IRATIO * (float)channels_loop[n].sum_P_import / LOOPSAMPLES;
        channels_vipf[n].realpower_export = VRATIO * IRATIO * (float)channels_loop[n].sum_P_export / LOOPSAMPLES;
        channels_vipf[n].apparentpower = channels_vipf[n].Vrms * channels_vipf[n].Irms;
        channels_vipf[n].power_factor = (channels_vipf[n].realpower_import + channels_vipf[n].realpower_export) / channels_vipf[n].apparentpower;
        // The doubling is because the adc only converts once every two times that the timer cycles
        // The addition of LOOPSAMPLES is because the timer is zero referenced
        channels_vipf[n].time = (((float)channels_loop[n].sum_timer + LOOPSAMPLES) * 2) / ADCTIMERFREQUENCY;
        channels_vipf[n].frequency = old_cyclecount / channels_vipf[n].time;
        channels_vipf[n].units_import = (channels_vipf[n].realpower_import * channels_vipf[n].time) / 3.6;
        channels_vipf[n].units_export = (channels_vipf[n].realpower_export * channels_vipf[n].time) / 3.6;
        channels_vipf[n].import_counter += channels_vipf[n].units_import;
        channels_vipf[n].export_counter += channels_vipf[n].units_export;
    }
    // Reset the loop accumulators
    memset(&channels_loop, 0, sizeof(channels_cycle));
}

void storeresults(){
    // some variables for FatFs
    FATFS FatFs; 	//Fatfs handle
    FIL fil; 		//File handle
    volatile FRESULT fres; //Result after operations

    // Open the file system
    f_mount(&FatFs, "", 1); //1=mount now

    for (int n = 0; n < NUMCHANNELS; n++){
        char filename[9];
        sprintf(filename, "Channel%i", n);

        //Now let's try and write a file
        fres = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);
        if(fres == FR_OK) {
            myprintf("I was able to open 'write.txt' for writing\r\n");
        } else {
            myprintf("f_open error (%i)\r\n", fres);
        }
        // Go to the end of the file so we can append
        f_lseek(&fil, f_size(&fil));

        char writebuff[500];
        sprintf(writebuff, "%.2f,",channels_vipf[n].Vrms);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].Irms);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].realpower_import);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].realpower_export);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].apparentpower);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].power_factor);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].time);
        sprintf(writebuff + strlen(writebuff), "%.2f,",channels_vipf[n].frequency);
        sprintf(writebuff + strlen(writebuff), "%.6f,",channels_vipf[n].units_import);
        sprintf(writebuff + strlen(writebuff), "%.6f,",channels_vipf[n].units_export);
        sprintf(writebuff + strlen(writebuff), "%.6f,",channels_vipf[n].import_counter);
        sprintf(writebuff + strlen(writebuff), "%.6f\r\n",channels_vipf[n].export_counter);

        UINT bytesWrote;
        fres = f_write(&fil, writebuff, strlen(writebuff), &bytesWrote);
        if(fres == FR_OK) {
            myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
        } else {
            myprintf("f_write error (%i)\r\n");
        }

        // Close the file when done
        f_close(&fil);
    }

    // We're done, so de-mount the drive
    f_mount(NULL, "", 0);

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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_SPI3_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
    start_SDADCs();
    HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
      // Calculate averages and transmit data to base station, ensure this can run constantly or the process will fail.
      if (newcycle) {
          addcycle();
      }
      if (cyclecount >= LOOPCYCLES){
          calculateVIPF();
          storeresults();
          HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_SDADC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
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
