/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cmsis_os.h"
#include "usb_host.h"

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

CRC_HandleTypeDef hcrc;

DMA2D_HandleTypeDef hdma2d;

I2C_HandleTypeDef hi2c3;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

SDRAM_HandleTypeDef hsdram1;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CRC_Init(void);
static void MX_DMA2D_Init(void);
//static void MX_FMC_Init(void);
static void MX_I2C3_Init(void);
//static void MX_LTDC_Init(void);
//static void MX_SPI5_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_sdram.h"

extern ADC_HandleTypeDef  hadc1;
extern TIM_HandleTypeDef  htim2;
extern UART_HandleTypeDef huart1;

extern LTDC_HandleTypeDef hltdc;

/* Wait for the LTDC to enter the vertical sync window
   (briefly between frames - safe to draw during this time) */
static inline void wait_vsync(void)
{
    /* Wait until LTDC is NOT in vsync (we're inside the visible frame) */
    while (LTDC->CDSR & LTDC_CDSR_VSYNCS) { }
    /* Now wait until vsync starts (frame finished, blanking period begins) */
    while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS)) { }
}
/* =========================================================
   CONFIGURATION
   ========================================================= */
#define FS                  500       /* sample rate in Hz */
#define THRESHOLD           0.015f    /* fixed threshold for R-wave detection */
#define REFRACTORY_SAMPLES  150       /* 300 ms = ignore peaks for this long after one */
#define HR_SMOOTH_LEN       5         /* average HR over last N beats */

/* =========================================================
   FILTER COEFFICIENTS
   Designed for Fs = 500 Hz, 2nd-order Butterworth
   HPF: fc = 5 Hz   - removes baseline wander, P/T waves
   LPF: fc = 15 Hz  - removes mains hum, muscle noise
   ========================================================= */
static const float hpf_b[3] = { 0.95654368f, -1.91308735f,  0.95654368f };
static const float hpf_a[2] = {-1.91119707f,  0.91497764f };

static const float lpf_b[3] = { 0.00782021f,  0.01564042f,  0.00782021f };
static const float lpf_a[2] = {-1.73472577f,  0.76600660f };

/* =========================================================
   FILTER STATE
   Each filter holds 2 previous inputs and 2 previous outputs
   ========================================================= */
static float hpf_x1 = 0.0f, hpf_x2 = 0.0f;
static float hpf_y1 = 0.0f, hpf_y2 = 0.0f;

static float lpf_x1 = 0.0f, lpf_x2 = 0.0f;
static float lpf_y1 = 0.0f, lpf_y2 = 0.0f;

/* =========================================================
   PEAK DETECTION STATE
   ========================================================= */
static uint32_t sample_n      = 0;       /* total samples since boot */
static uint32_t last_peak_n   = 0;       /* sample index of previous R-wave */
static uint32_t refractory    = 0;       /* countdown after a detected peak */

/* =========================================================
   HR SMOOTHING (running average of last 5 beats)
   ========================================================= */
static float    hr_buf[HR_SMOOTH_LEN] = {0};
static int      hr_buf_idx            = 0;
static int      hr_buf_count          = 0;

/* =========================================================
   STAGE 1A: HIGH-PASS FILTER  (Direct Form I)
   y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]
   ========================================================= */
static float hpf_filter(float x)
{
    float y = hpf_b[0]*x + hpf_b[1]*hpf_x1 + hpf_b[2]*hpf_x2
                         - hpf_a[0]*hpf_y1 - hpf_a[1]*hpf_y2;
    hpf_x2 = hpf_x1;  hpf_x1 = x;
    hpf_y2 = hpf_y1;  hpf_y1 = y;
    return y;
}

/* =========================================================
   STAGE 1B: LOW-PASS FILTER  (Direct Form I)
   ========================================================= */
static float lpf_filter(float x)
{
    float y = lpf_b[0]*x + lpf_b[1]*lpf_x1 + lpf_b[2]*lpf_x2
                         - lpf_a[0]*lpf_y1 - lpf_a[1]*lpf_y2;
    lpf_x2 = lpf_x1;  lpf_x1 = x;
    lpf_y2 = lpf_y1;  lpf_y1 = y;
    return y;
}

/* =========================================================
   HR SMOOTHING - average of last N instantaneous HRs
   ========================================================= */
static float hr_smoothed(float hr_now)
{
    hr_buf[hr_buf_idx % HR_SMOOTH_LEN] = hr_now;
    hr_buf_idx++;
    if (hr_buf_count < HR_SMOOTH_LEN) hr_buf_count++;

    float sum = 0.0f;
    for (int i = 0; i < hr_buf_count; i++) sum += hr_buf[i];
    return sum / (float)hr_buf_count;
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
  MX_CRC_Init();
  MX_DMA2D_Init();
//  MX_FMC_Init();
  MX_I2C3_Init();
//  MX_LTDC_Init();
//  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 4096);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
  }
}
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T2_TRGO;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DMA2D Initialization Function
  * @param None
  * @retval None
  */
static void MX_DMA2D_Init(void)
{

  /* USER CODE BEGIN DMA2D_Init 0 */

  /* USER CODE END DMA2D_Init 0 */

  /* USER CODE BEGIN DMA2D_Init 1 */

  /* USER CODE END DMA2D_Init 1 */
  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_ARGB8888;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_ARGB8888;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DMA2D_Init 2 */

  /* USER CODE END DMA2D_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
//static void MX_LTDC_Init(void)
//{
//
//  /* USER CODE BEGIN LTDC_Init 0 */
//
//  /* USER CODE END LTDC_Init 0 */
//
//  LTDC_LayerCfgTypeDef pLayerCfg = {0};
//
//  /* USER CODE BEGIN LTDC_Init 1 */
//
//  /* USER CODE END LTDC_Init 1 */
//  hltdc.Instance = LTDC;
//  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
//  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
//  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
//  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
//  hltdc.Init.HorizontalSync = 9;
//  hltdc.Init.VerticalSync = 1;
//  hltdc.Init.AccumulatedHBP = 29;
//  hltdc.Init.AccumulatedVBP = 3;
//  hltdc.Init.AccumulatedActiveW = 269;
//  hltdc.Init.AccumulatedActiveH = 323;
//  hltdc.Init.TotalWidth = 279;
//  hltdc.Init.TotalHeigh = 327;
//  hltdc.Init.Backcolor.Blue = 0;
//  hltdc.Init.Backcolor.Green = 0;
//  hltdc.Init.Backcolor.Red = 0;
//  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  pLayerCfg.WindowX0 = 0;
//  pLayerCfg.WindowX1 = 240;
//  pLayerCfg.WindowY0 = 0;
//  pLayerCfg.WindowY1 = 320;
//  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
//  pLayerCfg.Alpha = 255;
//  pLayerCfg.Alpha0 = 0;
//  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA;
//  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA;
//  pLayerCfg.FBStartAdress = 0xD0000000;
//  pLayerCfg.ImageWidth = 240;
//  pLayerCfg.ImageHeight = 320;
//  pLayerCfg.Backcolor.Blue = 0;
//  pLayerCfg.Backcolor.Green = 0;
//  pLayerCfg.Backcolor.Red = 0;
//  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN LTDC_Init 2 */
//
//  /* USER CODE END LTDC_Init 2 */
//
//}

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
//static void MX_SPI5_Init(void)
//{
//
//  /* USER CODE BEGIN SPI5_Init 0 */
//
//  /* USER CODE END SPI5_Init 0 */
//
//  /* USER CODE BEGIN SPI5_Init 1 */
//
//  /* USER CODE END SPI5_Init 1 */
//  /* SPI5 parameter configuration*/
//  hspi5.Instance = SPI5;
//  hspi5.Init.Mode = SPI_MODE_MASTER;
//  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
//  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
//  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
//  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
//  hspi5.Init.NSS = SPI_NSS_SOFT;
//  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
//  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
//  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
//  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
//  hspi5.Init.CRCPolynomial = 10;
//  if (HAL_SPI_Init(&hspi5) != HAL_OK)
//  {
//    Error_Handler();
//  }
//  /* USER CODE BEGIN SPI5_Init 2 */
//
//  /* USER CODE END SPI5_Init 2 */
//
//}
//
///**
//  * @brief TIM1 Initialization Function
//  * @param None
//  * @retval None
//  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/* FMC initialization function */
//static void MX_FMC_Init(void)
//{
//
//  /* USER CODE BEGIN FMC_Init 0 */
//
//  /* USER CODE END FMC_Init 0 */
//
//  FMC_SDRAM_TimingTypeDef SdramTiming = {0};
//
//  /* USER CODE BEGIN FMC_Init 1 */
//
//  /* USER CODE END FMC_Init 1 */
//
//  /** Perform the SDRAM1 memory initialization sequence
//  */
//  hsdram1.Instance = FMC_SDRAM_DEVICE;
//  /* hsdram1.Init */
//  hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
//  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
//  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
//  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
//  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
//  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
//  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
//  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
//  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
//  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_1;
//  /* SdramTiming */
//  SdramTiming.LoadToActiveDelay = 2;
//  SdramTiming.ExitSelfRefreshDelay = 7;
//  SdramTiming.SelfRefreshTime = 4;
//  SdramTiming.RowCycleDelay = 7;
//  SdramTiming.WriteRecoveryTime = 3;
//  SdramTiming.RPDelay = 2;
//  SdramTiming.RCDDelay = 2;
//
//  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
//  {
//    Error_Handler( );
//  }
//
//  /* USER CODE BEGIN FMC_Init 2 */
//
//  /* USER CODE END FMC_Init 2 */
//}

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ACP_RST_GPIO_Port, ACP_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, RDX_Pin|WRX_DCX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, LD3_Pin|LD4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : NCS_MEMS_SPI_Pin CSX_Pin OTG_FS_PSO_Pin */
  GPIO_InitStruct.Pin = NCS_MEMS_SPI_Pin|CSX_Pin|OTG_FS_PSO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : B1_Pin MEMS_INT1_Pin MEMS_INT2_Pin TP_INT1_Pin */
  GPIO_InitStruct.Pin = B1_Pin|MEMS_INT1_Pin|MEMS_INT2_Pin|TP_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACP_RST_Pin */
  GPIO_InitStruct.Pin = ACP_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ACP_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OC_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TE_Pin */
  GPIO_InitStruct.Pin = TE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(TE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RDX_Pin WRX_DCX_Pin */
  GPIO_InitStruct.Pin = RDX_Pin|WRX_DCX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD4_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
	/* USER CODE BEGIN 5 */

	  /* Boot indicator */
	  for (int i = 0; i < 4; i++) {
	      HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_13);
	      osDelay(150);
	  }
	  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);

	  const char *banner = "ECG QRS detector with LCD\r\n";
	  HAL_UART_Transmit(&huart1, (uint8_t*)banner, strlen(banner), 100);

	  /* ===== Initialise SDRAM and LCD via BSP ===== */
	  BSP_SDRAM_Init();
	  BSP_LCD_Init();
	  BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
	  BSP_LCD_SelectLayer(LCD_BACKGROUND_LAYER);
	  BSP_LCD_DisplayOn();
	  BSP_LCD_Clear(LCD_COLOR_BLACK);

	  /* Static labels */
	  BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	  BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	  BSP_LCD_SetFont(&Font24);
	  BSP_LCD_DisplayStringAt(0, 5, (uint8_t*)"HEART RATE", CENTER_MODE);

	  /* ===== Waveform area config ===== */
	  #define WAVE_X       0
	  #define WAVE_Y       80                /* below the BPM text */
	  #define WAVE_W       240
	  #define WAVE_H       180               /* leaves space below for footer */
	  #define WAVE_MID     (WAVE_Y + WAVE_H/2)

	  /* Draw waveform border */
	  BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
	  BSP_LCD_DrawRect(WAVE_X, WAVE_Y, WAVE_W, WAVE_H);

	  /* Waveform state */
	  static uint16_t wave_x = WAVE_X + 1;     /* current draw column */
	  static int16_t  prev_y = WAVE_MID;       /* previous sample's y */

	  /* ===== Start ADC sampling ===== */
	  HAL_TIM_Base_Start(&htim2);
	  HAL_ADC_Start(&hadc1);

	  uint32_t bpm_last_drawn = 0;

	  #define WAVE_X       0
	  #define WAVE_Y       80
	  #define WAVE_W       240
	  #define WAVE_H       180
	  #define WAVE_MID     (WAVE_Y + WAVE_H/2)

	  /* Force the waveform area to be solid black */
	  BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	  BSP_LCD_FillRect(WAVE_X, WAVE_Y, WAVE_W, WAVE_H);

	  for (;;)
	  {
	      if (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK) {
	          HAL_ADC_Start(&hadc1);
	          continue;
	      }
	      uint32_t raw = HAL_ADC_GetValue(&hadc1);
	      HAL_ADC_Start(&hadc1);

	      sample_n++;

	      float x = (float)raw / 4095.0f;
	      float x_hp = hpf_filter(x);
	      float x_bp = lpf_filter(x_hp);

	      /* ===== Draw waveform sample (every 2nd sample to reduce LCD load) ===== */
	      static int   draw_skip = 0;
	      static float bp_max_in_window = -1.0f;
	      static float bp_min_in_window =  1.0f;

	           if (x_bp > bp_max_in_window) bp_max_in_window = x_bp;
	           if (x_bp < bp_min_in_window) bp_min_in_window = x_bp;

	           draw_skip++;
	                 if (draw_skip >= 8) {
	                     draw_skip = 0;

	                     wait_vsync();    //VSYNC :3

	               /* Draw the extreme that traveled furthest from zero (preserves peaks) */
	               float draw_val = (fabsf(bp_max_in_window) > fabsf(bp_min_in_window))
	                                 ? bp_max_in_window : bp_min_in_window;
	               bp_max_in_window = -1.0f;
	               bp_min_in_window =  1.0f;

	               int16_t y_now = WAVE_MID - (int16_t)(draw_val * (WAVE_H / 2) * 25.0f);
	               if (y_now < WAVE_Y + 2)        y_now = WAVE_Y + 2;
	               if (y_now > WAVE_Y + WAVE_H-2) y_now = WAVE_Y + WAVE_H - 2;

	               /* Erase column ahead, draw current column black-then-green */
	               BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	               BSP_LCD_DrawVLine(wave_x, WAVE_Y + 1, WAVE_H - 2);

	               uint16_t next_x = wave_x + 1;
	               if (next_x >= WAVE_X + WAVE_W - 1) next_x = WAVE_X + 1;
	               BSP_LCD_DrawVLine(next_x, WAVE_Y + 1, WAVE_H - 2);

	               /* Draw line from previous y to current y at this column */
	               BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
	               if (prev_y != y_now) {
	                   int16_t y_top = (y_now < prev_y) ? y_now : prev_y;
	                   int16_t y_bot = (y_now > prev_y) ? y_now : prev_y;
	                   BSP_LCD_DrawVLine(wave_x, y_top, y_bot - y_top + 1);
	               } else {
	                   BSP_LCD_DrawPixel(wave_x, y_now, LCD_COLOR_GREEN);
	               }
	               prev_y = y_now;

	               wave_x++;
	               if (wave_x >= WAVE_X + WAVE_W - 1) wave_x = WAVE_X + 1;
	           }
	      /* ===== QRS detection (unchanged) ===== */
	      if (refractory > 0) refractory--;

	      if (refractory == 0 && x_bp > THRESHOLD)
	      {
	          if (last_peak_n > 0)
	          {
	              uint32_t delta_samples = sample_n - last_peak_n;
	              float    delta_t       = (float)delta_samples / (float)FS;

	              if (delta_t >= 0.3f && delta_t <= 1.2f)
	              {
	                  float hr_inst = 60.0f / delta_t;
	                  float hr_avg  = hr_smoothed(hr_inst);

	                  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_SET);

	                  /* UART log */
	                  char msg[80];
	                  int len = snprintf(msg, sizeof(msg),
	                      "BEAT  dt=%4dms  HR=%3dbpm\r\n",
	                      (int)(delta_t * 1000.0f), (int)(hr_avg + 0.5f));
	                  HAL_UART_Transmit(&huart1, (uint8_t*)msg, len, 50);

	                  /* Update BPM on screen if changed */
	                  uint32_t bpm_int = (uint32_t)(hr_avg + 0.5f);
	                  if (bpm_int != bpm_last_drawn) {
	                      wait_vsync();
	                	  char bpm_str[16];
	                      snprintf(bpm_str, sizeof(bpm_str), "%3lu BPM", bpm_int);
	                      BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	                      BSP_LCD_FillRect(50, 35, 140, 30);
	                      BSP_LCD_SetTextColor(LCD_COLOR_RED);
	                      BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	                      BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)bpm_str, CENTER_MODE);
	                      bpm_last_drawn = bpm_int;
	                  }
	              }
	          }
	          last_peak_n = sample_n;
	          refractory  = REFRACTORY_SAMPLES;
	      }
	      else if (refractory < REFRACTORY_SAMPLES - 40)
	      {
	          HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13, GPIO_PIN_RESET);
	      }

	      /* ===== Watchdog: clear BPM if no beats for 3 seconds ===== */
	            if (last_peak_n > 0 && (sample_n - last_peak_n) > 3 * FS)
	            {
	                if (bpm_last_drawn != 0) {
	                    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	                    BSP_LCD_FillRect(50, 35, 140, 30);
	                    BSP_LCD_SetTextColor(LCD_COLOR_DARKGRAY);
	                    BSP_LCD_SetBackColor(LCD_COLOR_BLACK);
	                    BSP_LCD_DisplayStringAt(0, 35, (uint8_t*)"  -- BPM", CENTER_MODE);
	                    bpm_last_drawn = 0;

	                    /* Also reset HR averaging buffer so old values don't pollute */
	                    hr_buf_idx   = 0;
	                    hr_buf_count = 0;
	                }
	            }

	  }
	  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
/**
  * @brief  Period elapsed callback in non blocking mode
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    HAL_IncTick();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */

#ifdef USE_FULL_ASSERT
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
