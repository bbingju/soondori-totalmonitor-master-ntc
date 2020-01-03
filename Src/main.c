/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "0_GlobalValue.h"
#include "0_StartDisplayTask.h"
#include "job_task.h"
#include "internal_uart_task.h"
#include "external_uart_task.h"
#include "system_task.h"
#include "0_StartRateTask.h"
#include "0_soonFlashMemory.h"
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
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;
DMA_HandleTypeDef hdma_sdio_rx;
DMA_HandleTypeDef hdma_sdio_tx;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

osSemaphoreId myBinarySemModeHandle;
osSemaphoreId myBinarySemUpHandle;
osSemaphoreId myBinarySemDownHandle;
/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
SYSTEM_STRUCT SysProperties;
osThreadId init_task_id;

/* osThreadId JobTaskHandle; */
uint32_t JobTaskBuffer[256];
osStaticThreadDef_t JobTaskControlBlock;

uint32_t DisplayTaskBuffer[128];
osStaticThreadDef_t DisplayTaskControlBlock;

uint32_t RateTaskBuffer[512];
osStaticThreadDef_t RateTaskControlBlock;

uint32_t InternalRxTaskBuffer[256];
osStaticThreadDef_t InternalRxTaskControlBlock;

uint32_t SystemTaskBuffer[256];
osStaticThreadDef_t SystemTaskControlBlock;

uint32_t ExternalRxTaskBuffer[512];
osStaticThreadDef_t ExternalRxControlBlock;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_RTC_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_IWDG_Init(void);
void init_task(void const * argument);
extern void StartDisplayTask(void const * argument);
extern void external_rx_task(void const * argument);
extern void external_tx_task(void const * argument);
extern void StartRateTask(void const * argument);
extern void internal_uart_task(void const * argument);
extern void system_task(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  uint32_t read = 0;

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
  MX_ADC2_Init();
  MX_RTC_Init();
  MX_SDIO_SD_Init();
  MX_SPI1_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  /* MX_IWDG_Init(); */
  /* USER CODE BEGIN 2 */

  /* Check if system reset is caused by IWDG */
  if (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) == SET) {
      __HAL_RCC_CLEAR_RESET_FLAGS();
      DBG_LOG("System reset caused by watchdog.\n");
  }

  SysProperties.displayMode = DPM_NORMAL;
  for (int j = 0; j < 4; j++)
  {
      for (int i = 0; i < 32; i++)
      {
          TestData.temperature[j][i].Float = (float)0.0;
          TestData.sensorState[j][i] = LDM_DONOT_CONNECT;
      }
  }

  INIT_SLOT_PROPERTIES(SysProperties.slots, MAX_SLOT_NUM);

  SysProperties.interval_ms = 1000;
  SysProperties.start_flag = FALSE;

  read = ReadFlash(FLASH_SAVE_CHK);

  if(read != FLASH_SAVE_FLAG)		//플래시에 기록이 없을경우
  {
      TestData.rtdCalibrationConst.Float = (float)0;
      doFlashWriteRevision();
  }
  else		//Flash 에 저장된 값이 揚獵? 경우
  {
      TestData.rtdCalibrationConst.UI32= ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);
  }

  HAL_GPIO_WritePin(SLAVE_CS0_GPIO_Port, SLAVE_CS0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLAVE_CS1_GPIO_Port, SLAVE_CS1_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLAVE_CS2_GPIO_Port, SLAVE_CS2_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLAVE_CS3_GPIO_Port, SLAVE_CS3_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLAVE_CS4_GPIO_Port, SLAVE_CS4_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(SLAVE_CS5_GPIO_Port, SLAVE_CS5_Pin, GPIO_PIN_RESET);

  SysProperties.mcuUUID[0].UI32 = HAL_GetUIDw0();
  SysProperties.mcuUUID[1].UI32 = HAL_GetUIDw1();
  SysProperties.mcuUUID[2].UI32 = HAL_GetUIDw2();

  HAL_Delay(5);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of myBinarySemMode */
  osSemaphoreDef(myBinarySemMode);
  myBinarySemModeHandle = osSemaphoreCreate(osSemaphore(myBinarySemMode), 1);

  /* definition and creation of myBinarySemUp */
  osSemaphoreDef(myBinarySemUp);
  myBinarySemUpHandle = osSemaphoreCreate(osSemaphore(myBinarySemUp), 1);

  /* definition and creation of myBinarySemDown */
  osSemaphoreDef(myBinarySemDown);
  myBinarySemDownHandle = osSemaphoreCreate(osSemaphore(myBinarySemDown), 1);

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
  osThreadDef(InitTask, init_task, osPriorityNormal, 0, 128);
  init_task_id = osThreadCreate(osThread(InitTask), NULL);

  /* HAL_IWDG_Refresh(&hiwdg); */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_VBAT;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only 
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */
	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0) != 0x32F2)
	{
  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date 
  */
  sTime.Hours = 0;
  sTime.Minutes = 0;
  sTime.Seconds = 0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 1;
  sDate.Year = 19;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the TimeStamp 
  */
  if (HAL_RTCEx_SetTimeStamp(&hrtc, RTC_TIMESTAMPEDGE_RISING, RTC_TIMESTAMPPIN_DEFAULT) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */
		HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
	}

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SDIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  huart1.Init.BaudRate = 921600;
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
  USART_TypeDef *regs = huart1.Instance;
  SET_BIT(regs->CR1, USART_CR1_IDLEIE);
  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */
  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */
  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */
  USART_TypeDef *regs = huart2.Instance;
  SET_BIT(regs->CR1, USART_CR1_IDLEIE);
  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */
  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */
  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 4;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */
  /* USER CODE END USB_OTG_FS_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream6_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream6_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, SD_LED_Pin|POWER_LED_Pin|SLAVE_OE1_Pin|UART_EN_BT_Pin 
                          |UART_EN_SLOT_Pin|DIGI2_LATCH_Pin|DIGI2_ENABLE_Pin|DIGI1_ENABLE_Pin 
                          |DIGI1_LATCH_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RESET_BUTTON_Pin|RELAY_SEL_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUZZER_SEL_Pin|END_REG2_Pin|CSP_1DE_Pin|DEV_ENABLE_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, END_REG1_Pin|BUFFER_OE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CSP_2DE_Pin|DIGI2_SCL_Pin|DIGI2_SDA_Pin|SLAVE_CS5_Pin 
                          |SLAVE_CS4_Pin|RS485_EN_Pin|DIGI1_SCL_Pin|DIGI1_SDA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLAVE_DEBUGE_GPIO_Port, SLAVE_DEBUGE_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, SLAVE_CS0_Pin|SLAVE_CS1_Pin|SLAVE_CS2_Pin|SLAVE_CS3_Pin 
                          |SLAVE_OE_Pin|DEV_BEAT_OUT_Pin|BLUETOOTH_FACTORY_RESET_Pin|BLUETOOTH_MODE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDIO_POWER_GPIO_Port, SDIO_POWER_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : SD_LED_Pin POWER_LED_Pin DIGI2_LATCH_Pin DIGI1_LATCH_Pin */
  GPIO_InitStruct.Pin = SD_LED_Pin|POWER_LED_Pin|DIGI2_LATCH_Pin|DIGI1_LATCH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : MODE_BUTTON_Pin */
  GPIO_InitStruct.Pin = MODE_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MODE_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : UP_BUTTON_Pin DOWN_BUTTON_Pin */
  GPIO_InitStruct.Pin = UP_BUTTON_Pin|DOWN_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_BUTTON_Pin */
  GPIO_InitStruct.Pin = RESET_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_SEL_Pin RELAY_SEL_Pin END_REG2_Pin CSP_1DE_Pin 
                           DEV_ENABLE_OUT_Pin */
  GPIO_InitStruct.Pin = BUZZER_SEL_Pin|RELAY_SEL_Pin|END_REG2_Pin|CSP_1DE_Pin 
                          |DEV_ENABLE_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : END_REG1_Pin BUFFER_OE_Pin */
  GPIO_InitStruct.Pin = END_REG1_Pin|BUFFER_OE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CSP_2DE_Pin SLAVE_DEBUGE_Pin RS485_EN_Pin */
  GPIO_InitStruct.Pin = CSP_2DE_Pin|SLAVE_DEBUGE_Pin|RS485_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SLAVE_OE1_Pin UART_EN_BT_Pin UART_EN_SLOT_Pin */
  GPIO_InitStruct.Pin = SLAVE_OE1_Pin|UART_EN_BT_Pin|UART_EN_SLOT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGI2_ENABLE_Pin DIGI1_ENABLE_Pin */
  GPIO_InitStruct.Pin = DIGI2_ENABLE_Pin|DIGI1_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGI2_SCL_Pin DIGI2_SDA_Pin */
  GPIO_InitStruct.Pin = DIGI2_SCL_Pin|DIGI2_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SLAVE_CS5_Pin SLAVE_CS4_Pin */
  GPIO_InitStruct.Pin = SLAVE_CS5_Pin|SLAVE_CS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : SLAVE_CS0_Pin SLAVE_CS1_Pin SLAVE_CS2_Pin SLAVE_CS3_Pin */
  GPIO_InitStruct.Pin = SLAVE_CS0_Pin|SLAVE_CS1_Pin|SLAVE_CS2_Pin|SLAVE_CS3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SLAVE_OE_Pin SDIO_POWER_Pin */
  GPIO_InitStruct.Pin = SLAVE_OE_Pin|SDIO_POWER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DEV_DEBUGE_Pin MCU1_MCU2_CHECK_Pin SDIO_DECTECT_Pin */
  GPIO_InitStruct.Pin = DEV_DEBUGE_Pin|MCU1_MCU2_CHECK_Pin|SDIO_DECTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : DEV_BEAT_OUT_Pin BLUETOOTH_FACTORY_RESET_Pin BLUETOOTH_MODE_Pin */
  GPIO_InitStruct.Pin = DEV_BEAT_OUT_Pin|BLUETOOTH_FACTORY_RESET_Pin|BLUETOOTH_MODE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : DEV_BEAT_IN_Pin */
  GPIO_InitStruct.Pin = DEV_BEAT_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DEV_BEAT_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SMPS2_DECTECT_Pin SMPS1_DECTECT_Pin */
  GPIO_InitStruct.Pin = SMPS2_DECTECT_Pin|SMPS1_DECTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGI1_SCL_Pin DIGI1_SDA_Pin */
  GPIO_InitStruct.Pin = DIGI1_SCL_Pin|DIGI1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 6, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    switch (GPIO_Pin) {
    case MODE_BUTTON_Pin:
        osSemaphoreRelease(myBinarySemModeHandle);
        break;
    case UP_BUTTON_Pin:
        osSemaphoreRelease(myBinarySemUpHandle);
        break;
    case DOWN_BUTTON_Pin:
        osSemaphoreRelease(myBinarySemDownHandle);
        break;
    }
}

/* void hard_fault_handler_c(uint32_t *hardfault_args, uint32_t r4, uint32_t r5, uint32_t r6) */
/* { */
/*     DBG_LOG("\n[Hard Fault]\n"); // After Joseph Yiu */

/*     DBG_LOG("r0 = %08X, r1 = %08X, r2 = %08X, r3 = %08X\n", hardfault_args[0], */
/*             hardfault_args[1], hardfault_args[2], hardfault_args[3]); */
/*     DBG_LOG("r4 = %08X, r5 = %08X, r6 = %08X, sp = %08X\n", r4, r5, r6, */
/*             (unsigned int)&hardfault_args[8]); */
/*     DBG_LOG("r12= %08X, lr = %08X, pc = %08X, psr= %08X\n", hardfault_args[4], */
/*             hardfault_args[5], hardfault_args[6], hardfault_args[7]); */

/*     DBG_LOG("bfar=%08X, cfsr=%08X, hfsr=%08X, dfsr=%08X, afsr=%08X\n", */
/*             *((volatile unsigned int *)(0xE000ED38)), */
/*             *((volatile unsigned int *)(0xE000ED28)), */
/*             *((volatile unsigned int *)(0xE000ED2C)), */
/*             *((volatile unsigned int *)(0xE000ED30)), */
/*             *((volatile unsigned int *)(0xE000ED3C))); */

/*     while (1) */
/*         ; */
/* } */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDisplayTask */
/**
  * @brief  Function implementing the InitTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDisplayTask */
void init_task(void const *argument)
{
    MX_FATFS_Init();

    osThreadStaticDef(JobTask, job_task, osPriorityNormal, 0, 256,
		    JobTaskBuffer, &JobTaskControlBlock);
    osThreadCreate(osThread(JobTask), NULL);

    osThreadStaticDef(myDisplayTask, StartDisplayTask, osPriorityNormal, 0, 128,
		    DisplayTaskBuffer, &DisplayTaskControlBlock);
    osThreadCreate(osThread(myDisplayTask), NULL);

    osThreadStaticDef(myRateTask, StartRateTask, osPriorityNormal, 0, 512,
		    RateTaskBuffer, &RateTaskControlBlock);
    osThreadCreate(osThread(myRateTask), NULL);

    osThreadStaticDef(IntRxTask, internal_rx_task, osPriorityNormal, 0, 256,
		    InternalRxTaskBuffer, &InternalRxTaskControlBlock);
    osThreadCreate(osThread(IntRxTask), NULL);

    osThreadStaticDef(SystemTask, system_task, osPriorityNormal, 0, 256,
		    SystemTaskBuffer, &SystemTaskControlBlock);
    osThreadCreate(osThread(SystemTask), NULL);

    osThreadStaticDef(ExtRxTask, external_rx_task, osPriorityNormal, 0, 512,
		    ExternalRxTaskBuffer, &ExternalRxControlBlock);
    osThreadCreate(osThread(ExtRxTask), NULL);

    osThreadTerminate(init_task_id);
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

void vApplicationIdleHook( void )
{
    static uint32_t elapsed_tick = 0;

    if (osKernelSysTick() - elapsed_tick > osKernelSysTickMicroSec(400)) {
        elapsed_tick = osKernelSysTick();
        HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
    }

    HAL_IWDG_Refresh(&hiwdg);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
