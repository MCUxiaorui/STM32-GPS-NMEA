/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
#include "minmea.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct GPS_Data {
  struct minmea_time time;
  struct minmea_date date;
  float latitude;
  float longitude;
  float altitude;
  float speed;
  float err_latitude;
  float err_longitude;
  float err_altitude;
  uint8_t total_sats;
  uint8_t fix_type;
  uint8_t fix_quality;
  char nmeaBuffer[128];
  int bufferIndex;
  bool receiving;
  char c;
  struct minmea_float true_track_degrees;
  struct minmea_float magnetic_track_degrees;
  struct minmea_float speed_knots;
  struct minmea_float speed_kph;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void parseNEMA(const char *nmeaBuffer, struct GPS_Data *gps);
void GPS_Data_Receive(UART_HandleTypeDef *huart, struct GPS_Data *gps);
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  struct GPS_Data gpsData;
  gpsData.bufferIndex = 0;
  gpsData.receiving = false;
  char uartBuffer[128];
  /*
  *  int bufferIndex;
  bool receiving;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    GPS_Data_Receive(&huart1, &gpsData);
    if (!gpsData.receiving) {
      float latitude = gpsData.latitude;
      float longitude = gpsData.longitude;
      float altitude = gpsData.altitude;
      float speed = gpsData.speed;
      uint8_t hours = gpsData.time.hours;
      uint8_t minutes = gpsData.time.minutes;
      uint8_t seconds = gpsData.time.seconds;
      uint8_t day = gpsData.date.day;
      uint8_t month = gpsData.date.month;
      uint8_t year = gpsData.date.year;
      int_least32_t True_track_deg = gpsData.true_track_degrees.value / gpsData.true_track_degrees.scale;
      int_least32_t Magnetic_track_deg = gpsData.magnetic_track_degrees.value / gpsData.magnetic_track_degrees.scale;
      int_least32_t speed_knots = gpsData.speed_knots.value / gpsData.speed_knots.scale;
      int_least32_t speed_kph = gpsData.speed_kph.value / gpsData.speed_kph.scale;

      uint8_t a = 0;

        // Big enough for your message
      int len = snprintf(uartBuffer, sizeof(uartBuffer),
          "Lat: %f, Lon: %f, Alt: %f m, Spd: %f m/s, "
          "Time: %02u:%02u:%02u, Date: %02u-%02u-%02u\r\n",
          latitude,
          longitude,
          altitude,
          speed,
          hours,
          minutes,
          seconds,
          day,
          month,
          year
      );
      HAL_UART_Transmit(&huart1, uartBuffer, len, 500);
      HAL_Delay(1000);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  RTC_DateTypeDef DateToUpdate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_ETRMODE2;
  sClockSourceConfig.ClockPolarity = TIM_CLOCKPOLARITY_NONINVERTED;
  sClockSourceConfig.ClockPrescaler = TIM_CLOCKPRESCALER_DIV1;
  sClockSourceConfig.ClockFilter = 0;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  huart1.Init.BaudRate = 9600;
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

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}
/*
 *
*  char nmeaBuffer[128];
  int bufferIndex = 0;
  bool receiving = false;
  char c;
 *
/* USER CODE BEGIN 4 */
// void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
//   int a = 0;
// }

void GPS_Data_Receive(UART_HandleTypeDef *huart, struct GPS_Data *gps) {
  static uint8_t gpsRxBuffer[512];
  HAL_UARTEx_ReceiveToIdle_DMA(huart, gpsRxBuffer, sizeof(gpsRxBuffer));

  char *token = strtok(gpsRxBuffer, "\r\n");  // split on CR and/or LF
  while (token) {
    if (token[0] == '$') {
      // this is a full NMEA sentence
      parseNEMA(token, gps);
    }
    token = strtok(NULL, "\r\n");
  }



  // if (HAL_UART_Receive(huart, (uint8_t *) &gps->c, 1, HAL_MAX_DELAY) == HAL_OK) {
  //   if (!gps->receiving) {
  //     if (gps->c == '$') {
  //       gps->receiving = true;
  //       gps->bufferIndex = 0;
  //       gps->nmeaBuffer[gps->bufferIndex++] = gps->c;
  //     }
  //   } else {
  //     if (gps->bufferIndex < 128 - 1) {
  //       gps->nmeaBuffer[gps->bufferIndex++] = gps->c;
  //
  //       // Replace with your chosen ending condition:
  //       if (gps->c == '\n') // or '\r' or '*' etc.
  //       {
  //         gps->nmeaBuffer[gps->bufferIndex] = '\0'; // null terminate
  //         gps->receiving = false;
  //
  //         // Use the sentence here
  //         if (minmea_check(gps->nmeaBuffer, true)) {
  //           parseNEMA(gps->nmeaBuffer, gps);
  //         }
  //       }
  //     } else {
  //       // buffer overflow protection
  //       gps->receiving = false;
  //       gps->bufferIndex = 0;
  //     }
  //   }
  // }
}

void parseNEMA(const char *nmeaBuffer, struct GPS_Data *gps) {
  switch (minmea_sentence_id(nmeaBuffer, true)) {
    case MINMEA_SENTENCE_RMC: {
      struct minmea_sentence_rmc frame;
      if (minmea_parse_rmc(&frame, nmeaBuffer)) {
        gps->latitude = minmea_tocoord(&frame.latitude);
        gps->longitude =minmea_tocoord(&frame.longitude);
        gps->speed =minmea_tofloat(&frame.speed);
        gps->date.day = frame.date.day;
        gps->date.month = frame.date.month;
        gps->date.year = frame.date.year;
        gps->time.hours = frame.time.hours;
        gps->time.minutes = frame.time.minutes;
        gps->time.seconds = frame.time.seconds;
        gps->time.microseconds = frame.time.microseconds;

      }
    } break;

    case MINMEA_SENTENCE_GGA: {
      struct minmea_sentence_gga frame;
      if (minmea_parse_gga(&frame, nmeaBuffer)) {
        gps->fix_quality = frame.fix_quality;
      }
    } break;

    case MINMEA_SENTENCE_GSV: {
      struct minmea_sentence_gsv frame;
      if (minmea_parse_gsv(&frame, nmeaBuffer)) {
        gps->total_sats = frame.total_sats;
      }
    } break;

    case MINMEA_SENTENCE_GBS: {
      struct minmea_sentence_gbs frame;
      if (minmea_parse_gbs(&frame, nmeaBuffer)) {
        gps->err_latitude = minmea_tocoord(&frame.err_latitude);
        gps->err_longitude = minmea_tocoord(&frame.err_longitude);
        gps->err_altitude = minmea_tofloat(&frame.err_altitude);
      }
    } break;

    case MINMEA_SENTENCE_GLL: {
      break;
      struct minmea_sentence_gll frame;
      if (minmea_parse_gll(&frame, nmeaBuffer)) {
      }
    }

    case MINMEA_SENTENCE_GSA: {
      break;
      struct minmea_sentence_gsa frame;
      if (minmea_parse_gsa(&frame, nmeaBuffer)) {
        gps->fix_type = frame.fix_type;
      }
    }

    case MINMEA_SENTENCE_GST: {
      break;
      struct minmea_sentence_gst frame;
      if (minmea_parse_gst(&frame, nmeaBuffer)) {
      }
    }

    case MINMEA_SENTENCE_VTG: {
      struct minmea_sentence_vtg frame;
      if (minmea_parse_vtg(&frame, nmeaBuffer)) {
        gps->true_track_degrees = frame.true_track_degrees;
        gps->magnetic_track_degrees = frame.magnetic_track_degrees;
        gps->speed_knots = frame.speed_knots;
        gps->speed_kph = frame.speed_kph;
      }
    }

    case MINMEA_SENTENCE_ZDA: {
      break;
      struct minmea_sentence_zda frame;
      if (minmea_parse_zda(&frame, nmeaBuffer)) {
      }
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
