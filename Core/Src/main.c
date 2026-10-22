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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include "st7789.h"
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

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

SPI_HandleTypeDef hspi1;

/* Definitions for UI_Task */
osThreadId_t UI_TaskHandle;
const osThreadAttr_t UI_Task_attributes = {
  .name = "UI_Task",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Temp_Task */
osThreadId_t Temp_TaskHandle;
const osThreadAttr_t Temp_Task_attributes = {
  .name = "Temp_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_Task */
osThreadId_t CAN_TaskHandle;
const osThreadAttr_t CAN_Task_attributes = {
  .name = "CAN_Task",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TempQueue */
osMessageQueueId_t TempQueueHandle;
const osMessageQueueAttr_t TempQueue_attributes = {
  .name = "TempQueue"
};
/* USER CODE BEGIN PV */
float debug_temp = 0.0f;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void CAN_Config_And_Start(void)
{
    CAN_FilterTypeDef filter;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0; // Đẩy dữ liệu vào Hòm thư 0
    filter.SlaveStartFilterBank = 14; // Điểm chia ranh giới Filter cho CAN2

    // Cấp phép cho CAN1 (Nhận tin nhắn vào FIFO0)
    filter.FilterBank = 0;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // Cấp phép cho CAN2 (Nhận tin nhắn vào FIFO0)
    filter.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// đọc cảm ứng
uint8_t TP_Read_XY(uint16_t *x, uint16_t *y)
{
    uint8_t tx_x[3] = {0x90, 0, 0};
    uint8_t tx_y[3] = {0xD0, 0, 0};
    uint8_t rx_x[3] = {0}, rx_y[3] = {0};
    uint16_t raw_x = 0, raw_y = 0;

    vTaskSuspendAll();

    // Hạ tốc độ SPI cho cảm ứng
    __HAL_SPI_DISABLE(&hspi1);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR_Msk) | SPI_BAUDRATEPRESCALER_128;
    __HAL_SPI_ENABLE(&hspi1);


    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    // dò tọa độ
    HAL_SPI_TransmitReceive(&hspi1, tx_x, rx_x, 3, 10);
    raw_x = ((rx_x[1] << 8) | rx_x[2]) >> 4;
    HAL_SPI_TransmitReceive(&hspi1, tx_y, rx_y, 3, 10);
    raw_y = ((rx_y[1] << 8) | rx_y[2]) >> 4;


    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

    // nâng tốc độ lên
    __HAL_SPI_DISABLE(&hspi1);
    SPI1->CR1 = (SPI1->CR1 & ~SPI_CR1_BR_Msk) | SPI_BAUDRATEPRESCALER_8;
    __HAL_SPI_ENABLE(&hspi1);

    xTaskResumeAll();

    // Loại bỏ rác vật lý (Nếu không chạm, điện áp thường nảy lên 4095 hoặc 0)
    if (raw_x == 0 || raw_x >= 4095 || raw_y == 0 || raw_y >= 4095) return 0;


    *x = raw_x;
    *y = raw_y;

    return 1;
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
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  CAN_Config_And_Start();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of TempQueue */
  TempQueueHandle = osMessageQueueNew (16, sizeof(uint32_t), &TempQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of UI_Task */
  UI_TaskHandle = osThreadNew(StartDefaultTask, NULL, &UI_Task_attributes);

  /* creation of Temp_Task */
  Temp_TaskHandle = osThreadNew(StartTask02, NULL, &Temp_Task_attributes);

  /* creation of CAN_Task */
  CAN_TaskHandle = osThreadNew(StartTask03, NULL, &CAN_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
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
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = ENABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief CAN2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 6;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = ENABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

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
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LCD_RST_Pin|LCD_CS_Pin|LCD_DC_Pin|TP_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_BL_GPIO_Port, LCD_BL_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LCD_RST_Pin LCD_BL_Pin LCD_CS_Pin TP_CS_Pin */
  GPIO_InitStruct.Pin = LCD_RST_Pin|LCD_BL_Pin|LCD_CS_Pin|TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_DC_Pin */
  GPIO_InitStruct.Pin = LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LCD_DC_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the UI_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	osDelay(200);
	  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);

	  ST7789_Init();

	  ST7789_Fill_Color(BLACK);

	  ST7789_WriteString(20, 10, "[NHOM 04]", Font_11x18, WHITE, BLACK);
	  ST7789_WriteString(10, 60, "Internal Temp: ", Font_11x18, WHITE, BLACK);

	  ST7789_Fill(20, 150, 100, 200, GREEN);
	  ST7789_WriteString(35, 165, "PLAY", Font_11x18, BLACK, GREEN);


      ST7789_Fill(140, 150, 220, 200, RED);
	  ST7789_WriteString(150, 165, "PAUSE", Font_11x18, WHITE, RED);

	  float current_temp = 0.0f;
	  char temp_str[20];
  /* Infinite loop */
  for(;;)
  {
	  // CẬP NHẬT NHIỆT ĐỘ TỪ QUEUE
	        if (osMessageQueueGet(TempQueueHandle, &current_temp, NULL, 10) == osOK)
	        {
	            debug_temp = current_temp; // Lưu vào biến toàn cục cho CAN gửi
	            sprintf(temp_str, "%.1f C   ", current_temp);
	            vTaskSuspendAll();
	            ST7789_WriteString(150, 60, temp_str, Font_11x18, RED, BLACK);
	            xTaskResumeAll();
	        }

	        // Nhường CPU 40ms để xoay vòng quét
	        osDelay(50);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the Temp_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
	uint32_t adc_val;
	float voltage;
	float temperature;
  /* Infinite loop */
  for(;;)
  {
	  // Khởi động bộ chuyển đổi ADC
	      HAL_ADC_Start(&hadc1);

	      // Chờ ADC đọc xong (Timeout 100ms)
	      if (HAL_ADC_PollForConversion(&hadc1, 100) == HAL_OK)
	      {
	        // Lấy giá trị thô (0 - 4095)
	        adc_val = HAL_ADC_GetValue(&hadc1);

	        // Tính toán điện áp và nhiệt độ
	        voltage = (float)adc_val * 3.3f / 4095.0f;
	        temperature = ((voltage - 0.76f) / 0.0025f) + 25.0f;

	        // Đẩy giá trị nhiệt độ vào Queue để gửi sang UI_Task
	        osMessageQueuePut(TempQueueHandle, &temperature, 0, 0);
	      }

	      // Tắt ADC để tiết kiệm năng lượng
	      HAL_ADC_Stop(&hadc1);

	      // 1 giây (1000ms) rồi đo lại
	      osDelay(1000);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Touch_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
	osDelay(1000);
	// Khởi tạo cảm ứng
	static uint8_t play_locked = 0;
	uint16_t touch_x, touch_y;
  /* Infinite loop */
  for(;;)
  {
	  if (TP_Read_XY(&touch_x, &touch_y))
	  	        {
		              if (touch_x >= 0 && touch_x <= 500 && touch_y >= 1300 && touch_y <= 1650)
		              {
		            	  if (play_locked == 0)
		            	                    {
		            	                        play_locked = 1;

		            	                        vTaskSuspendAll();
		            	                        ST7789_WriteString(10, 230, "CAN: TX...      ", Font_11x18, YELLOW, BLACK);
		            	                        xTaskResumeAll();

		            	                        osDelay(300);

		            	                        HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);

		            	                        CAN_TxHeaderTypeDef TxHeader;
		            	                        TxHeader.StdId = 0x123;
		            	                        TxHeader.ExtId = 0x00;
		            	                        TxHeader.IDE = CAN_ID_STD;
		            	                        TxHeader.RTR = CAN_RTR_DATA;
		            	                        TxHeader.DLC = 4;
		            	                        TxHeader.TransmitGlobalTime = DISABLE;

		            	                        uint32_t TxMailbox;
		            	                        uint8_t TxData[4];
		            	                        memcpy(TxData, &debug_temp, 4);

		            	                        // lấy dữ liệu từ CAN1
		            	                        if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) == HAL_OK)
		            	                        {
		            	                            osDelay(30);

		            	                            // Xem có nhận được ACK từ dây không?
		            	                            if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) != 3)
		            	                            {
		            	                                vTaskSuspendAll();
		            	                                ST7789_WriteString(10, 230, "CAN1: TX Error! ", Font_11x18, RED, BLACK);
		            	                                xTaskResumeAll();
		            	                            }
		            	                            else
		            	                            {
		            	                                // nếu được thì kiểm tra CAN2
		            	                                if (HAL_CAN_GetRxFifoFillLevel(&hcan2, CAN_RX_FIFO0) > 0)
		            	                                {
		            	                                    CAN_RxHeaderTypeDef RxHeader;
		            	                                    uint8_t RxData[4];
		            	                                    float received_temp;

		            	                                    HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData);
		            	                                    memcpy(&received_temp, RxData, 4);

		            	                                    int p_nguyen = (int)received_temp;
		            	                                    int p_le = (int)((received_temp - p_nguyen) * 10);
		            	                                    if(p_le < 0) p_le = -p_le;

		            	                                    char can_msg[30];
		            	                                    sprintf(can_msg, "CAN2 RX: %d.%d C", p_nguyen, p_le);

		            	                                    vTaskSuspendAll();
		            	                                    ST7789_WriteString(10, 230, can_msg, Font_11x18, GREEN, BLACK);
		            	                                    xTaskResumeAll();
		            	                                }
		            	                                else
		            	                                {
		            	                                    vTaskSuspendAll();
		            	                                    ST7789_WriteString(10, 230, "CAN2: No Data!  ", Font_11x18, RED, BLACK);
		            	                                    xTaskResumeAll();
		            	                                }
		            	                            }
		            	                        }
		            	                    }
		              }


		              else if (touch_x >= 1600 && touch_x <= 1900 && touch_y >= 1300 && touch_y <= 1650)
		                          {
		                              vTaskSuspendAll();
		                              ST7789_WriteString(10, 230, "System SLEEPING!", Font_11x18, RED, BLACK);
		                              xTaskResumeAll();

		                              osDelay(500);

		                              // Tắt nhịp đếm của HAL (TIM1)
		                              HAL_SuspendTick();

		                              // TẮT CHUÔNG BÁO THỨC CỦA FREERTOS
		                              SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

		                              // Dọn sạch rác ngắt của nút PA0
		                              __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

		                              // ĐI NGỦ SÂU
		                              HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);

		                              // Khi nhấn nút WAKEUP

		                              // Bật lại chuông báo thức của FreeRTOS
		                              SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

		                              // Bật lại nhịp đếm của HAL
		                              HAL_ResumeTick();

		                              // Bật lại LCD
		                              HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
		                              __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);

		                              vTaskSuspendAll();
		                              ST7789_WriteString(10, 230, "System AWAKE!   ", Font_11x18, GREEN, BLACK);
		                              xTaskResumeAll();

		                              osDelay(500);
		                          }
		              else
		              {
		                  play_locked = 0;
		              }
		          }
		          else
		          {
		              play_locked = 0;
		          }

		          osDelay(40);
  }
  /* USER CODE END StartTask03 */
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
  if (htim->Instance == TIM1)
  {
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
