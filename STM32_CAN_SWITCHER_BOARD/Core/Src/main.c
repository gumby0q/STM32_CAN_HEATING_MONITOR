/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include "DallasTemperature.h"
#include "OneWire.h"
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

void reset_DS_sensor(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
  these below suspends warning but may be not good
  https://stackoverflow.com/questions/73742774/gcc-arm-none-eabi-11-3-is-not-implemented-and-will-always-fail
*/
/* suspends warning >>>> */
int _close(int file) { return -1; }
void _lseek(void) {}
void _read(void) {}
int _fstat_r(void) { return -1; }
int _getpid_r(void) { return -1; }
int _isatty_r(void) { return -1; }
int _kill_r(/* struct _reent */ void  *ptr, int pid, int sig) { return -1; }
// int _write(int file, char *ptr, int len) {
//   int todo;
//   for (todo = 0; todo < len; todo++) {
//     // outbyte (*ptr++);
//   }
//   return len;
// }

/* suspends warning <<<< */

int _write(int file, char *ptr, int len)
{
  // HAL_UART_Transmit(&huart3, (uint8_t *)ptr, len, HAL_MAX_DELAY);
  HAL_UART_Transmit_DMA(&huart3, (uint8_t *)ptr, len);
  uint32_t tickstart = 0U;
  tickstart = HAL_GetTick();

  while (HAL_UART_GetState(&huart3) != HAL_UART_STATE_READY)
  {
    if ((HAL_GetTick() - tickstart) > 500)
    {
      // return OW_TIMEOUT;
      return 0;
    }
  }
  return len;
}


//#define BOILER_TEMP_CAN_ID 0x0d0
#define BOILER_TEMP_CAN_ID 0x0d0
//#define RELAY_CONTROL_CAN_ID 0x080
#define RELAY_CONTROL_CAN_ID 0x090
//#define RELAY_STATUS_CAN_ID 0x081
// #define RELAY_STATUS_CAN_ID 0x091
#define PUMP_STATUS_CAN_ID 0x091


#define DATA_WAIT_TIMEOUT 10


CAN_FilterTypeDef sFilterConfig;
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8];
uint8_t RxData[8];
uint32_t TxMailbox;

// static u8g2_t u8g2;

int32_t temperature;
uint32_t pressure;
uint32_t humidity;
float input_packet_boiler_temperature;
volatile uint8_t relays_status = 0x0;


#define TEMPERATURE_ERROR_TIMEOUT_CNT_MAX  10*1000*5  /* 100 nano 1 tick */ 
volatile uint32_t temperature_error_timeout_cnt = 0;

volatile uint8_t input_packet_boiler_timeout = 0;

/* for human testing */
// #define PUMP_TEMP_HIST_ON   (int16_t)25
// #define PUMP_TEMP_HIST_OFF  (int16_t)22

/* real */
#define PUMP_TEMP_HIST_ON   (int16_t)55
#define PUMP_TEMP_HIST_OFF  (int16_t)40

volatile uint8_t global_pump_flag = 0; /* on-off flag */

volatile uint8_t global_ds_error = 0;

volatile uint8_t temperature1timeout = 0;
volatile uint8_t temperature2timeout = 0;
volatile uint8_t humidity1timeout = 0;
volatile uint8_t humidity2timeout = 0;

#define GLOBAL_TEMPERATURES_ARRAY_LENGTH  5 
volatile uint8_t global_temperatures_array_cnt = 0;
volatile int16_t global_temperatures_array[GLOBAL_TEMPERATURES_ARRAY_LENGTH] = {-127};

// Function to sort the array using Bubble Sort
void bubbleSort(int16_t *arr, uint8_t arr_l)
{
  uint8_t i, j;
  for (i = 0; i < arr_l - 1; i++)
  {
    for (j = 0; j < arr_l - i - 1; j++)
    {
      if (arr[j] > arr[j + 1])
      {
        // Swap temp and arr[i]
        int16_t temp = arr[j];
        arr[j] = arr[j + 1];
        arr[j + 1] = temp;
      }
    }
  }
}

// Function to calculate mean of middle three values
int16_t meanOfMiddleThreeOfFive(int16_t * arr/* , uint8_t arr_l */) {
    uint8_t arr_l = 5;
    
    bubbleSort(arr, arr_l);  // Sort the array
    return (arr[1] + arr[2] + arr[3]) / 3;  // Sum and divide middle three values
}

// int main() {
//     float tempValues[5] = {34.5, 36.1, 37.2, 35.5, 33.8};
//     float mean = meanOfMiddleThree(tempValues, 5);
//     printf("Mean of middle three temperatures: %f\n", mean);
//     return 0;
// }



// function to print a device address
void printAddress(CurrentDeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    char buf[4];
    sprintf(buf, "%02X ", deviceAddress[i]);
    printf(buf);
  }
}

void read_and_send_ds_data(void) {
  printf("Requesting temperatures...");

  char buf[50];
  DT_RequestTemperatures(); // Send the command to get temperatures
  printf("DONE\r\n");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  printf("Temperature for the device 1 (index 0) is: ");

  float ds_temperature = DEVICE_DISCONNECTED_C;
  int8_t err_c = DT_GetTempCByIndex(0, &ds_temperature);
  // float ds_temperature = DT_GetTempCByIndex(0);
  // sprintf(buf, "\n ds_temperature %.4f\r\n", ds_temperature);
  sprintf(buf, "\n err_c %i ds_temperature %f\r\n", err_c, ds_temperature);
  printf(buf);

  float value = ds_temperature;
  int value_int = (int)(value * 1000);               // Convert to an integer with 3 decimal places preserved
  printf("ds_temperature = %d.%d\n", value_int / 1000, value_int % 1000); // Print as two integers

  /* set global_temperatures_array >>> */
  global_temperatures_array[global_temperatures_array_cnt] = (int16_t)ds_temperature;
  global_temperatures_array_cnt++;
  if (global_temperatures_array_cnt >= GLOBAL_TEMPERATURES_ARRAY_LENGTH) {
    global_temperatures_array_cnt = 0;
  }
  /* set global_temperatures_array calc mean temp<<< */

  /* calc mean temp >>> */
  uint8_t temp_arr_l = 5;
  int16_t temp_arr[5] = {0};
  for (uint8_t i = 0; i < temp_arr_l; i++)
  {
    temp_arr[i] = global_temperatures_array[i];
  }
  int16_t mean_temp = meanOfMiddleThreeOfFive(temp_arr);
  printf("mean_temp %d\n", mean_temp);
  // int16_t temp_arr_test[5] = {0, 10, 12, 23, 100};
  // int16_t mean_temp_test = meanOfMiddleThreeOfFive(temp_arr_test);
  // printf("mean_temp_test 15 == %d?\n", mean_temp_test);


  /* calc mean temp <<< */

  /* temperature_error_timeout_cnt <<< */
  if (err_c == 0) {
    temperature_error_timeout_cnt = 0; 
  }
  /* temperature_error_timeout_cnt <<< */

  /* check if pump need to be ON >>> */
  
  uint8_t tmp_flag = global_pump_flag;
  /* if already OFF */
  if (tmp_flag == 0) {
    if (mean_temp >= PUMP_TEMP_HIST_ON) {
      tmp_flag = 1;
    }
  /* if already ON */
  } else {
    if (mean_temp <= PUMP_TEMP_HIST_OFF) {
      tmp_flag = 0;
    }
  }

  if (temperature_error_timeout_cnt >= TEMPERATURE_ERROR_TIMEOUT_CNT_MAX) {
    printf("temperature_error_timeout_cnt > MAX VALUE\n");

    /* force ON */
    tmp_flag = 1; 
  }

  global_pump_flag = tmp_flag;

  if (global_pump_flag > 0) {
    /* low lvl active */
    HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, 0);
  } else {
    HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, 1);
  }

  /* check if pump need to be ON <<< */


  TxHeader.StdId = BOILER_TEMP_CAN_ID;
  TxHeader.DLC = 5;
  uint8_t protection_staus = HAL_GPIO_ReadPin(ONEWIRE_PROTECTION_Input_GPIO_Port, ONEWIRE_PROTECTION_Input_Pin);
  printf("ds_short circuit = %d\n", protection_staus); // Print as two integers

  /* set onewire reading error */
  uint8_t temp_sens_error = err_c;
  /* set power short circuit protection */
  if (protection_staus == 0) { /* low means protection was triggered */
    temp_sens_error = temp_sens_error | 0b10000000;
  }

  TxData[0] = temp_sens_error;
  
  /* copy temperature value */
  for (int i=0; i<4 ;++i) {
    TxData[i + 1] = ((uint8_t*)&ds_temperature)[i];
  }
  HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);


  /* try to reset ds setting after disconnect */
  if (global_ds_error != 0) {
    if (temp_sens_error == 0) {
      printf("\ntry to reset ds settings\n");
      reset_DS_sensor();
    }
  }
  global_ds_error = temp_sens_error;


  // HAL_Delay(2500);
  // /* send relays status */
  // TxHeader.StdId = RELAY_STATUS_CAN_ID;
  // TxHeader.DLC = 1;
  // TxData[0] = relays_status;
  // HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void reset_DS_sensor(void)
{
  
  // locate devices on the bus
  char buf[30];

  // arrays to hold device address
  CurrentDeviceAddress insideThermometer;

  uint8_t ds_error = 0;

  OW_Reset();

  if (OW_Reset() == OW_OK)
  {
    printf("[%8lu] OneWire 1 line devices are present :)\r\n", HAL_GetTick());
  }
  else
  {
    printf("[%8lu] OneWire 1 line no devices :(\r\n", HAL_GetTick());
  }

  DT_SetWaitForConversion(1);

  printf("Locating devices...\n");
  ds_error = DT_Begin();
  if (ds_error != DS_OK)
  {
    sprintf(buf, "DT_Begin err %d \n", ds_error);
    printf(buf);
  }

  printf("Found ");
  sprintf(buf, "%d devices.\n", DT_GetDeviceCount());
  printf(buf);

  // report parasite power requirements
  printf("Parasite power is: ");
  if (DT_IsParasitePowerMode())
    printf("ON\r\n");
  else
    printf("OFF\r\n");

  if (!DT_GetAddress(insideThermometer, 0))
    printf("Unable to find address for Device 0\r\n");

  printf("Device 0 Address: ");
  printAddress(insideThermometer);
  printf("\r\n");

  // set the resolution to 12 bit (Each Dallas/Maxim device is capable of several different resolutions)
  DT_SetResolution(insideThermometer, 12, true);

  printf("Device 0 Resolution: ");
  sprintf(buf, "%d", DT_GetResolution(insideThermometer));
  printf(buf);
  printf("\r\n");
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
  MX_CAN_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  
  /* they are open drain and ON if low lvl */
  HAL_GPIO_WritePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin, 1);
  HAL_GPIO_WritePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin, 1);

  HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_RESET);

  sFilterConfig.FilterBank = 0;
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig.FilterIdHigh = 0x0000;
  sFilterConfig.FilterIdLow = 0x0000;
  sFilterConfig.FilterMaskIdHigh = 0x0000;
  sFilterConfig.FilterMaskIdLow = 0x0000;
  sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig.FilterActivation = ENABLE;
  sFilterConfig.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
  {
    /* Filter configuration Error */
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    /* Start Error */
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY) != HAL_OK)
  {
    /* Notification Error */
    Error_Handler();
  }

  // TxHeader.StdId = BOILER_TEMP_CAN_ID;
  // TxHeader.ExtId = 0x00;
  // TxHeader.RTR = CAN_RTR_DATA;
  // TxHeader.IDE = CAN_ID_STD;
  // TxHeader.DLC = 4;
  // TxHeader.TransmitGlobalTime = DISABLE;
  // TxData[0] = 1;
  // TxData[1] = 2;
  // TxData[2] = 3;
  // TxData[3] = 4;
  printf("Debug UART is OK!\r\n");

  // if (OW_Reset() == OW_OK)
  // {
  //   printf("OneWire devices are present :)\r\n");
  // }
  // else
  // {
  //   printf("OneWire no devices :(\r\n");
  // }

  /* TIMER */
  HAL_TIM_Base_Start_IT(&htim4);
  /* TIMER */


  OW_Init();
  reset_DS_sensor();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    HAL_GPIO_TogglePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin);
    // HAL_GPIO_TogglePin(TRIAC_1_GPIO_Port, TRIAC_1_Pin);
    // HAL_GPIO_TogglePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin);
    
    read_and_send_ds_data();

    // HAL_Delay(250);
    // HAL_Delay(1000);
    // HAL_Delay(2500);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL10;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 10;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 40;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 100;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_HalfDuplex_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
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
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(STATUS_LED_GPIO_Port, STATUS_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIAC_1_Pin|TRIAC_2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN_ENABLE_GPIO_Port, CAN_ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : STATUS_LED_Pin */
  GPIO_InitStruct.Pin = STATUS_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(STATUS_LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIAC_1_Pin TRIAC_2_Pin */
  GPIO_InitStruct.Pin = TRIAC_1_Pin|TRIAC_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN_ENABLE_Pin */
  GPIO_InitStruct.Pin = CAN_ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN_ENABLE_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ONEWIRE_PROTECTION_Input_Pin */
  GPIO_InitStruct.Pin = ONEWIRE_PROTECTION_Input_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ONEWIRE_PROTECTION_Input_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan_)
{
//  HAL_GPIO_TogglePin(GPIOC, LED_Pin);
}



void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan_)
{
  // HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData);
//  HAL_GPIO_TogglePin(GPIOC, LED_Pin);
//
//   uint32_t u;
//    u = RxData[0];
//    u += (uint32_t)(RxData[1] << 8);
//    u += (uint32_t)(RxData[2] << 16);
//    u += (uint32_t)(RxData[3] << 24);
//
//
//
//
//  if(RxHeader.StdId==0xf0){
//    pressure = u;
//  }else if(RxHeader.StdId==0xf0+1){
//    temperature = u;
//    temperature1timeout = 0;
//    temperature2timeout = 0;
//  }else if(RxHeader.StdId==0xf0+2){
//    humidity = u;
//    humidity1timeout = 0;
//    humidity2timeout = 0;
//  }else if(RxHeader.StdId==RELAY_CONTROL_CAN_ID){
//    Control_peripheral_relays(RxData[0]);
//  }else if(RxHeader.StdId==BOILER_TEMP_CAN_ID){
//    input_packet_boiler_temperature = u;
//    for (int i=0; i<4 ;++i) {
//      ((uint8_t*)&input_packet_boiler_temperature)[i] = RxData[i];
//    }
//    input_packet_boiler_timeout = 0;
//  }
}

uint16_t pump_status_send_cnt = 0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

  if (htim->Instance == TIM4)
  {
    temperature_error_timeout_cnt++;
    if (temperature_error_timeout_cnt >= TEMPERATURE_ERROR_TIMEOUT_CNT_MAX) {
      temperature_error_timeout_cnt = TEMPERATURE_ERROR_TIMEOUT_CNT_MAX;

    }

    pump_status_send_cnt++;
    if (pump_status_send_cnt >= 10*300) {
      pump_status_send_cnt = 0;
      
      TxHeader.StdId = PUMP_STATUS_CAN_ID;
      TxHeader.DLC = 1;
      /* set pump status */
      TxData[0] = global_pump_flag;

      HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);


      // HAL_GPIO_TogglePin(TRIAC_2_GPIO_Port, TRIAC_2_Pin);
    }
  }

  // if (htim->Instance == TIM1)
  // {
  //   //  HAL_GPIO_TogglePin(GPIOC, LED_Pin);
  //   ++temperature1timeout;
  //   ++temperature2timeout;
  //   ++humidity1timeout;
  //   ++humidity2timeout;
  //   ++input_packet_boiler_timeout;

  //   if (temperature1timeout >= DATA_WAIT_TIMEOUT)
  //   {
  //     temperature1timeout = DATA_WAIT_TIMEOUT;
  //   }

  //   if (temperature2timeout >= DATA_WAIT_TIMEOUT)
  //   {
  //     temperature2timeout = DATA_WAIT_TIMEOUT;
  //   }

  //   if (humidity1timeout >= DATA_WAIT_TIMEOUT)
  //   {
  //     humidity1timeout = DATA_WAIT_TIMEOUT;
  //   }

  //   if (humidity2timeout >= DATA_WAIT_TIMEOUT)
  //   {
  //     humidity2timeout = DATA_WAIT_TIMEOUT;
  //   }

  //   if (input_packet_boiler_timeout >= DATA_WAIT_TIMEOUT)
  //   {
  //     input_packet_boiler_timeout = DATA_WAIT_TIMEOUT;
  //   }
  // }
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
