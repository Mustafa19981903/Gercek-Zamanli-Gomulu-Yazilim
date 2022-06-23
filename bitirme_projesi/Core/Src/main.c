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
#include "cmsis_os.h"
#include "lwip.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"tcp_echoserver.h"
#include"udp_echoserver.h"
#include"dwt_stm32_delay.h"
#include "BMP180.h"
#include "ssd1306.h"
#include "stdio.h"
#include"string.h"
#include"mpu6050.h"
#include "i2c-lcd.h"
#include "httpd.h"

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
CAN_HandleTypeDef hcan1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId bmp180TaskHandle;
osThreadId hcsr04taskHandle;
osThreadId hcsr501taskHandle;
osThreadId dht11taskHandle;
osThreadId dcmotor1taskHandle;
osThreadId dcmotor2taskHandle;
osThreadId nextionsenddataHandle;
osThreadId mpu6050taskHandle;
osThreadId usbttl_tx_rxHandle;
osThreadId canbussendatataHandle;
osThreadId ethernetHandle;
uint32_t sensor_time;
float distance;
uint16_t pulse;
uint32_t mTxMailbox;
CAN_TxHeaderTypeDef pTxHeader;
CAN_FilterTypeDef sFilterConfig;
uint8_t cmdend[3]= {0xFF ,0xFF ,0xFF};
MPU6050_t MPU6050;
uint8_t rx_buffer[50], tx_buffer[50];
/* USER CODE BEGIN PV */

//const char* LedCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
//const tCGI LedCGI = { "/leds.cgi", LedCGIhandler };
//tCGI theCGItable[1];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
void bmp180_handlerTask(void const * argument);
void hcsr04_handlertask(void const * argument);
void hcsr501handler_task(void const * argument);
void dht11_handlertask(void const * argument);
void dcmotor1_handlertask(void const * argument);
void dcmotor2_handlertask(void const * argument);
void nextionsenddata_handlertask(void const * argument);
void mpu6050_handlertask(void const * argument);
void usbt_tx_rx_handlertask(void const * argument);
void canbussendata_handlertask(void const * argument);
void ethernet_handler(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float sicaklik = 0;
float basinc = 0;
float yukseklik= 0;

char sicaklik1[10];
char basinc1[10];
char yukseklik1[10];
uint8_t durum=0,Humidity=0,Temperature=0;
uint16_t tempVal=0,humVal=0;
uint8_t dhtVal[2];
uint8_t mData[40];
uint16_t mTime1 = 0, mTime2 = 0;
uint16_t mbit = 0;
uint8_t  parityVal = 0, genParity = 0;
uint8_t mymessage[50];

 CAN_TxHeaderTypeDef pTxHeader;
 uint32_t pTxMailbox;
CAN_RxHeaderTypeDef pRxHeader;
 uint8_t count;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


	const char* LedCGIhandler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
	{
	    uint32_t i = 0;

	    if (iIndex == 0) {

	        //turning the LED lights off
	        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

	    }

	    for (i = 0; i < iNumParams; i++) {

	        if (strcmp(pcParam[i], "led") == 0)

	        {

	            if (strcmp(pcValue[i], "1") == 0) {
	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
	            }

	            else if (strcmp(pcValue[i], "2") == 0) {

	                HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);

	            }

	        }

	    }
	    return "/index2.html";
	}





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
  MX_I2C1_Init();
  MX_I2C3_Init();
  MX_USART3_UART_Init();
  MX_CAN1_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_CAN_Start(&hcan1);
  HAL_TIM_Base_Start(&htim6);
  //theCGItable[0] = LedCGI;
 // http_set_cgi_handlers(theCGItable, 1);
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
  /* definition and creation of bmp180Task */
  osThreadDef(bmp180Task, bmp180_handlerTask, osPriorityNormal, 0, 128);
  bmp180TaskHandle = osThreadCreate(osThread(bmp180Task), NULL);

  /* definition and creation of hcsr04task */
  osThreadDef(hcsr04task, hcsr04_handlertask, osPriorityNormal, 0, 128);
  hcsr04taskHandle = osThreadCreate(osThread(hcsr04task), NULL);

  /* definition and creation of hcsr501task */
  osThreadDef(hcsr501task, hcsr501handler_task, osPriorityIdle, 0, 128);
  hcsr501taskHandle = osThreadCreate(osThread(hcsr501task), NULL);

  /* definition and creation of dht11task */
  osThreadDef(dht11task, dht11_handlertask, osPriorityIdle, 0, 128);
  dht11taskHandle = osThreadCreate(osThread(dht11task), NULL);

  /* definition and creation of dcmotor1task */
  osThreadDef(dcmotor1task, dcmotor1_handlertask, osPriorityIdle, 0, 128);
  dcmotor1taskHandle = osThreadCreate(osThread(dcmotor1task), NULL);

  /* definition and creation of dcmotor2task */
  osThreadDef(dcmotor2task, dcmotor2_handlertask, osPriorityIdle, 0, 128);
  dcmotor2taskHandle = osThreadCreate(osThread(dcmotor2task), NULL);

  /* definition and creation of nextionsenddata */
  osThreadDef(nextionsenddata, nextionsenddata_handlertask, osPriorityIdle, 0, 128);
  nextionsenddataHandle = osThreadCreate(osThread(nextionsenddata), NULL);

  /* definition and creation of mpu6050task */
  osThreadDef(mpu6050task, mpu6050_handlertask, osPriorityIdle, 0, 128);
  mpu6050taskHandle = osThreadCreate(osThread(mpu6050task), NULL);

  /* definition and creation of usbttl_tx_rx */
  osThreadDef(usbttl_tx_rx, usbt_tx_rx_handlertask, osPriorityIdle, 0, 128);
  usbttl_tx_rxHandle = osThreadCreate(osThread(usbttl_tx_rx), NULL);

  /* definition and creation of canbussendatata */
  osThreadDef(canbussendatata, canbussendata_handlertask, osPriorityIdle, 0, 128);
  canbussendatataHandle = osThreadCreate(osThread(canbussendatata), NULL);

  /* definition and creation of ethernet */
  osThreadDef(ethernet, ethernet_handler, osPriorityIdle, 0, 128);
  ethernetHandle = osThreadCreate(osThread(ethernet), NULL);

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
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
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
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_5TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 100-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 0xffff-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PE6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA14 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD12 PD13 PD14 PD15
                           PD4 PD5 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void ileri_motor1(int s)
{




	  TIM3->CCR1=s;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

}

void geri_motor1(int s)
{

	  TIM3->CCR1=s;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_SET);



}

void dur_motor1()
{

	  TIM3->CCR1=0;
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);

}

void ileri_motor2(int a)
{




	  TIM3->CCR2=a;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

}

void geri_motor2(int a)
{

	  TIM3->CCR2=a;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);



}

void dur_motor2()
{

	  TIM3->CCR2=0;
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_bmp180_handlerTask */
/**
  * @brief  Function implementing the bmp180Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_bmp180_handlerTask */
void bmp180_handlerTask(void const * argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
	BMP180_Start();

	  SSD1306_Init();

	  SSD1306_GotoXY (35,0);
	  SSD1306_Puts ("BMP180", &Font_11x18, 1);
	  SSD1306_GotoXY (10,20);
	  SSD1306_Puts ("Barometric", &Font_11x18, 1);
	  SSD1306_GotoXY (30,40);
	  SSD1306_Puts ("Sensor", &Font_11x18, 1);
	  SSD1306_GotoXY (20,40);
	  SSD1306_UpdateScreen(); //display
	  HAL_Delay(2000);
	  SSD1306_Clear();
  for(;;)
  {
	  float sicaklik = 0;
	  float basinc = 0;
	  float yukseklik= 0;

	  	  sicaklik = BMP180_GetTemp();

	  	  basinc = BMP180_GetPress (0);

	  	  yukseklik = BMP180_GetAlt(0);


	     SSD1306_GotoXY (35,0);
	     SSD1306_Puts ("BMP180", &Font_11x18, 1);
	     SSD1306_GotoXY (0,0);
	     SSD1306_Puts ("sicaklik", &Font_11x18, 1);
	     SSD1306_GotoXY (20,40);
	     sprintf(sicaklik1, "%.2f", sicaklik);
	     SSD1306_Puts(sicaklik1, &Font_11x18, 1);
	     SSD1306_DrawCircle(80, 40, 2, 1);  //To print degree only
	     SSD1306_GotoXY (85,40);  //To print celcius
	     SSD1306_Puts ("C", &Font_11x18, 1);
	     SSD1306_UpdateScreen(); //display
	     HAL_Delay(2000);
	     SSD1306_Clear();


	     SSD1306_GotoXY (20,0);
	     SSD1306_Puts ("basinc", &Font_11x18, 1);
	     SSD1306_GotoXY (10,40);
	     sprintf(basinc1, "%.2f", basinc);
	     SSD1306_Puts(basinc1, &Font_11x18, 1);
	     SSD1306_GotoXY (100,40);
	     SSD1306_Puts ("pa", &Font_11x18, 1);
	     SSD1306_UpdateScreen(); //display
	     HAL_Delay(2000);
	     SSD1306_Clear();

	     SSD1306_GotoXY (20,0);
	     SSD1306_Puts ("yukseklik", &Font_11x18, 1);
	     SSD1306_GotoXY (15,40);
	     sprintf(yukseklik1, "%.2f", yukseklik);
	     SSD1306_Puts(yukseklik1, &Font_11x18, 1);
	     SSD1306_GotoXY (90,40);
	     SSD1306_Puts ("m", &Font_11x18, 1);
	     SSD1306_UpdateScreen(); //display
	     HAL_Delay(2000);
	     SSD1306_Clear();

	     HAL_Delay (2000);
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_hcsr04_handlertask */
/**
* @brief Function implementing the hcsr04task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hcsr04_handlertask */
void hcsr04_handlertask(void const * argument)
{
  /* USER CODE BEGIN hcsr04_handlertask */


  /* Infinite loop */
	{
		uint32_t local_time=0;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_SET);
		DWT_Delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_14, GPIO_PIN_RESET);


		while(!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15));


		while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))
		{
			local_time++;
			DWT_Delay_us(1);
		}
		return local_time;



	}




  for(;;)
  {
	  sensor_time=Read_HCSR04();
	  distance=sensor_time/58;

  	  if(distance>6 && distance<12)



    osDelay(1);
  }
  /* USER CODE END hcsr04_handlertask */
}

/* USER CODE BEGIN Header_hcsr501handler_task */
/**
* @brief Function implementing the hcsr501task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_hcsr501handler_task */
void hcsr501handler_task(void const * argument)
{
  /* USER CODE BEGIN hcsr501handler_task */
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6==GPIO_PIN_SET))
	  {
   		  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"1. ve 2. motorlar durduruldu. "), 100);
   		  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

   		  dur_motor1();
   		  dur_motor2();
	  }
    osDelay(1);
  }
  /* USER CODE END hcsr501handler_task */
}

/* USER CODE BEGIN Header_dht11_handlertask */
/**
* @brief Function implementing the dht11task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dht11_handlertask */
void dht11_handlertask(void const * argument)
{
  /* USER CODE BEGIN dht11_handlertask */
  /* Infinite loop */
	void delay(uint16_t time){

	   __HAL_TIM_SET_COUNTER(&htim6,0);

	   while(__HAL_TIM_GET_COUNTER(&htim6)< time);

	}


	void Set_Pin_Output(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	   GPIO_InitTypeDef DHT11_DATA={0};

	   DHT11_DATA.Pin=GPIO_Pin;
	   DHT11_DATA.Mode=GPIO_MODE_OUTPUT_PP;
	   DHT11_DATA.Pull=GPIO_NOPULL;
	   DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	   HAL_GPIO_Init(GPIOx,&DHT11_DATA);

	}

	void Set_Pin_Input(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin){


	   GPIO_InitTypeDef DHT11_DATA={0};

	   DHT11_DATA.Pin=GPIO_Pin;
	   DHT11_DATA.Mode=GPIO_MODE_INPUT;
	   DHT11_DATA.Pull=GPIO_NOPULL;
	   DHT11_DATA.Speed=GPIO_SPEED_FREQ_LOW;

	   HAL_GPIO_Init(GPIOx,&DHT11_DATA);

	}

	#define DHT11_PORT GPIOE
	#define DHT11_PIN GPIO_PIN_6





	uint8_t DHT11_Read (void){

	  for(int a=0;a<40;a++) mData[a]=0;
	   mTime1 = 0, mTime2 = 0, durum=0, tempVal=0, humVal=0, parityVal = 0, genParity = 0,  mbit = 0;

	     Set_Pin_Output(DHT11_PORT,DHT11_PIN);
	    HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,GPIO_PIN_RESET);
	    delay(18000);
	      Set_Pin_Input(DHT11_PORT,DHT11_PIN);

	    __HAL_TIM_SET_COUNTER(&htim6, 0);
	   while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;

	   __HAL_TIM_SET_COUNTER(&htim6, 0);
	   while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	   mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);

	   __HAL_TIM_SET_COUNTER(&htim6, 0);
	   while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	    mTime2 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);


	   if(mTime1 < 75 && mTime1 > 85 && mTime2 < 75 && mTime2 > 85)
	   {

	      return 0;
	   }




	   for(int j = 0; j < 40; j++)
	   {
	      __HAL_TIM_SET_COUNTER(&htim6, 0);
	      while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_RESET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	      __HAL_TIM_SET_COUNTER(&htim6, 0);
	      while(HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN) == GPIO_PIN_SET) if((uint16_t)__HAL_TIM_GET_COUNTER(&htim6) > 500) return 0;
	      mTime1 = (uint16_t)__HAL_TIM_GET_COUNTER(&htim6);


	      if(mTime1 > 20 && mTime1 < 30)
	      {
	         mbit = 0;
	      }
	      else if(mTime1 > 60 && mTime1 < 80)
	      {
	          mbit = 1;
	      }


	      mData[j] = mbit;

	   }


	   for(int i = 0; i < 8; i++)
	   {
	      humVal += mData[i];
	      humVal = humVal << 1;
	   }


	   for(int i = 16; i < 24; i++)
	   {
	      tempVal += mData[i];
	      tempVal = tempVal << 1;
	   }


	   for(int i = 32; i < 40; i++)
	   {
	      parityVal += mData[i];
	      parityVal = parityVal << 1;
	   }

	   parityVal = parityVal >> 1;
	   humVal = humVal >> 1;
	   tempVal = tempVal >> 1;

	   genParity = humVal + tempVal;


	   dhtVal[0]= tempVal;
	   dhtVal[1] = humVal;

	   return 1;
	}
  for(;;)
  {
	durum=DHT11_Read();
	if (durum ==1)   {Temperature=tempVal; Humidity=humVal;}
	HAL_Delay(1000);
    osDelay(1);
  }
  /* USER CODE END dht11_handlertask */
}

/* USER CODE BEGIN Header_dcmotor1_handlertask */
/**
* @brief Function implementing the dcmotor1task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dcmotor1_handlertask */
void dcmotor1_handlertask(void const * argument)
{
  /* USER CODE BEGIN dcmotor1_handlertask */
  /* Infinite loop */



  for(;;)
  {
	  ileri_motor1(85);
	  osDelay(1000);
	  geri_motor1(85);
	  osDelay(1000);


  }
  /* USER CODE END dcmotor1_handlertask */
}

/* USER CODE BEGIN Header_dcmotor2_handlertask */
/**
* @brief Function implementing the dcmotor2task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_dcmotor2_handlertask */
void dcmotor2_handlertask(void const * argument)
{
  /* USER CODE BEGIN dcmotor2_handlertask */
  /* Infinite loop */
  for(;;)
  {
	  ileri_motor2(85);
	  osDelay(1000);
	  geri_motor2(85);
	  osDelay(1000);
  }
  /* USER CODE END dcmotor2_handlertask */
}

/* USER CODE BEGIN Header_nextionsenddata_handlertask */
/**
* @brief Function implementing the nextionsenddata thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_nextionsenddata_handlertask */
void nextionsenddata_handlertask(void const * argument)
{
  /* USER CODE BEGIN nextionsenddata_handlertask */
  /* Infinite loop */
	void NEXTION_SendFloat (char *obj, float num, int dp)
	{


	   uint8_t *buffer = malloc(30*sizeof (char));
	   int len = sprintf ((char *)mymessage, "%s.vvs1=%d", obj, dp);
	   HAL_UART_Transmit(&huart1, buffer, len, 1000);
	   HAL_UART_Transmit(&huart1, cmdend, 3, 100);

	   int32_t number = num*(pow(10,dp));
	   len = sprintf ((char *)mymessage, "%s.val=%ld", obj, number);
	   HAL_UART_Transmit(&huart1, mymessage, len, 1000);
	   HAL_UART_Transmit(&huart1, cmdend, 3, 100);
	   free(buffer);
	}
  for(;;)
  {

	  sprintf((char*)mymessage,"n0.val=%d",distance);
	  	  HAL_UART_Transmit(&huart1, mymessage, strlen(mymessage), 100);
	  	  HAL_UART_Transmit(&huart1, cmdend, 3, 100);
	  	  NEXTION_SendFloat("x0", sicaklik, 3);


    osDelay(1);
  }
  /* USER CODE END nextionsenddata_handlertask */
}

/* USER CODE BEGIN Header_mpu6050_handlertask */
/**
* @brief Function implementing the mpu6050task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_mpu6050_handlertask */
void mpu6050_handlertask(void const * argument)
{
  /* USER CODE BEGIN mpu6050_handlertask */
  /* Infinite loop */
	  while (MPU6050_Init(&hi2c1) == 1);
  for(;;)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  HAL_Delay (100);
    osDelay(1);
  }
  /* USER CODE END mpu6050_handlertask */
}

/* USER CODE BEGIN Header_usbt_tx_rx_handlertask */
/**
* @brief Function implementing the usbttl_tx_rX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usbt_tx_rx_handlertask */
void usbt_tx_rx_handlertask(void const * argument)
{
  /* USER CODE BEGIN usbt_tx_rx_handlertask */
  /* Infinite loop */
  for(;;)
  {HAL_UART_Receive(&huart3, (uint8_t*)rx_buffer, 50, 100);


  if(rx_buffer[0] =='m' &&  rx_buffer[1] =='o' &&   rx_buffer[2] =='t' &&   rx_buffer[3] =='o' &&
		  rx_buffer[4] =='r' && rx_buffer[5] =='1' &&  rx_buffer[6] ==' ' &&   rx_buffer[7] =='i' &&
		  rx_buffer[8] =='l' &&   rx_buffer[9] =='e' &&   rx_buffer[10] =='r' &&   rx_buffer[11] =='i' )
  {

	  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"1. motor ileri calisti. "), 100);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	  ileri_motor1(85);


  }

  if(rx_buffer[0] =='m' &&  rx_buffer[1] =='o' &&   rx_buffer[2] =='t' &&   rx_buffer[3] =='o' &&
		  rx_buffer[4] =='r' && rx_buffer[5] =='1' &&  rx_buffer[6] ==' ' &&   rx_buffer[7] =='g' &&
		  rx_buffer[8] =='e' &&   rx_buffer[9] =='r' &&   rx_buffer[10] =='i' )
  {

	  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"1. motor geri calisti. "), 100);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);


	  geri_motor1(85);




  }


  if(rx_buffer[0] =='m' &&  rx_buffer[1] =='o' &&   rx_buffer[2] =='t' &&   rx_buffer[3] =='o' &&
		  rx_buffer[4] =='r' && rx_buffer[5] =='2' &&  rx_buffer[6] ==' ' &&   rx_buffer[7] =='i' &&
		  rx_buffer[8] =='l' &&   rx_buffer[9] =='e' &&   rx_buffer[10] =='r' &&   rx_buffer[11] =='i' )
  {

	  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"2. motor ileri calisti. "), 100);
	  ileri_motor2(85);
	  HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);


  }

  if(rx_buffer[0] =='m' &&  rx_buffer[1] =='o' &&   rx_buffer[2] =='t' &&   rx_buffer[3] =='o' &&
		  rx_buffer[4] =='r' && rx_buffer[5] =='2' &&  rx_buffer[6] ==' ' &&   rx_buffer[7] =='g' &&
		  rx_buffer[8] =='e' &&   rx_buffer[9] =='r' &&   rx_buffer[10] =='i' )
  {

	  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"2. motor geri calisti. "), 100);
	  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	  geri_motor2(85);


  }


  if(HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_6)==GPIO_PIN_SET)
  {


	  HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, sprintf(tx_buffer,"1. ve 2. motorlar durduruldu. "), 100);


	  dur_motor1();
	  dur_motor2();

  }

    osDelay(1);
  }
  /* USER CODE END usbt_tx_rx_handlertask */
}

/* USER CODE BEGIN Header_canbussendata_handlertask */
/**
* @brief Function implementing the canbussendatata thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_canbussendata_handlertask */
void canbussendata_handlertask(void const * argument)
{
  /* USER CODE BEGIN canbussendata_handlertask */
  /* Infinite loop */
	  HAL_CAN_Start(&hcan1);
	     // Enable interrupts
	  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	    //Set Transmit Parameters
	   pTxHeader.DLC = 1; // GÖNDERECEĞİMİZ MESAJ KAÇ BYTE
	   pTxHeader.IDE = CAN_ID_EXT;
	   pTxHeader.RTR = CAN_RTR_DATA;
	   pTxHeader.ExtId = 0x1FBF9000;

	   // Set Filter Parameters
	   sFilterConfig.FilterActivation = ENABLE;
	   sFilterConfig.FilterBank =0;
	   sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	   sFilterConfig.FilterIdHigh=0x1FBF9000 >> 13; // 0x1FBF9000 0x1FBF9FFF
	   sFilterConfig.FilterIdLow=0x0000;
	   sFilterConfig.FilterMaskIdHigh=0xFFFFF000 >> 13;
	   sFilterConfig.FilterMaskIdLow=0x0000;
	   sFilterConfig.FilterMode=CAN_FILTERMODE_IDMASK;
	   sFilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	   HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

  for(;;)
  {
	  HAL_CAN_AddTxMessage(&hcan1, &pTxHeader, &count,&pTxMailbox);
    osDelay(1);
  }
  /* USER CODE END canbussendata_handlertask */
}

/* USER CODE BEGIN Header_ethernet_handler */
/**
* @brief Function implementing the ethernet thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ethernet_handler */
void ethernet_handler(void const * argument)
{
  /* USER CODE BEGIN ethernet_handler */
  /* Infinite loop */
  for(;;)
  {
	  //MX_LWIP_Process();

    osDelay(1);
  }
  /* USER CODE END ethernet_handler */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
