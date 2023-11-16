/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "LoRa.h"
#include "string.h"
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
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */
LoRa my_lora;
uint8_t lora_state = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t read_data[20];
uint8_t rx_count;
uint8_t *send_data;
uint8_t count =0;
int16_t vel, pos;
uint8_t *rx_buff, rx_index = 0;

bool rx = false;
int			RSSI;


////Handle recieve data;
void handle_rx_data(uint8_t* receive_data, uint8_t count);

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(GPIO_Pin);
  /* NOTE: This function Should not be modified, when the callback is needed,
           the HAL_GPIO_EXTI_Callback could be implemented in the user file
   */
//  if(GPIO_Pin == DIO0_Pin)
//  {
//	  /// Check RxDone
////	  if((LoRa_read(&my_lora, RegIrqFlags)&0x40)!= 0){
////
////
////
////	  }
//	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//  }

  if(GPIO_Pin == DIO_Pin)
    {
  	  /// Check RxDone
  //	  if((LoRa_read(&my_lora, RegIrqFlags)&0x40)!= 0){
  //
  //
  //
  //	  }
  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
    }

}

void handle_rx_data(uint8_t* receive_data, uint8_t count){

	if(!count)
		return;

	rx_buff = (uint8_t*)malloc(sizeof(uint8_t));
	for(int i =0 ; i< count; i++){

		switch(receive_data[i]){

		case (uint8_t)'r':
			////
			break;

		case (uint8_t)'s':
			////
			break;

		case (uint8_t)'v':
			////
			vel = atoi((const char*)rx_buff);
			//memset(rx_buff,0,sizeof(rx_buff));
			rx_index = 0;
			//free(receive_data);
			break;

		case (uint8_t)'p':
			pos = atoi((const char*)rx_buff);
			//memset(rx_buff,0,sizeof(rx_buff));
			rx_index =0;

			break;

		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			rx_buff = (uint8_t*)realloc(rx_buff,(rx_index+2)*sizeof(uint8_t));
			rx_buff[rx_index++] |= receive_data[i];
			break;

		}

	}
	free(rx_buff);



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
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  my_lora = newLoRa();
  my_lora.CS_port		= NSS_GPIO_Port	;		my_lora.reset_port	= RST_GPIO_Port	;
  my_lora.CS_pin		= NSS_Pin		;		my_lora.reset_pin	= RST_Pin		;

  my_lora.DIO0_port	= DIO_GPIO_Port;
  my_lora.reset_pin	= DIO_Pin		;		my_lora.hSPIx		= &hspi1			;

  my_lora.frequency             = 434;             // default = 433 MHz
  my_lora.spredingFactor        = SF_9;            // default = SF_7
  my_lora.bandWidth             = BW_250KHz;       // default = BW_125KHz
  my_lora.crcRate               = CR_4_8;          // default = CR_4_5
  my_lora.power                 = POWER_17db;      // default = 20db
  my_lora.overCurrentProtection = 120;             // default = 100 mA
  my_lora.preamble              = 10;              // default = 8;

  //LoRa_reset(&my_lora);
  if(LoRa_init(&my_lora)==LORA_OK)
	  {
	  	  lora_state = 1;
	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	  	  HAL_Delay(500);
	  };
  if(LoRa_init(&my_lora)==LORA_NOT_FOUND)
  	  {
  	  	  lora_state = 2;
//  	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//  	  	  HAL_Delay(500);
  	  };
  if(LoRa_init(&my_lora)==LORA_UNAVAILABLE)
  	  {
  	  	  lora_state = 3;
//  	  	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
//  	  	  HAL_Delay(500);
  	  };
  ////
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

  LoRa_startReceiving(&my_lora);
  //send_data[0] = 0X30;   /// slave address

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  ///-------------Transmit---------------///
//	  send_data = (uint8_t *)malloc(9*sizeof(uint8_t));
//	  if(send_data == NULL)
//		  return 0;
//	  //send_data = (uint8_t*)malloc(10*sizeof(uint8_t));
//	  send_data[0] = 0x30;
//	  for(count =1;count<4;count++){
//
//	  	send_data[count] =  48+count;
//	  }
//	  //send_data[1] = 100;
//	  //send_data[2] = (uint8_t)('v');
//	  //send_data[3] = 100;
//	  send_data[4] = 'v' ;
//
//	  for(count =5;count<8;count++){
//
//	  	  	send_data[count] =  48+count;
//	  	  }
//	  send_data[8] = 'p';
//	  if(LoRa_transmit(&my_lora, send_data, 9, 500))
//	  {
////		  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
////	  	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
//	  }
//
//	  free(send_data);

	  ///------------Recieve----------------///
	  //read_data =(uint8_t*)malloc(2*sizeof(uint8_t));
	  rx_count = LoRa_receive(&my_lora, read_data, 128);

	  handle_rx_data(read_data, rx_count);


	  //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_12);
	  //// free read_data after handling receive data ///
	  //free(read_data);

	  HAL_Delay(1000);

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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RST_Pin|NSS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO0_Pin */
  GPIO_InitStruct.Pin = DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(DIO0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIO_Pin */
  GPIO_InitStruct.Pin = DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RST_Pin NSS_Pin PB12 */
  GPIO_InitStruct.Pin = RST_Pin|NSS_Pin|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
