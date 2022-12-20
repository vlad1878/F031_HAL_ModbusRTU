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
#include <stdbool.h>
#include <lcd1602_i2c_lib.h>
#include "ModbusRTU_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UART_DMA_RX_BUFFER_SIZE 64
#define RE72_ADRESS 0x88
#define RE72_BYTE_ORDER 0x01
#define RE72_SP 0xFF4
#define RE72_PV 0x1B58
#define RE72_FUNCTION_READ_REGISTER 0x03
#define RE72_FUNCTION_WRITE_REGISTER 0x06
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
extern char tx_buffer_lcd[40];
extern uint16_t ADC_SMA_Data[3];
extern uint16_t ADC_RAW_Data[3];
extern bool adc_flag;

extern uint8_t ModbusRTU_tx_buffer[ModbusRTU_TX_BUFFER_SIZE];
extern uint8_t ModbusRTU_rx_buffer[ModbusRTU_RX_BUFFER_SIZE];
volatile bool ModbusRTU_rx_flag = 0;
unsigned long t_ModbusRTU_tx = 0;

uint8_t uart_dma_rx_buffer[UART_DMA_RX_BUFFER_SIZE] = {0, };

volatile bool display_flag = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void Read_Register_RE72(uint16_t Register, uint8_t Function_code);
void Read_Two_Registers_RE72(uint16_t Register, uint8_t Function_code);
void Write_Register_RE72(uint16_t Register, uint8_t Function_code, uint16_t Data_To_Write);
void Display_Data_ModbusRTU(void);
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
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  t_ModbusRTU_tx = HAL_GetTick();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
  lcd1602_Init();
  lcd1602_SetCursor(0, 0);
  for(int i = 0; i < 20; i++){
	  static uint8_t val = 0;
	  switch(val){
	  case 0:
			sprintf(tx_buffer_lcd, "Initialization.   ");
			lcd1602_Print_text(tx_buffer_lcd);
			HAL_Delay(300);
			val++;
			break;
	  case 1:
			sprintf(tx_buffer_lcd, "Initialization..   ");
			lcd1602_Print_text(tx_buffer_lcd);
			HAL_Delay(300);
			val++;
			break;
	  case 2:
		  	sprintf(tx_buffer_lcd, "Initialization...   ");
		    lcd1602_Print_text(tx_buffer_lcd);
		    HAL_Delay(300);
		    val++;
		    break;
	  case 3:
		  	sprintf(tx_buffer_lcd, "Initialization.   ");
		    lcd1602_Print_text(tx_buffer_lcd);
		    HAL_Delay(300);
		    val++;
		    break;
	  }
	  if(val == 3){
		  val = 0;
	  }
  }


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  Display_Data_ModbusRTU();
	  if((HAL_GetTick() - t_ModbusRTU_tx) >= 5000){
		  t_ModbusRTU_tx = HAL_GetTick();
		  Read_Register_RE72(RE72_SP, RE72_FUNCTION_READ_REGISTER);
		  //Read_TWO_Registers_RE72(RE72_PV, RE72_FUNCTION_READ_REGISTER);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(User_Led_GPIO_Port, User_Led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : User_Led_Pin */
  GPIO_InitStruct.Pin = User_Led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(User_Led_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
	if(huart->Instance == USART1){
		memcpy(ModbusRTU_rx_buffer, uart_dma_rx_buffer, Size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart1, uart_dma_rx_buffer, UART_DMA_RX_BUFFER_SIZE);
		display_flag = 1;
	}
}

void Read_Register_RE72(uint16_t Register, uint8_t Function_code){
	if(Function_code == 0x03){
		ModbusRTU_Read_Holding_Registers_0x03(RE72_ADRESS, Register, 1, RE72_BYTE_ORDER);
		HAL_UART_Transmit(&huart1, ModbusRTU_tx_buffer, 8, 1000);
	}
}

void Read_TWO_Registers_RE72(uint16_t Register, uint8_t Function_code){
	if(Function_code == 0x03){
		ModbusRTU_Read_Holding_Registers_0x03(RE72_ADRESS, Register, 2, RE72_BYTE_ORDER);
		HAL_UART_Transmit(&huart1, ModbusRTU_tx_buffer, 8, 1000);
	}
}

void Write_Register_RE72(uint16_t Register, uint8_t Function_code, uint16_t Data_To_Write){
	if(Function_code == 0x06){
		ModbusRTU_Write_Single_Register_0x06(RE72_ADRESS, Register, Data_To_Write, RE72_BYTE_ORDER);
		HAL_UART_Transmit(&huart1, ModbusRTU_tx_buffer, 8, 1000);
	}
}


void Display_Data_ModbusRTU(void){
	if(display_flag){
		lcd1602_SetCursor(0, 0);
		sprintf(tx_buffer_lcd, "ModbusRTU rx:  ");
		lcd1602_Print_text(tx_buffer_lcd);
		lcd1602_SetCursor(0, 1);
		sprintf(tx_buffer_lcd, "0x%04x%04x   ", ModbusRTU_rx_buffer[3], ModbusRTU_rx_buffer[4]);
		display_flag = 0;
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
