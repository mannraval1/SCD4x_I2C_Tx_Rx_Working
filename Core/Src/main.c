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
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#define SCD4x_I2C_ADDR 		(0x62 << 1)

#define CRC8_POLYNOMIAL 	0x31
#define CRC8_INIT 			0xFF


typedef struct measurements {

	uint8_t CO2;
	float Humidity;
	float Temperature;

	uint8_t CO2_crc;
	uint8_t Humidity_crc;
	uint8_t Temperature_crc;

} Measurements_TypeDef;

typedef struct settings {

	uint16_t temperatureOffset;
	uint8_t	temperatureOffset_crc;
	uint16_t Altitude;
	uint8_t Altitude_crc;
	bool AutoSelfCalib;
	uint8_t AutoSelfCalib_crc;

} Settings_TypeDef;

typedef struct sensor_id {

	uint64_t sensorID;
	uint8_t sensorID_Data_0_crc;
	uint8_t sensorID_Data_1_crc;
	uint8_t sensorID_Data_2_crc;

} SensorID_TypeDef;


/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/*--------------------------------------------------------------------------------------
----------------------------------SCD4x FUCNTIONS DECLARATIONS--------------------------
---------------------------------------------------------------------------------------*/

void SCD4x_Start_Periodic_Measurement(void);
void SCD4x_Read_Measurement(void);
void SCD4x_Stop_Periodic_Measurement(void);
void SCD4x_Set_Temperature_Offset(uint16_t offset);
void SCD4x_Get_Temperature_Offset(void);
void SCD4x_Set_Sensor_Altitude(uint16_t altitude); // One can set altitude to get higher CO2 accuracy, default altitude is 0.
void SCD4x_Get_Sensor_Altitude(void);
void SCD4x_Set_Ambient_Pressure(uint16_t ambientP);
void SCD4x_Set_Automatic_Self_Calibration(bool autoCalibMode);
void SCD4x_Get_Automatic_Self_Calibration(void);
void SCD4x_Start_Low_Power_Periodic_Measurement(void);
bool SCD4x_Get_Data_Ready_Status(void);
void SCD4x_Persist_Settings(void);
void SCD4x_Get_Serial_Number(void);
bool SCD4x_Perform_Self_Test(void);
void SCD4x_Perform_Factory_Reset(void);
void SCD4x_Reinit(void);
void SCD41x_Measure_Single_Shot(void); // SCD41 only.
void SCD41x_Measure_Single_Shot_RHT_Only(void); // SCD41 only. Use read_measurement command, CO2 is returned as 0 ppm.
void SCD4x_Error_Handler(void);
static uint8_t SCD4x_Generate_CRC(const uint8_t* data, uint16_t count);

/********************************************************************************************************************************
**************************************************Variable Declarations**********************************************************
********************************************************************************************************************************/

Measurements_TypeDef Measurements;
Settings_TypeDef Settings;
SensorID_TypeDef Sensor;

extern Measurements_TypeDef Measurements;
uint16_t co2;
float temp;
float hum;

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* Configure the system clock */
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  HAL_Delay(60000);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	 SCD4x_Start_Periodic_Measurement();
	  SCD4x_Read_Measurement();
	  co2 = Measurements.CO2;
 /* while (1)
  {
    SCD4x_Start_Periodic_Measurement();
	  SCD4x_Read_Measurement();
	  co2 = Measurements.CO2;
		HAL_UART_Transmit(&huart1,(uint8_t*)co2,sizeof(co2),500);
  }*/
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
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
  hi2c1.Init.Timing = 0x10909CEC;
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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, USB_OTG_FS_PWR_EN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPBTLE_RF_SPI3_CSN_GPIO_Port, SPBTLE_RF_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ISM43362_SPI3_CSN_GPIO_Port, ISM43362_SPI3_CSN_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : M24SR64_Y_RF_DISABLE_Pin M24SR64_Y_GPO_Pin ISM43362_RST_Pin ISM43362_SPI3_CSN_Pin */
  GPIO_InitStruct.Pin = M24SR64_Y_RF_DISABLE_Pin|M24SR64_Y_GPO_Pin|ISM43362_RST_Pin|ISM43362_SPI3_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_OVRCR_EXTI3_Pin SPSGRF_915_GPIO3_EXTI5_Pin SPBTLE_RF_IRQ_EXTI6_Pin ISM43362_DRDY_EXTI1_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_OVRCR_EXTI3_Pin|SPSGRF_915_GPIO3_EXTI5_Pin|SPBTLE_RF_IRQ_EXTI6_Pin|ISM43362_DRDY_EXTI1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : BUTTON_EXTI13_Pin */
  GPIO_InitStruct.Pin = BUTTON_EXTI13_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BUTTON_EXTI13_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_A5_Pin ARD_A4_Pin ARD_A3_Pin ARD_A2_Pin
                           ARD_A1_Pin ARD_A0_Pin */
  GPIO_InitStruct.Pin = ARD_A5_Pin|ARD_A4_Pin|ARD_A3_Pin|ARD_A2_Pin
                          |ARD_A1_Pin|ARD_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D1_Pin ARD_D0_Pin */
  GPIO_InitStruct.Pin = ARD_D1_Pin|ARD_D0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF8_UART4;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D10_Pin SPBTLE_RF_RST_Pin ARD_D9_Pin */
  GPIO_InitStruct.Pin = ARD_D10_Pin|SPBTLE_RF_RST_Pin|ARD_D9_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D4_Pin */
  GPIO_InitStruct.Pin = ARD_D4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
  HAL_GPIO_Init(ARD_D4_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D7_Pin */
  GPIO_InitStruct.Pin = ARD_D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D7_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D13_Pin ARD_D12_Pin ARD_D11_Pin */
  GPIO_InitStruct.Pin = ARD_D13_Pin|ARD_D12_Pin|ARD_D11_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D3_Pin */
  GPIO_InitStruct.Pin = ARD_D3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D3_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ARD_D6_Pin */
  GPIO_InitStruct.Pin = ARD_D6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ARD_D6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ARD_D8_Pin ISM43362_BOOT0_Pin ISM43362_WAKEUP_Pin LED2_Pin
                           SPSGRF_915_SDN_Pin ARD_D5_Pin */
  GPIO_InitStruct.Pin = ARD_D8_Pin|ISM43362_BOOT0_Pin|ISM43362_WAKEUP_Pin|LED2_Pin
                          |SPSGRF_915_SDN_Pin|ARD_D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DFSDM1_DATIN2_Pin DFSDM1_CKOUT_Pin */
  GPIO_InitStruct.Pin = DFSDM1_DATIN2_Pin|DFSDM1_CKOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_DFSDM1;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : QUADSPI_CLK_Pin QUADSPI_NCS_Pin OQUADSPI_BK1_IO0_Pin QUADSPI_BK1_IO1_Pin
                           QUAD_SPI_BK1_IO2_Pin QUAD_SPI_BK1_IO3_Pin */
  GPIO_InitStruct.Pin = QUADSPI_CLK_Pin|QUADSPI_NCS_Pin|OQUADSPI_BK1_IO0_Pin|QUADSPI_BK1_IO1_Pin
                          |QUAD_SPI_BK1_IO2_Pin|QUAD_SPI_BK1_IO3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_UART3_TX_Pin INTERNAL_UART3_RX_Pin */
  GPIO_InitStruct.Pin = INTERNAL_UART3_TX_Pin|INTERNAL_UART3_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : LPS22HB_INT_DRDY_EXTI0_Pin LSM6DSL_INT1_EXTI11_Pin ARD_D2_Pin HTS221_DRDY_EXTI15_Pin
                           PMOD_IRQ_EXTI12_Pin */
  GPIO_InitStruct.Pin = LPS22HB_INT_DRDY_EXTI0_Pin|LSM6DSL_INT1_EXTI11_Pin|ARD_D2_Pin|HTS221_DRDY_EXTI15_Pin
                          |PMOD_IRQ_EXTI12_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : USB_OTG_FS_PWR_EN_Pin SPBTLE_RF_SPI3_CSN_Pin PMOD_RESET_Pin STSAFE_A100_RESET_Pin */
  GPIO_InitStruct.Pin = USB_OTG_FS_PWR_EN_Pin|SPBTLE_RF_SPI3_CSN_Pin|PMOD_RESET_Pin|STSAFE_A100_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_XSHUT_Pin LED3_WIFI__LED4_BLE_Pin */
  GPIO_InitStruct.Pin = VL53L0X_XSHUT_Pin|LED3_WIFI__LED4_BLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : VL53L0X_GPIO1_EXTI7_Pin LSM3MDL_DRDY_EXTI8_Pin */
  GPIO_InitStruct.Pin = VL53L0X_GPIO1_EXTI7_Pin|LSM3MDL_DRDY_EXTI8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : INTERNAL_SPI3_SCK_Pin INTERNAL_SPI3_MISO_Pin INTERNAL_SPI3_MOSI_Pin */
  GPIO_InitStruct.Pin = INTERNAL_SPI3_SCK_Pin|INTERNAL_SPI3_MISO_Pin|INTERNAL_SPI3_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PMOD_SPI2_SCK_Pin */
  GPIO_InitStruct.Pin = PMOD_SPI2_SCK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PMOD_SPI2_SCK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PMOD_UART2_CTS_Pin PMOD_UART2_RTS_Pin PMOD_UART2_TX_Pin PMOD_UART2_RX_Pin */
  GPIO_InitStruct.Pin = PMOD_UART2_CTS_Pin|PMOD_UART2_RTS_Pin|PMOD_UART2_TX_Pin|PMOD_UART2_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

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

/*********************************************************************************************************************************************
																										Functions Declarations
*********************************************************************************************************************************************/

void SCD4x_Start_Periodic_Measurement(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0xB1;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(5000);
}

void SCD4x_Read_Measurement(void)
{
	bool isDataReady;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[9] = {0};

	isDataReady = SCD4x_Get_Data_Ready_Status();

	int i = 0;
	while(i < 10)
	{
		if(isDataReady == 0)
			isDataReady = SCD4x_Get_Data_Ready_Status();
		else
			break;
	}

	if(!(isDataReady))
	{
		SCD4x_Stop_Periodic_Measurement();
		SCD4x_Reinit();
		SCD4x_Start_Periodic_Measurement();
	}

	txBuff[0] = 0xEC;
	txBuff[1] = 0x05;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Measurements.CO2 = (rxBuff[0] << 8) | rxBuff[1];
	Measurements.CO2_crc = rxBuff[2];

	Measurements.Temperature = (float)(-45 + ((((rxBuff[3] << 8) | rxBuff[4]) * 175) / pow(2, 16)));
	Measurements.Temperature_crc = rxBuff[5];

	Measurements.Humidity = (float)(100 * ((rxBuff[6] << 8) | rxBuff[7]) / pow(2, 16));
	Measurements.Humidity_crc = rxBuff[8];

}

void SCD4x_Stop_Periodic_Measurement(void) // idle mode.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x3F;
	txBuff[1] = 0x86;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(500);
}

void SCD4x_Set_Temperature_Offset(uint16_t offset)
{
	uint8_t txBuff[5] = {0};
	uint8_t offsetBuff[2] = {0};

	offsetBuff[0] = offset >> 8;
	offsetBuff[1] = offset & 0xFF;

	txBuff[0] = 0x24;
	txBuff[1] = 0x1D;
	txBuff[2] = offset >> 8;
	txBuff[3] = offset & 0xFF;
	txBuff[4] = SCD4x_Generate_CRC(offsetBuff, sizeof(offsetBuff));

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Get_Temperature_Offset(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x18;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Settings.temperatureOffset = (rxBuff[0] << 8) | rxBuff[1];
	Settings.temperatureOffset_crc = rxBuff[3];

}

void SCD4x_Set_Sensor_Altitude(uint16_t altitude) // One can set altitude to get higher CO2 accuracy, default altitude is 0.
{
	uint8_t txBuff[5] = {0};
	uint8_t altitudeBuff[2] = {0};

	altitudeBuff[0] = altitude >> 8;
	altitudeBuff[1] = altitude & 0xFF;

	txBuff[0] = 0x24;
	txBuff[1] = 0x27;
	txBuff[2] = altitude & 0xFF;
	txBuff[3] = altitude >> 8;
	txBuff[4] = SCD4x_Generate_CRC(altitudeBuff, sizeof(altitudeBuff));

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Get_Sensor_Altitude(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x22;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Settings.Altitude = (rxBuff[0] << 8) | rxBuff[1];
	Settings.Altitude_crc = rxBuff[3];
}

void SCD4x_Set_Ambient_Pressure(uint16_t ambientP)
{
	uint8_t txBuff[5] = {0};
	uint8_t ambientPBuff[2] = {0};

	ambientPBuff[0] = ambientP >> 8;
	ambientPBuff[1] = ambientP & 0xFF;

	txBuff[0] = 0x0E;
	txBuff[1] = 0x00;
	txBuff[2] = ambientP & 0xFF;
	txBuff[3] = ambientP >> 8;
	txBuff[4] = SCD4x_Generate_CRC(ambientPBuff, sizeof(ambientPBuff));

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

void SCD4x_Set_Automatic_Self_Calibration(bool autoCalibMode)
{
	uint8_t txBuff[5] = {0};
	uint8_t autoCalibModeBuff[2] = {0};

	if(!(autoCalibMode))
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x00;
	}
	else if(autoCalibMode)
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x01;
	}
	else
	{
		autoCalibModeBuff[0] = 0x00;
		autoCalibModeBuff[1] = 0x01;
	}

	txBuff[0] = 0x24;
	txBuff[1] = 0x16;
	txBuff[2] = autoCalibMode & 0xFF;
	txBuff[3] = autoCalibMode >> 8;
	txBuff[4] = SCD4x_Generate_CRC(autoCalibModeBuff, sizeof(autoCalibModeBuff));

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);

}

void SCD4x_Get_Automatic_Self_Calibration(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x23;
	txBuff[1] = 0x13;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if(((rxBuff[0] << 8) | rxBuff[1]) == 0 && rxBuff[3] == 0x81)
		Settings.AutoSelfCalib = false;
	else if(((rxBuff[0] << 8) | rxBuff[1]) == 1 && rxBuff[3] == 0xB0)
		Settings.AutoSelfCalib = true;
	else
		SCD4x_Error_Handler();

	Settings.AutoSelfCalib_crc = rxBuff[3];
}

void SCD4x_Start_Low_Power_Periodic_Measurement(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0xAC;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1);
}

bool SCD4x_Get_Data_Ready_Status(void)
{
	bool dataReadyFlag;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0xE4;
	txBuff[1] = 0xB8;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if((((rxBuff[0] << 8) | rxBuff[1]) & 0x07FF) != 0)
		dataReadyFlag = true;
	else
		dataReadyFlag = false;

	return dataReadyFlag;
}

void SCD4x_Persist_Settings(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x15;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);

	HAL_Delay(1000);
}

void SCD4x_Get_Serial_Number(void)
{
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[9] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x82;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	Sensor.sensorID = (rxBuff[0] << 40) | (rxBuff[1] << 32) | (rxBuff[3] << 24) |
					  (rxBuff[4] << 16) | (rxBuff[6] << 8) | rxBuff[7];

	Sensor.sensorID_Data_0_crc = rxBuff[2];
	Sensor.sensorID_Data_1_crc = rxBuff[5];
	Sensor.sensorID_Data_2_crc = rxBuff[8];
}

bool SCD4x_Perform_Self_Test(void)
{
	bool malfunctionFlag;
	uint8_t txBuff[2] = {0};
	uint8_t rxBuff[3] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x39;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(10000);
	HAL_I2C_Master_Receive(&hi2c1, SCD4x_I2C_ADDR, rxBuff, sizeof(rxBuff), 500);

	if(((rxBuff[0] << 8) | (rxBuff[1])) != 0)
		malfunctionFlag = true;
	else
		malfunctionFlag = false;

	return malfunctionFlag;
}

void SCD4x_Perform_Factory_Reset(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x32;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(1200);
}

void SCD4x_Reinit(void)
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x36;
	txBuff[1] = 0x46;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(20);
}

void SCD41x_Measure_Single_Shot(void) // SCD41 only.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0x9D;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(5000);
}

void SCD41x_Measure_Single_Shot_RHT_Only(void) // SCD41 only. Use read_measurement command, CO2 is returned as 0 ppm.
{
	uint8_t txBuff[2] = {0};

	txBuff[0] = 0x21;
	txBuff[1] = 0x96;

	HAL_I2C_Master_Transmit(&hi2c1, SCD4x_I2C_ADDR, txBuff, sizeof(txBuff), 500);
	HAL_Delay(50);
}

static uint8_t SCD4x_Generate_CRC(const uint8_t* data, uint16_t count)
{
	uint16_t current_byte;
	uint8_t crc = CRC8_INIT;
	uint8_t crc_bit;

	/* calculates 8-Bit checksum with given polynomial */
	for (current_byte = 0; current_byte < count; ++current_byte)
	{
		crc ^= (data[current_byte]);

		for (crc_bit = 8; crc_bit > 0; --crc_bit)
		{
			if (crc & 0x80)
				crc = (crc << 1) ^ CRC8_POLYNOMIAL;
			else
				crc = (crc << 1);
		}
	}
	return crc;
}

void SCD4x_Error_Handler(void) {}



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
