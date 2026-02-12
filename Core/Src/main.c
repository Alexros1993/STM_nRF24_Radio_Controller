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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define CS_LOW() HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET)
#define CS_HIGH() HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET)

#define CE_LOW() HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_RESET)
#define CE_HIGH() HAL_GPIO_WritePin(SPI1_CE_GPIO_Port, SPI1_CE_Pin, GPIO_PIN_SET)

#define TX_BTN_LOW() HAL_GPIO_WritePin(TX_BTN_GPIO_Port, TX_BTN_Pin, GPIO_PIN_RESET)
#define TX_BTN_HIGH() HAL_GPIO_WritePin(TX_BTN_GPIO_Port, TX_BTN_Pin, GPIO_PIN_SET)

#define W_REGISTER 0x20
#define R_REGISTER 0X00
#define STATUS_REGISTER 0x07
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
typedef struct {
  HAL_StatusTypeDef hal_status;
  uint8_t operation_result;
} nRF24_Operation_Status;

typedef struct {
  uint8_t Rx_dr;
  uint8_t Tx_ds;
  uint8_t Max_rt;
  uint8_t Rx_pipe;
  uint8_t Tx_full;
} nRF24_StatusTypeDef;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
nRF24_Operation_Status write_to_reg(uint8_t addr, uint8_t byte);
nRF24_Operation_Status read_reg(uint8_t reg);
void init_config();
nRF24_Operation_Status read_and_modify(uint8_t reg, uint8_t mask, uint8_t value);
nRF24_StatusTypeDef process_status(uint8_t reg);
nRF24_Operation_Status clear_status(uint8_t mask);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
nRF24_Operation_Status write_to_reg(const uint8_t addr, const uint8_t byte) {
  if (HAL_GPIO_ReadPin(SPI1_CE_GPIO_Port, SPI1_CE_Pin) == 1) {
    // Add debug log
    const nRF24_Operation_Status status = {
      HAL_ERROR, 0
    };
    return status;
  }

  const uint8_t data[2] = {(W_REGISTER | (addr & 0x1F)), byte};
  CS_LOW();
  const HAL_StatusTypeDef HAL_Result = HAL_SPI_Transmit(&hspi1, data, sizeof(data), 1000);
  CS_HIGH();

  const nRF24_Operation_Status status = {
    HAL_Result, byte
  };

  return status;
}

nRF24_Operation_Status read_reg(const uint8_t reg) {
  if (HAL_GPIO_ReadPin(SPI1_CE_GPIO_Port, SPI1_CE_Pin) == 1) {
    // Add debug log
    const nRF24_Operation_Status status = {
      HAL_ERROR, 0
    };
    return status;
  }
  uint8_t  received_reg_data[2];
  const uint8_t reading_reg = (R_REGISTER | (reg & 0x1F));
  const uint8_t sending_data[2] = {reading_reg, 0x0};
  CS_LOW();
  const HAL_StatusTypeDef HAL_status = HAL_SPI_TransmitReceive(&hspi1, sending_data, received_reg_data, sizeof(sending_data), 1000);
  CS_HIGH();

  const nRF24_Operation_Status op_status = {
    HAL_status, received_reg_data[1]
  };

  return  op_status;
}

nRF24_Operation_Status read_and_modify(const uint8_t reg, const uint8_t mask, const uint8_t value) {
  const nRF24_Operation_Status read_op_status = read_reg(reg);

  if (read_op_status.hal_status == HAL_OK) {
    if ((read_op_status.operation_result & mask) == (value & mask)) {
      return read_op_status;
    }
    const uint8_t reg_value = read_op_status.operation_result;
    const uint8_t byte = (reg_value & ~mask) | (value & mask);
    const nRF24_Operation_Status write_op_status = write_to_reg(reg, byte);

    return write_op_status;
  }
  return read_op_status;
}

nRF24_StatusTypeDef process_status(const uint8_t reg) {
  const nRF24_StatusTypeDef status = {
    (reg >> 6) & 0x01,
    (reg >> 5) & 0x01,
    (reg >> 4) & 0x01,
    (reg >> 1) & 0x07,
    (reg >> 0) & 0x01,
  };
  return status;
}

nRF24_Operation_Status clear_status(const uint8_t mask) {
  const uint8_t allowed_mask = 0b01110000;

  if ((mask & ~allowed_mask) == 0) {
    const uint8_t data[2] = {(W_REGISTER | (STATUS_REGISTER & 0x1F)), mask};
    CS_LOW();
    const HAL_StatusTypeDef HAL_Result = HAL_SPI_Transmit(&hspi1, data, sizeof(data), 1000);
    CS_HIGH();

    const nRF24_Operation_Status w_op_status = {
      HAL_Result,
      mask
    };
    return w_op_status;
  }

  const nRF24_Operation_Status status = {
    HAL_ERROR, 0
  };
  return status;
}

void init_config() {
  
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

  /* USER CODE END 2 */

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TX_BTN_Pin|SPI1_CS_Pin|SPI1_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : TX_BTN_Pin */
  GPIO_InitStruct.Pin = TX_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TX_BTN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_CS_Pin SPI1_CE_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin|SPI1_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
