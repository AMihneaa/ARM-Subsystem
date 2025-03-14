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
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
#define NUMS_LED 8 //Number of LEDs
#define NUMS_BITS_DATA 4

// LED Pins
#define LED_PINS {LED_MOD_Pin, LED_PREG_Pin, LED_DAT_Pin, LED_START_Pin, LED_DATA0_Pin, LED_DATA1_Pin, LED_DATA2_Pin, LED_DATA3_Pin}

// LED Ports
#define LED_PORTS {LED_MOD_GPIO_Port, LED_PREG_GPIO_Port, LED_DAT_GPIO_Port, LED_START_GPIO_Port, LED_DATA0_GPIO_Port, LED_DATA1_GPIO_Port, LED_DATA2_GPIO_Port, LED_DATA3_GPIO_Port}

uint8_t startStatus = 0; // Start Status Variable
uint8_t datStatus = 0; // DAT Status Variable
uint8_t modStatus = 0; // MOD Status Variable
uint8_t pregStatus = 0; // PREG Status Variable
uint8_t ledData[NUMS_LED]; // Array for storing LED Status
uint8_t dataArr[NUMS_BITS_DATA]; // Array used for storing data for transmissions


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void EXTI_INIT(void);

void sendLED(void);
void updateLedData(uint16_t, uint8_t);
void ARM_WR_CDA(void);
void ARM_WR_DATA(void);
void ARM_RD_DATA(void);

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
  HAL_TIM_Base_Start_IT(&htim2); // Init Tim2 Instance

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  EXTI_Init();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_PLLCLK, RCC_MCODIV_1);
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

	 TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	    TIM_MasterConfigTypeDef sMasterConfig = {0};

	    htim2.Instance = TIM2;
	    htim2.Init.Prescaler = 7199; // 72MHz / (7199 + 1) = 10kHz
	    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	    htim2.Init.Period = 99; // 10kHz / (99 + 1) = 100Hz (10ms)
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

	    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	    {
	        Error_Handler();
	    }

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DATA0_Pin|DATA1_Pin|DATA2_Pin|DATA3_Pin
                          |CDA0_Pin|CDA1_Pin|CLK_Pin|LED_DATA0_Pin
                          |LED_DATA1_Pin|LED_DATA2_Pin|LED_DATA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_MOD_Pin|LED_PREG_Pin|LED_DAT_Pin|LED_START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : DATA0_Pin DATA1_Pin DATA2_Pin DATA3_Pin
                           CDA0_Pin CDA1_Pin CLK_Pin LED_DATA0_Pin
                           LED_DATA1_Pin LED_DATA2_Pin LED_DATA3_Pin */
  GPIO_InitStruct.Pin = DATA0_Pin|DATA1_Pin|DATA2_Pin|DATA3_Pin
                          |CDA0_Pin|CDA1_Pin|CLK_Pin|LED_DATA0_Pin
                          |LED_DATA1_Pin|LED_DATA2_Pin|LED_DATA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ACK_Pin DAT_Pin START_Pin */
  GPIO_InitStruct.Pin = ACK_Pin|DAT_Pin|START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_DATA0_Pin INPUT_DATA1_Pin INPUT_DATA2_Pin INPUT_DATA3_Pin
                           MOD_Pin PREG_Pin */
  GPIO_InitStruct.Pin = INPUT_DATA0_Pin|INPUT_DATA1_Pin|INPUT_DATA2_Pin|INPUT_DATA3_Pin
                          |MOD_Pin|PREG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB
		  , &GPIO_InitStruct);

  /*Configure GPIO pins : LED_MOD_Pin LED_PREG_Pin LED_DAT_Pin LED_START_Pin */
  GPIO_InitStruct.Pin = LED_MOD_Pin|LED_PREG_Pin|LED_DAT_Pin|LED_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/*
 * Check if startStatus || datStaus
 */
void TIM2_CallBack(void){
	if ((startStatus & 1) | (startStatus & (1 << 1))){
		if (startStatus == 1){
			sendLED();
			startStatus = 1 << 1;

			return ;
		}

		ARM_WR_CDA();
		startStatus = 0;
	}else if ((datStatus & 1) | (datStatus & (1 << 1))){
		if (datStatus == 1){
			sendLED();
			datStatus = 1 << 1;

			return ;
		}

		if (modStatus){
			ARM_WR_DATA();
		}else{
			ARM_RD_DATA();
		}
	}
}

/*
 * SET/RESET led based on ledData array
*/
void sendLED(void){
    uint16_t led_pins[] = LED_PINS;
    GPIO_TypeDef* led_ports[] = LED_PORTS;

    for (int i = 0; i < NUMS_LED; i++) {
           HAL_GPIO_WritePin(*(led_ports + i), *(led_pins + i), *(ledData + i) != 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
   }
}

/*
 * @brief Updates the `ledData[]` array for a specific LED pin.
 * @param pin: The LED pin to modify.
 * @param state: The new state (1 = ON, 0 = OFF).
 */
void updateLedData(uint16_t pin, uint8_t state){
	uint16_t led_pins[] = LED_PINS;

	for(int i = 0; i < NUMS_LED; i++){
		if (pin == *(led_pins + i)){
			*(ledData + i) = state;
			break;
		}
	}
}

/*
 * Write CDA = 2 * PRG + MOD;
 */
void ARM_WR_CDA(void){
	// CDA = 2 * pregStatus + modStatus;


	return ;
}

/*
 *
 */
void ARM_WR_DATA(void){

	return ;
}

/*
 *
 */
void ARM_RD_DATA(void){
	return ;
}

/*
 * Enable GPIO IR
 */
void EXTI_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable clocks for GPIOA and GPIOB
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    // Define Pins & Ports Arrays
    uint16_t exti_pins[] = {START_Pin, DAT_Pin, PREG_Pin, MOD_Pin};
    GPIO_TypeDef* exti_ports[] = {START_GPIO_Port, DAT_GPIO_Port, PREG_GPIO_Port, MOD_GPIO_Port};

    // Configure

    //Setting up port for Rising Only Interrupt
    for (int i = 0; i < 2; i++){
    	GPIO_InitStruct.Pin = *(exti_pins + i);
    	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    	HAL_GPIO_Init(*(exti_ports + i), &GPIO_InitStruct);
    }

    // Setting up port for Rising & Falling Interrupt
    for (int i = 2; i < 4; i++){
    	GPIO_InitStruct.Pin = *(exti_pins + i); //Alternative for exti_pins[i]
    	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
    	GPIO_InitStruct.Pull = GPIO_NOPULL;
    	HAL_GPIO_Init(*(exti_ports + i), &GPIO_InitStruct); //Alternative for exti_ports[i]
    }

    // Enable NVIC for the corresponding EXTI lines
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI1_IRQn);

    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void Start_Button_Pressed(void){
	HAL_DELAY(10);
	/*
	 * Check if the button was intentionally press
	 * startStatus = 1 << 0
	 * startStatys = 1 << 1 = 2
	 */
	if (HAL_GPIO_ReadPin(START_GPIO_PORT, START_Pin) == GPIO_PIN_SET){
		startStatus = 1 << startStatus;
	}

}

void Dat_Button_Pressed(void){
	HAL_DELAY(10);

	/*
	 * Check if the button was intentionally press
	 */
	if (HAL_GPIO_ReadPin(DAT_GPIO_PORT, DAT_Pin) == GPIO_PIN_SET){
		datStatus = 1 << datStatus;
	}

}

void Preg_Button_Pressed(void){
	pregStatus ^= 1;
}

void Mod_Button_Pressed(void){
	modStatus ^= 1;
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
