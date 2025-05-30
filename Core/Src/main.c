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
#include "stm32f1xx_hal.h"

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

#define BUTTONS_PINS   {INPUT_DATA0_Pin, INPUT_DATA1_Pin, INPUT_DATA2_Pin, INPUT_DATA3_Pin}
#define BUTTONS_PORTS  {INPUT_DATA0_GPIO_Port, INPUT_DATA1_GPIO_Port, INPUT_DATA2_GPIO_Port, INPUT_DATA3_GPIO_Port}

#define DATA_PINS   {DATA0_Pin, DATA1_Pin, DATA2_Pin, DATA3_Pin}
#define DATA_PORTS  {DATA0_GPIO_Port, DATA1_GPIO_Port, DATA2_GPIO_Port, DATA3_GPIO_Port}

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
uint8_t cdaStatus = 0; // (CDA & 1) Status for CDA0_Pin and (CDA & (1 << 1)) Status for CDA1_Pin

uint8_t Q = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void EXTI_Init();
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
//  EXTI_Init();

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

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR2;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DATA0_Pin|DATA1_Pin|DATA2_Pin|DATA3_Pin
                          |CDA0_Pin|CDA1_Pin|LED_DATA1_Pin|LED_DATA2_Pin
                          |LED_DATA3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_RGB0_Pin|LED_RGB1_Pin|LED_MOD_Pin|LED_PREG_Pin
                          |LED_DAT_Pin|LED_START_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : DATA0_Pin DATA1_Pin DATA2_Pin DATA3_Pin
                           CDA0_Pin CDA1_Pin LED_DATA1_Pin LED_DATA2_Pin
                           LED_DATA3_Pin */
  GPIO_InitStruct.Pin = DATA0_Pin|DATA1_Pin|DATA2_Pin|DATA3_Pin
                          |CDA0_Pin|CDA1_Pin|LED_DATA1_Pin|LED_DATA2_Pin
                          |LED_DATA3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_DATA0_Pin LED_DATA0_Pin */
  GPIO_InitStruct.Pin = INPUT_DATA0_Pin|LED_DATA0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_RGB0_Pin LED_RGB1_Pin LED_MOD_Pin LED_PREG_Pin
                           LED_DAT_Pin LED_START_Pin */
  GPIO_InitStruct.Pin = LED_RGB0_Pin|LED_RGB1_Pin|LED_MOD_Pin|LED_PREG_Pin
                          |LED_DAT_Pin|LED_START_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : INPUT_DATA1_Pin INPUT_DATA2_Pin INPUT_DATA3_Pin DAT_Pin
                           START_Pin ACK_Pin CLK_Pin MOD_Pin
                           PREG_Pin */
  GPIO_InitStruct.Pin = INPUT_DATA1_Pin|INPUT_DATA2_Pin|INPUT_DATA3_Pin|DAT_Pin
                          |START_Pin|ACK_Pin|CLK_Pin|MOD_Pin
                          |PREG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_RGB2_Pin */
  GPIO_InitStruct.Pin = LED_RGB2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_RGB2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*
 * @brief Timer2 interrupt callback (every 10ms).
 *        Handles communication protocol FSM between two MCUs.
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if (htim->Instance != TIM2){
		return ;
	}

	switch(Q){  // Finite State Machine (FSM) controller using Q as state variable
		case 0:
			// Check if START button is pressed
			if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET){
				Q = 3; // Go to START signal handling state
				startStatus = 1; // Mark that dat was pressed
				updateLedData(LED_START_Pin, (1 & startStatus)); // Update LED status
			}

			// Check if DAT button is pressed
			if (HAL_GPIO_ReadPin(DAT_GPIO_Port, DAT_Pin) == GPIO_PIN_SET){
				Q = 1; // Go to data signal handling state
				datStatus = 1; // Mark that start was pressed
				updateLedData(LED_DAT_Pin, (1 & datStatus)); // Update LED status
			}

			// Check if PREG button toggled
			if (HAL_GPIO_ReadPin(PREG_GPIO_Port, PREG_Pin) == GPIO_PIN_SET){
				pregStatus = 1; // Set PREG status to 1
				updateLedData(LED_PREG_Pin, pregStatus); // Update LED
			}

			// Check if MOD button toggled
			if (HAL_GPIO_ReadPin(MOD_GPIO_Port, MOD_Pin) == GPIO_PIN_SET){
				modStatus = 1; // Set MOD status to 1
				updateLedData(LED_MOD_Pin, modStatus); // Update LED
			}

			sendLED(); // Update all LED outputs according to ledData[]
			break;

		case 1:
			// Wait for START button to be still pressed
			if (HAL_GPIO_ReadPin(START_GPIO_Port, START_Pin) == GPIO_PIN_SET){
				Q = 2; // Go to START setup
				startStatus = 1; // Toggle START flag
			}
			break;

		case 2:
		{
			// Set ACK pin as input
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = ACK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(ACK_GPIO_Port, &GPIO_InitStruct);

			// Set CLK pin as output
			GPIO_InitStruct.Pin = CLK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

			// Prepare data pins as OUTPUT for transmission
			uint16_t data_pins[] = DATA_PINS;
			GPIO_TypeDef* data_ports[] = DATA_PORTS;
			setPinMode(data_pins, data_ports, NUMS_BITS_DATA, GPIO_MODE_OUTPUT_PP);

			Q = 10; // Proceed to transmit command (CDA)
			break;
		}

		case 3:
			// Wait for DAT button to be still pressed
			if (HAL_GPIO_ReadPin(DAT_GPIO_Port, DAT_Pin) == GPIO_PIN_SET){
				Q = 4; // Continue flow
				datStatus = 1; // Toggle DAT flag
			}
			break;

		case 4:
			// Decide whether to read or write based on MOD button
			if (modStatus){
				Q = 6; // MOD = 1 → Read mode
			}else{
				Q = 5; // MOD = 0 → Write mode
			}
			break;

		case 5:
		{
			// Set ACK as OUTPUT
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = ACK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(ACK_GPIO_Port, &GPIO_InitStruct);

			// Set CLK as INPUT (will wait for clock from external device)
			GPIO_InitStruct.Pin = CLK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

			Q = 30; // Ready to transmit actual data
			break;
		}

		case 6:
		{
			// Setup for reading data
			GPIO_InitTypeDef GPIO_InitStruct = {0};
			GPIO_InitStruct.Pin = ACK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
			HAL_GPIO_Init(ACK_GPIO_Port, &GPIO_InitStruct);

			// Set CLK pin as OUTPUT (we will toggle it)
			GPIO_InitStruct.Pin = CLK_Pin;
			GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
			GPIO_InitStruct.Pull = GPIO_NOPULL;
			HAL_GPIO_Init(CLK_GPIO_Port, &GPIO_InitStruct);

			// Configure data pins as INPUT
			uint16_t data_pins[] = DATA_PINS;
			GPIO_TypeDef* data_ports[] = DATA_PORTS;
			setPinMode(data_pins, data_ports, NUMS_BITS_DATA, GPIO_MODE_INPUT);

			Q = 60; // Ready to receive data
			break;
		}

		case 10:
			// Set CDA based on MOD and PREG button states
			cdaStatus = 2 * pregStatus + modStatus;
			Q = 11;
			break;

		case 11:
			// Transmit CDA on 2 pins
			WR_CDA();

			// Set CLK low to signal start of transmission
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET);
			Q = 12;
			break;

		case 12:
			// Set CLK high to latch data
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET);
			Q = 13;
			break;

		case 13:
			// Wait for ACK to go LOW (waiting for external device)
			if (HAL_GPIO_ReadPin(ACK_GPIO_Port, ACK_Pin) != GPIO_PIN_SET){
				Q = 14;
			}
			break;

		case 14:
			// Wait for ACK to go HIGH again (external device accepted command)
			if (HAL_GPIO_ReadPin(ACK_GPIO_Port, ACK_Pin) == GPIO_PIN_SET){
				startStatus = 0; // Reset flag
				Q = 0; // Go back to idle
			}
			break;

		case 30:
			readButtonData(); // Read 4-bit data
			SEND_DATA();      // Send it through GPIO
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_RESET); // Clock low
			Q = 31;
			break;

		case 31:
			HAL_GPIO_WritePin(CLK_GPIO_Port, CLK_Pin, GPIO_PIN_SET); // Clock high
			Q = 32;
			break;

		case 32:
			// Wait for external device to lower ACK
			if (HAL_GPIO_ReadPin(ACK_GPIO_Port, ACK_Pin) != GPIO_PIN_SET){
				Q = 33;
			}
			break;

		case 33:
			// Wait for external device to raise ACK back
			if (HAL_GPIO_ReadPin(ACK_GPIO_Port, ACK_Pin) != GPIO_PIN_SET){
				Q = 0;
				datStatus = 0; // Reset data flag
			}
			break;

		case 60:
			// Wait for CLK to go LOW (indicating start from external MCU)
			if (HAL_GPIO_ReadPin(CLK_GPIO_Port, CLK_Pin) != GPIO_PIN_SET){
				Q = 61;
			}
			break;

		case 61:
			// Wait for CLK to go HIGH again (indicating data ready)
			if (HAL_GPIO_ReadPin(CLK_GPIO_Port, CLK_Pin) == GPIO_PIN_SET){
				Q = 62;
			}
			break;

		case 62:
			READ_DATA(); // Read incoming 4-bit data
			HAL_GPIO_WritePin(ACK_GPIO_Port, ACK_Pin, GPIO_PIN_RESET); // Pull ACK low
			Q = 63;
			break;

		case 63:
			// Finish by pulling ACK high
			HAL_GPIO_WritePin(ACK_GPIO_Port, ACK_Pin, GPIO_PIN_SET);
			datStatus = 0; // Reset flag
			Q = 0; // Back to idle
			break;

		default:
			// Safety default in case Q has invalid value
			break;
	}
}


/*
 * @brief Updates physical LED outputs based on the `ledData[]` array.
 *        Each element in `ledData` corresponds to an LED's ON (1) or OFF (0) state.
 */
void sendLED(void){
    // Define local arrays holding pin numbers and corresponding GPIO ports for the LEDs.
    uint16_t led_pins[] = LED_PINS;
    GPIO_TypeDef* led_ports[] = LED_PORTS;

    // Loop through each LED (total NUMS_LED = 8)
    for (int i = 0; i < NUMS_LED; i++) {
        /*
         * Write to each LED pin:
         * - If `ledData[i]` is 1 → GPIO_PIN_SET (turn LED ON)
         * - If `ledData[i]` is 0 → GPIO_PIN_RESET (turn LED OFF)
         * Uses pointer arithmetic to access port and pin arrays.
         */
        HAL_GPIO_WritePin(
            *(led_ports + i),                  // LED GPIO port
            *(led_pins + i),                   // LED GPIO pin
            (*(ledData + i) & 1) != 0          // LED state: ON if value is 1, else OFF
                ? GPIO_PIN_SET
                : GPIO_PIN_RESET
        );
    }
}

/*
 * @brief Updates the `ledData[]` array for a specific LED pin.
 *        This function is used to change the logical state of an LED
 *        (used later by sendLED() to update physical output).
 *
 * @param pin:   The specific GPIO pin associated with the LED.
 * @param state: Desired state for the LED (1 = ON, 0 = OFF).
 */
void updateLedData(uint16_t pin, uint8_t state){
    // Create a local array containing the list of all defined LED pins
    uint16_t led_pins[] = LED_PINS;

    // Loop through the list of LED pins to find the matching one
    for(int i = 0; i < NUMS_LED; i++){
        // If the given pin matches the current pin in the array
        if (pin == *(led_pins + i)){
            // Update the corresponding index in ledData with the new state
            *(ledData + i) = state;
            // Exit the loop after finding the match
            break;
        }
    }
}


/*
 * Enable GPIO IR
 */
void EXTI_Init(void) {
//    GPIO_InitTypeDef GPIO_InitStruct = {0};
//
//    // Enable clocks for GPIOA and GPIOB (Already enabled in MX_GPIO_Init )
//    	__HAL_RCC_GPIOA_CLK_ENABLE();
//    	__HAL_RCC_GPIOB_CLK_ENABLE();
//
//    // Define Pins & Ports Arrays
//    uint16_t exti_pins[] = {START_Pin, DAT_Pin, PREG_Pin, MOD_Pin};
//    GPIO_TypeDef* exti_ports[] = {START_GPIO_Port, DAT_GPIO_Port, PREG_GPIO_Port, MOD_GPIO_Port};
//
//    // Configure
//
//    //Setting up port for Rising Only Interrupt
//    for (int i = 0; i < 2; i++){
//    	GPIO_InitStruct.Pin = *(exti_pins + i);
//    	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
//    	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
//    	HAL_GPIO_Init(*(exti_ports + i), &GPIO_InitStruct);
//    }
//
//    // Setting up port for Rising & Falling Interrupt
//    for (int i = 2; i < 4; i++){
//    	GPIO_InitStruct.Pin = *(exti_pins + i); //Alternative for exti_pins[i]
//    	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
//    	GPIO_InitStruct.Pull = GPIO_NOPULL;
//    	HAL_GPIO_Init(*(exti_ports + i), &GPIO_InitStruct); //Alternative for exti_ports[i]
//    }
//
//    // Enable NVIC for the corresponding EXTI lines
//    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
//    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
//
//    HAL_NVIC_SetPriority(EXTI1_IRQn, 2, 0);
//    HAL_NVIC_EnableIRQ(EXTI1_IRQn);
//
//    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
//    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/*
 * @brief Reads data from CDA0 and CDA1 pins.
 * @details Configures the pins as input, reads their states, and stores values.
 */
void RD_CDA(void){
		GPIO_InitTypeDef GPIO_InitStruct = {0};

	    // Configure CDA0 and CDA1 as INPUT mode
	    GPIO_InitStruct.Pin = CDA0_Pin | CDA1_Pin;
	    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	    GPIO_InitStruct.Pull = GPIO_NOPULL;
	    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	    // Read pin states
	    uint8_t cda0_state = HAL_GPIO_ReadPin(CDA0_GPIO_Port, CDA0_Pin);
	    uint8_t cda1_state = HAL_GPIO_ReadPin(CDA1_GPIO_Port, CDA1_Pin);

	    // Store or process the received data (modify as needed)
	    cdaStatus = (cda1_state << 1) | cda0_state;
}

/*
 * @brief Writes data to CDA0 and CDA1 pins.
 *
 * @details This function uses the global variable `cdaStatus` to set the logic level
 *          on two output pins (CDA0 and CDA1). The value of `cdaStatus` is a 2-bit number (0 to 3)
 *          and each bit controls one of the two CDA pins:
 *            - Bit 0 (LSB) controls CDA0
 *            - Bit 1 controls CDA1
 *          The pin logic is set as follows:
 *            - 00 (0) -> CDA0 = LOW, CDA1 = LOW
 *            - 01 (1) -> CDA0 = HIGH, CDA1 = LOW
 *            - 10 (2) -> CDA0 = LOW, CDA1 = HIGH
 *            - 11 (3) -> CDA0 = HIGH, CDA1 = HIGH
 */
void WR_CDA(void){
    // Set CDA0 pin based on bit 0 of cdaStatus.
    // If bit 0 is 1, set CDA0 HIGH; otherwise, set it LOW.
    HAL_GPIO_WritePin(CDA0_GPIO_Port, CDA0_Pin, (cdaStatus & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Set CDA1 pin based on bit 1 of cdaStatus.
    // If bit 1 is 1, set CDA1 HIGH; otherwise, set it LOW.
    HAL_GPIO_WritePin(CDA1_GPIO_Port, CDA1_Pin, (cdaStatus & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


/*
 * @brief Reads data from button input pins and stores the values in the `dataArr[]` array.
 *
 * @details Each button is connected to a specific pin (defined in BUTTONS_PINS and BUTTONS_PORTS).
 *          This function loops through all 4 buttons, reads their digital state (HIGH or LOW),
 *          and stores the result (0 or 1) in the `dataArr[]` array at the corresponding index.
 *
 *          The array `dataArr[]` acts as a buffer to hold the state of each input button.
 *
 *          - INPUT: BUTTONS_PINS[] = list of pins connected to buttons
 *                   BUTTONS_PORTS[] = list of corresponding GPIO ports
 *          - OUTPUT: dataArr[] = stores the state (0 or 1) for each button
 */
void readButtonData(void){
    // Define the pins connected to each input button
    uint16_t btn_pins[] = BUTTONS_PINS;

    // Define the ports where each button is connected
    GPIO_TypeDef *btn_ports[] = BUTTONS_PORTS;

    // Iterate through all input buttons (0 to NUMS_BITS_DATA - 1)
    for (int i = 0; i < NUMS_BITS_DATA; i++){
        // Read the state of each button pin (GPIO_PIN_SET or GPIO_PIN_RESET)
        // and store 0 or 1 into the dataArr array
        *(dataArr + i) = HAL_GPIO_ReadPin(*(btn_ports + i), *(btn_pins + i));
    }
}


/*
 * @brief Configures the DATA pins as output and sends the values stored in the `dataArr[]` array.
 *
 * @details This function iterates over the 4 data lines (DATA0 to DATA3) and writes either
 *          a HIGH or LOW logic level depending on the value stored in the `dataArr[]`.
 *
 *          - If `dataArr[i] == 1`, the corresponding GPIO pin is set to HIGH (GPIO_PIN_SET).
 *          - If `dataArr[i] == 0`, the GPIO pin is set to LOW (GPIO_PIN_RESET).
 *
 *          The data pins and their corresponding ports are stored in the `DATA_PINS` and `DATA_PORTS` macros.
 */
void SEND_DATA(void){
    // Array of data pin numbers (e.g., DATA0_Pin to DATA3_Pin)
    uint16_t data_pins[] = DATA_PINS;

    // Array of corresponding GPIO port pointers (e.g., GPIOA)
    GPIO_TypeDef* data_ports[] = DATA_PORTS;

    // Iterate through each data pin
    for (int i = 0; i < NUMS_BITS_DATA; i++) {
        // Write the logic level to the pin based on dataArr[i]
        // If dataArr[i] is 1 → GPIO_PIN_SET (logic high)
        // If dataArr[i] is 0 → GPIO_PIN_RESET (logic low)
        HAL_GPIO_WritePin(*(data_ports + i), *(data_pins + i), (dataArr[i] ? GPIO_PIN_SET : GPIO_PIN_RESET));
    }
}


/*
 * @brief Configures DATA pins as input and reads values into `dataArr[]`.
 *
 * @details This function reads the digital state (HIGH or LOW) of each data pin (DATA0 to DATA3)
 *          and stores the result in the `dataArr[]` array. Additionally, it calls `updateLedData()`
 *          for each pin to reflect the read value on the corresponding LED.
 */
void READ_DATA(void){
    // Define the array of data pin numbers (e.g., DATA0_Pin to DATA3_Pin)
    uint16_t data_pins[] = DATA_PINS;

    // Define the array of corresponding GPIO port pointers (e.g., GPIOA)
    GPIO_TypeDef* data_ports[] = DATA_PORTS;

    // Loop through each data pin
    for (int i = 0; i < NUMS_BITS_DATA; i++) {
        // Read the pin value: 1 (HIGH) or 0 (LOW)
        *(dataArr + i) = HAL_GPIO_ReadPin(*(data_ports + i), *(data_pins + i));

        // Update the LED data state to reflect the read value
        updateLedData(*(data_pins + i), *(dataArr + i));
    }
}


/*
 * @brief Configures multiple GPIO pins to a specified mode.
 *
 * @param dataPins: Array of GPIO pins to be configured (e.g., DATA0_Pin, DATA1_Pin).
 * @param dataPorts: Array of corresponding GPIO ports for each pin (e.g., GPIOA, GPIOB).
 * @param numPins: Total number of pins to configure.
 * @param mode: GPIO mode to apply (e.g., GPIO_MODE_INPUT, GPIO_MODE_OUTPUT_PP).
 */
void setPinMode(uint16_t *dataPins, GPIO_TypeDef **dataPorts, uint8_t numPins, uint32_t mode) {
    // Structure used to define the configuration for each GPIO pin
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Loop over each pin to apply the configuration
    for (int i = 0; i < numPins; i++) {
        // Set the current pin from the array
        GPIO_InitStruct.Pin = dataPins[i]; // *(dataPins + i)

        // Set the desired mode (input/output/pull-up/etc.)
        GPIO_InitStruct.Mode = mode;

        // No internal pull-up or pull-down resistors enabled
        GPIO_InitStruct.Pull = GPIO_NOPULL;

        // Set pin speed to low (sufficient for digital GPIO use)
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

        // Apply the configuration to the corresponding port and pin
        HAL_GPIO_Init(dataPorts[i], &GPIO_InitStruct);
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
