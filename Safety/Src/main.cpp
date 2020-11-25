#include "main.hpp"

#include "Debug.hpp"
#include "GPIO.hpp"
#include "Clock.hpp"
#include "UART.hpp"
#include "PWM.hpp"
#include "PPM.hpp"
#include "Watchdog.hpp"
#include "Profiler.h"

#include "stm32f0xx_hal.h"
#include "stm32f0xx_hal_iwdg.h"
#include "safety_controller.hpp"


char buffer[200]; //buffer for printing
StatusCode setupPWM(PWMManager &manager);
StatusCode setupPPM(PPMChannel &ppm);
void print_ppm_state(char *buffer, PPMChannel &ppm);

IWDG_HandleTypeDef hiwdg2; //HAL Watchdog Decleration
extern UART_HandleTypeDef huart2;


void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
void Error_Handler(void);
void callMXfunctions();

int main() {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	//HAL_IWDG_Init(&hiwdg2); //Start the watchdog
	SystemClock_Config();
	callMXfunctions();
	
	
	StatusCode status;

	PWMManager &manager = PWMManager::getInstance();
	status = setupPWM(manager);
	info("PWMSetup", status);


	PPMChannel ppm;
	status = setupPPM(ppm);

	info("PPM Setup", status);
	
	/*
	GPIOPin led1 = GPIOPin(LED1_GPIO_PORT, LED1_GPIO_PIN, GPIO_OUTPUT, GPIO_STATE_LOW, GPIO_RES_NONE);
	GPIOPin led2 = GPIOPin(LED2_GPIO_PORT, LED2_GPIO_PIN, GPIO_OUTPUT, GPIO_STATE_LOW, GPIO_RES_NONE);
	GPIOPin led3 = GPIOPin(LED3_GPIO_PORT, LED3_GPIO_PIN, GPIO_OUTPUT, GPIO_STATE_LOW, GPIO_RES_NONE);
	GPIOPin buzzer = GPIOPin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, GPIO_OUTPUT, GPIO_STATE_LOW, GPIO_RES_NONE);
	led1.set_state(GPIO_STATE_HIGH);
	led2.set_state(GPIO_STATE_HIGH);
	*/
	safety_controller_init();

	while (true) 
	{
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_10);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_11);
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_12);
		HAL_Delay(500);
		/*
		led1.set_state(GPIO_STATE_HIGH);
		led2.set_state(GPIO_STATE_HIGH);
		*/
		//safety_run(hiwdg2, ppm);

	}
}

void print_ppm_state(char *buffer, PPMChannel &ppm) {
	int len = sprintf(buffer,
					  "CH1 (p, us): %d %lu\r\nCH2 (p, us): %d %lu\r\n"
					  "CH3 (p, us): %d %lu\r\nCH4 (p, us): %d %lu\r\n"
					  "CH5 (p, us): %d %lu\r\nCH6 (p, us): %d %lu\r\n"
					  "CH7 (p, us): %d %lu\r\nCH8 (p, us): %d %lu\r\n",
					  ppm.get(1), ppm.get_us(1),
					  ppm.get(2), ppm.get_us(2),
					  ppm.get(3), ppm.get_us(3),
					  ppm.get(4), ppm.get_us(4),
					  ppm.get(5), ppm.get_us(5),
					  ppm.get(6), ppm.get_us(6),
					  ppm.get(7), ppm.get_us(7),
					  ppm.get(8), ppm.get_us(8));

	sprintf(&buffer[len], "PPM Disconnected? : %d\r\n", ppm.is_disconnected(get_system_time()));
}


StatusCode setupPWM(PWMManager &manager)
{
	StatusCode status = manager.setup();
	manager.channel(1).set(50);
	manager.channel(2).set(25);
	manager.channel(3).set(75);
	manager.channel(4).set(100);
	manager.channel(5).set(25);
	manager.channel(6).set(75);
	manager.channel(7).set(100);
	manager.channel(8).set(25);
	manager.channel(9).set(50);
	manager.channel(10).set(75);
	manager.channel(11).set(100);
	manager.channel(12).set(50);

	return status;
}

StatusCode setupPPM(PPMChannel &ppm)
{
	ppm.setNumChannels(8);
	ppm.setLimits(1, 1000, 2000, 50);
	StatusCode status = ppm.setup();
	ppm.setTimeout(200);
	return status;
}


void SystemClock_Config(void)
{

 RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  HAL_RCC_MCOConfig(RCC_MCO, RCC_MCO1SOURCE_SYSCLK, RCC_MCODIV_1);

 
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

void callMXfunctions()
{
	MX_GPIO_Init();
	MX_USART2_UART_Init();
	/*
  	MX_I2C1_Init();
  	MX_IWDG_Init();
  	MX_SPI1_Init();
  	MX_TIM1_Init();
  	MX_TIM3_Init();
  	MX_TIM14_Init();
  	MX_TIM15_Init();
  	MX_TIM16_Init();
  	MX_TIM17_Init();
  	MX_USART1_UART_Init();
  	
	  */
}

static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}