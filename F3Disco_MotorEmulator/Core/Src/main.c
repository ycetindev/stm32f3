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
#include <stdbool.h>
#include "motor_model.h"

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
DAC_HandleTypeDef hdac;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim6;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DAC_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

volatile uint32_t printf_counter = 0;
volatile uint32_t tick_10kHz = 0;
volatile uint32_t tick_1kHz = 0;
volatile uint32_t tick_100Hz = 0;
volatile uint32_t tick_10Hz = 0;
volatile uint32_t tick_1Hz = 0;
volatile bool IsFirstCaptured = 0;
volatile uint32_t ICVal1 = 0, ICVal2 = 0, Difference = 0, frequency = 0;
volatile float duty_cycle = 0.0;
volatile float applied_voltage = 0.0;
const float VDC = 12.0; // fixed VDC

#define SCALE_CURRENT 250U
#define SCALE_OMEGA   1000U

void TaskA_100kHz(void)
{

	update_motor_model(applied_voltage);
}

void TaskB_10kHz(void)
{
	duty_cycle = (Difference * 100) / TIM2->ARR;
	applied_voltage = (duty_cycle * 0.01f) * VDC;
#if 0
	if(applied_voltage != 0)
	{
		if(printf_counter <= 50000U) // 5sec
		{
			printf("%lu, %.2f, %.2f \n", printf_counter, dbg_current, dbg_omega);
			printf_counter++;
		}
	}
#endif
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dbg_current * SCALE_CURRENT);
	HAL_DAC_SetValue(&hdac, DAC_CHANNEL_2, DAC_ALIGN_12B_R, dbg_omega * SCALE_OMEGA);
}

void Check_Duty_Zero(void)
{
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET) // to detect 0% PWM
    {
    	Difference = 0; // clear
        IsFirstCaptured = 0;  // reset
    }
}

void TaskD_100Hz(void)
{
	Check_Duty_Zero();
}

static inline void run_scheduler(void)
{
	TaskA_100kHz();

	if (++tick_10kHz >= 10)  		{ tick_10kHz = 0; 	TaskB_10kHz(); 	}
	//if (++tick_1kHz  >= 100)  		{ tick_1kHz = 0; 	TaskC_1kHz(); 	}
	if (++tick_100Hz >= 1000) 		{ tick_100Hz = 0; 	TaskD_100Hz(); 	}
	//if (++tick_10Hz  >= 10000) 		{ tick_10Hz = 0; 	TaskE_10Hz(); 	}
	//if (++tick_1Hz   >= 100000) 	{ tick_1Hz = 0; 	TaskF_1Hz(); 	}
}


void TIM6_DAC_IRQHandler(void)
{
	if(__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE))
	{
		__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);

		run_scheduler();
	}

}

void TIM2_IRQHandler(void)
{
	if((TIM2->SR) & (TIM_SR_CC1IF))
	{
		TIM2->SR = ~(TIM_SR_CC1IF);

		if(IsFirstCaptured == 0 && (GPIOA->IDR & GPIO_PIN_15))
		{//rising edge
			IsFirstCaptured = 1;
			ICVal1 = TIM2->CCR1;
		}
		else if(IsFirstCaptured)
		{//falling edge
			ICVal2 = TIM2->CCR1;
			if(ICVal2 > ICVal1)
			{
				Difference = ICVal2 - ICVal1;
			}
			else
			{
				Difference = (TIM2->ARR - ICVal1) + ICVal2;
			}
			IsFirstCaptured = 0;
		}
	}
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
	MX_DAC_Init();
	MX_TIM6_Init();
	MX_TIM2_Init();
	/* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac, DAC_CHANNEL_2);

	HAL_TIM_Base_Start_IT(&htim6);

	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);  // Start Input Capture

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin))
		{
			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
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

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
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
}

/**
 * @brief DAC Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC_Init(void)
{

	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */

	/** DAC Initialization
	 */
	hdac.Instance = DAC;
	if (HAL_DAC_Init(&hdac) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC_Init 2 */

	/* USER CODE END DAC_Init 2 */

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
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 0;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 7199;
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
	if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 15;
	if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

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
	htim6.Init.Prescaler = 9;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 71;
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
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOE, LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
			|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : B1_Pin */
	GPIO_InitStruct.Pin = B1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD7_Pin
                           LD9_Pin LD10_Pin LD8_Pin LD6_Pin */
	GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD7_Pin
			|LD9_Pin|LD10_Pin|LD8_Pin|LD6_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/* Retargets the C library printf function to the USART */
PUTCHAR_PROTOTYPE
{
	// Write a single character to UART4 and wait until transmission is complete
	//HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
	ITM_SendChar(ch);
	return ch;
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
