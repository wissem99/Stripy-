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

TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_tim1_ch1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define MAX_LED 29
#define USE_BRIGHTNESS 0
#define BlinkerLEDs 7

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];  // for brightness

int datasentflag=0;

int BlinkerSpeed = 200; //Blinker Running LED Speed. Adjust this to match with your Bike blinker speed.
int BlinkerOffDelay = 300; //Blinker Off time. Adjust this to match with your Bike blinker speed.


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
}

void Set_LED (int LEDnum, int Green, int Red, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

#define PI 3.14159265

void Set_Brightness (int brightness)  // 0-45
{
#if USE_BRIGHTNESS

	if (brightness > 45) brightness = 45;
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Mod[i][0] = LED_Data[i][0];
		for (int j=1; j<4; j++)
		{
			float angle = 90-brightness;  // in degrees
			angle = angle*PI / 180;  // in rad
			LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
		}
	}

#endif

}

uint16_t pwmData[(24*MAX_LED)+50];

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
#if USE_BRIGHTNESS
		color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
#else

		color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 80;  // 2/3 of 120
			}

			else pwmData[indx] = 30;  // 1/3 of 120

			indx++;
		}

	}

	for (int i=0; i<50; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void Reset_LED (void)
{
	for (int i=0; i<MAX_LED; i++)
	{
		LED_Data[i][0] = i;
		LED_Data[i][1] = 0;
		LED_Data[i][2] = 0;
		LED_Data[i][3] = 0;
	}
}




// functions
// brakeFull : all leds red with 100% Brightness : 36 leds

void BrakeFull()
{
  for (int i = 0; i < MAX_LED; i++)
  {
	  Set_LED (i, 0, 255 , 0);
  }
  WS2812_Send();
}

// BrakeMidlle : led 10 to led 36-10

void BrakeMiddle()
{
  for (int i = BlinkerLEDs; i < (MAX_LED - BlinkerLEDs); i++)
  {
    Set_LED(i,0,255,0);
  }
    WS2812_Send();
}

// ParkFull : all led are red with 25% Brightness

void ParkFull()
{
  for (int i = 0; i < MAX_LED; i++)
  {
	  Set_LED(i,0,255,0);
  }
      WS2812_Send();
}

// ParkMiddle : led 10 to 36 : red with 25%brightness

void ParkMiddle()
{
  for (int i = BlinkerLEDs; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED (i,0,60,0);
	   }
	       WS2812_Send();
	 }

// Left Functions

// Led for BlinkerLEDs-1 to 0 are Blinking with delay equal to BlinkerSpeed

void LeftBlinker()
{
  for (int i = (BlinkerLEDs-1); i >= 0; i--)
  {
	  Set_LED(i,100, 240, 0);
	  WS2812_Send();
	  HAL_Delay(BlinkerSpeed) ;

  }
}

//Led for 0 to BlinkerLEDs are reset to 0

void LeftDim()
{
  for (int i = 0; i < BlinkerLEDs; i++)
  {
	  Set_LED(i,0, 0, 0);
  }
  WS2812_Send();
}

// Led for 0 to (MAX_LED - BlinkerLEDs) are set to Red color with 30% Brightness

void LeftLit()
{
  for (int i = 0; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED(i,0, 150, 0);
  }
  WS2812_Send();
}

// Led for 0 to (MAX_LED - BlinkerLEDs) are set to Red color with full Brightness

void LeftFull()
{
  for (int i = 0; i < (MAX_LED - BlinkerLEDs); i++)
  {
	  Set_LED(i,0, 255, 0);
  }
  WS2812_Send();
}

// Right Functions

//Led for (MAX_LED - BlinkerLEDs) to MAX_LED are Blinking with delay equal to BlinkerSpeed

void RightBlinker()
{
  for (int i = (MAX_LED - BlinkerLEDs); i < MAX_LED; i++)
  {
	  Set_LED(i,100, 240, 0);
	  WS2812_Send();
      HAL_Delay(BlinkerSpeed);
  }
}

//Led for (MAX_LED - BlinkerLEDs) to MAX_LED are reset to 0

void RightDim()
{
   for (int i = (MAX_LED - BlinkerLEDs); i < MAX_LED; i++)
  {
	   Set_LED(i,0, 0, 0);
  }
   WS2812_Send();
}

// Led for BlinkerLEDs to MAX_LED are set to Red color with 30% Brightness

void RightLit()
{
  for (int i = BlinkerLEDs; i < MAX_LED; i++)
  {
	  Set_LED(i,0, 150, 0);
  }
  WS2812_Send();
}

// Led for BlinkerLEDs to MAX_LED are set to Red color with full Brightness

void RightFull()
{
  for (int i = BlinkerLEDs; i < MAX_LED; i++)
  {
	  Set_LED(i,0, 255, 0);
  }
  WS2812_Send();
}

// Dual blinking

void DualBlinker()
{
  for (int i = (BlinkerLEDs-1); i >= 0; i--)
  {
	  Set_LED(i,100, 240, 0);
	  Set_LED(MAX_LED-1-i,100, 240, 0);
	  WS2812_Send();
      HAL_Delay(BlinkerSpeed);
  }
}

void Left(){
	LeftDim();
	RightLit();
	LeftBlinker();
	LeftDim();
	HAL_Delay(BlinkerOffDelay);

}
void Right(){
	RightDim();
	LeftLit();
	RightBlinker();
	RightDim();
	HAL_Delay(BlinkerOffDelay);

}
void Left_Break(){
	LeftDim();
	RightFull();
	LeftBlinker();
	LeftDim();
	HAL_Delay(BlinkerOffDelay);

}
void Right_Break(){
	RightDim();
	LeftFull();
	RightBlinker();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
}
void Dual(){
	LeftDim();
	RightDim();
	ParkMiddle();
	DualBlinker();
	LeftDim();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
}
void Dual_Break(){
	LeftDim();
	RightDim();
	BrakeMiddle();
	DualBlinker();
	LeftDim();
	RightDim();
	HAL_Delay(BlinkerOffDelay);
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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  /*for (int i = 0; i < 21; i++)
  	  	 	  	   { Set_LED(i,0, 255, 0);
  		 			//Set_LED(i-1,0, 0, 0);

  		 	  	 	  }*/

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  for (int i = 0; i < 7; i++) {
	 	 		Set_LED(i,100, 240, 0);
	 	 		WS2812_Send();
	 	 	}

	 	 for (int i = 0; i < 7; i++)
	 	  	   {
	 		 for(int j=0 ; j<150; j++){
	 			 Set_LED(i,0, 0, 0);
	 			//Set_LED(i-1,0, 0, 0);
	 	  	 	  WS2812_Send();
	 	  	 	  }
	 	  	   }

	 	/*for (int i = 0; i < 7; i++)
	 		 	  	   {
	 		 	  	 	  Set_LED(i,0, 0, 0);
	 		 	  	   }*/
	 	// HAL_Delay(1000);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 90-1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

