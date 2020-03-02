/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "math.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

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
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define resolution 221.1f
#define presicion 4
#define PI 3.1415f
#define RayonR 62.5f
#define RayonL 62.5f
#define entraxe 160.0f
#define coefK 200
#define TIMEINTERVAL 0.01
#define KV 350
#define KI 0.00009
#define KD 5000

int enter =0;
int CountEntry = 0;
unsigned int TicksRightNow = 0;
unsigned int TicksLeftNow = 0;
unsigned int TicksRightPrev = 0;
unsigned int TicksLeftPrev = 0;
float	DistR =0.0f;
float DistL =0.0f;
int DiffR = 0.0;
int DiffL= 0.0;
float dR = 0.0f;
float dL = 0.0f;
float dC = 0.0f;
double X = 0.0;
double Y = 0.0;
double PHI = 0.0;
float TargetDistance;
float CrtlR;
float CrtlL;
float PMWbaseR=1000;
float PMWbaseL=1000;
float PMWR;
float PMWL;
float erreur;
float PMWRMAX=3000;
float PMWLMAX=3000;
float PMWRMIN=1000;
float PMWLMIN=1000;
char s[20];
char ExtInfo1[20];
char ExtInfo2[20];
char ExtInfo3[20];
float VelocityRight ;
float VelocityLeft;
float DfVR;
float DfVL;
float Rv;
float Lv;
float Vmin = 0.05;
float Vmax = 0.25;
float VLeft;
float VRight;
float distnowR;
float distnowL;
float distprevR;
float distprevL;
float erreurIR;
float erreurIL;
int testV = 0;
float erreurPrevDR =0;
float erreurNowR;
float erreurPrevDL=0;
float erreurNowL;
float difDerivaR;
float difDerivaL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void initEncoder(void);
float tickstoDistance(int,float);
void CalculationFunction(void);
void initMotors(void);
void RunForward(int,int);
void RunBackward(int,int);
void RunToGoal(int,int);
void Rotate(int,int);
void stopp(void);
void VelocityControl(int,int);
void VelocityAsserv(float,float);
void Trapezy(float);
void Move(int,int,int);
int checkStop(void);
int CompareString(uint8_t*,char*,int);
void DataConverting(uint8_t*);
void ExtractInfo(uint8_t* ,int );
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)	
		{
			if (htim->Instance==TIM7) //check if the interrupt comes from TIM7
				{
					CalculationFunction();
					CountEntry++;
				}
				
						
	}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t data[20]="75.5 66.6 90.0\n";
uint8_t dataIn[20];
char  DataOut[20];
uint8_t	TestIn [20];
int result = 0;
float Tab,Tab2;
	
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
   HAL_TIM_Base_Start_IT(&htim7);
	 initEncoder();
	 initMotors();
	 stopp();
	 TestIn[0] = 49;
	 TestIn[1] = 46;
	 TestIn[2] = 51;
	 TestIn[3] = 97;
	 TestIn[4] = 52;
	 TestIn[5] = 53;
	 TestIn[6] = 54;
	 TestIn[7] = 97;
	 /*Test Motor Direction 
	 
	 Rotate(1500,1);
	 RunForward(1500,1500);
	 HAL_Delay(500);
	 stopp();
	 */
	 //Rotate(2000,-1);
	 //RunBackward(2000,2000);
	 //RunForward(2000,2000);
	 //RunToGoal(-2000,2300);
	//HAL_Delay(1000);
	 //Move(1500,2000,2000);
	 //stopp();
	VelocityAsserv(0.0001,0.00005);
	 // Test Moving Distance
	 /*while (DistR<300)
	 {
		 RunToGoal(2000,2000);
	 }
	 stopp();
	 */
	 ExtractInfo(TestIn,9);
	// sprintf(s,"123");
	 Tab = atof(ExtInfo1);
	 Tab2 = atof(ExtInfo2);
  /* USER CODE END 2 */
 
 

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		
		/* Communication Test
		HAL_UART_Receive(&huart2,(uint8_t*)dataIn,10,100);
		result = CompareString(dataIn,"abc",4);
		if( result)
		{
			HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_5);
		HAL_Delay(100);
		}
		

		//HAL_UART_Transmit(&huart2,data,20,1000);
		*/
		
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 4199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 999;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void initEncoder()
{
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1);
	TIM2->CNT = 0;
	TIM3->CNT = 0;

}
void RunForward(int VR,int VL)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,VL);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
	
}
void RunBackward(int VR,int VL)
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,VL);
	
}


void Rotate(int VR,int Sens)
{		if (Sens>0){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
			}
else{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
}
	
	
}
void RunToGoal(int VR,int VL)
{
	
	if ((VR>0)&&(VL>0)){
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,VL);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
			}
else if ((VR<0)&&(VL>0)) {
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,-VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,VL);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
}
else if ((VR>0)&&(VL<0)) {
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,-VL);
}

else if ((VR<0)&&(VL<0)) {
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,-VR);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,-VL);
}

}

void stopp()
{
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,0);
	__HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,0);
}
void initMotors()
{
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
	stopp();
	
}
float tickstoDistance(int ticks,float R)
{
	return((PI*R*ticks)/(resolution*presicion));
}

void CalculationFunction()
{
	
			TicksRightNow = TIM2->CNT ;
			TicksLeftNow = TIM3->CNT ;
	
			//DistR = tickstoDistance(TicksRightNow,RayonR);
			//DistL = tickstoDistance(TicksLeftNow,RayonL);
	
			DiffR = (TicksRightNow - TicksRightPrev);
			DiffL =(TicksLeftNow - TicksLeftPrev);
			TicksRightPrev = TicksRightNow;
			TicksLeftPrev  = TicksLeftNow;
	
			if (DiffR > 50000) 
			{
			   DiffR = DiffR - 65535 ;
			}
			else if (DiffR < -50000) 
			{
			   DiffR = DiffR + 65535 ;
			}
			
			if (DiffL > 50000) 
			{
			   DiffL = DiffL - 65535 ;
			}
			else if (DiffL < -50000) 
			{
			   DiffL = DiffL + 65535 ;
			}
			
			dR = tickstoDistance(DiffR,RayonR);
			dL = tickstoDistance(DiffL,RayonL);
			dC = (dR + dL)/2;
			
			DistR += dR;
			DistL += dL;
			if (CountEntry >1) 
			{
				distnowR = DistR;
				distnowL = DistL;
			VelocityRight = ((distnowR - distprevR) * 0.001) / (1000*TIMEINTERVAL) ;
			VelocityLeft =  ((distnowL - distprevL) * 0.001) / (1000*TIMEINTERVAL) ;
				CountEntry = 0;
				distprevR = DistR;
				distprevL = DistL;
				enter++;
			}
			// odometry :
			X += dC*cos(PHI);
			Y += dC*sin(PHI);
			PHI += ((dR-dL)/entraxe);
			
			while (PHI>PI)
			{
				PHI -= 2*PI;
			}
			while (PHI<-PI)
			{
				PHI += 2*PI;
			}
	
}

void VelocityControl(int VR,int VL)
{
	if (VR>PMWRMAX)  PMWR = PMWRMAX;
	if (VL>PMWLMAX)  PMWL = PMWLMAX;
	if (VR<PMWRMIN)  PMWR = PMWRMIN;
	if (VL<PMWLMIN)  PMWL = PMWLMIN;
	
	
	
}
int checkStop()
{
	if ((DistR<TargetDistance-5) || (DistL<TargetDistance-5)) return (1);
	else if ((DistR>TargetDistance+5) || (DistL>TargetDistance+5)) return (1);
	else  return (0);
	
}
int signe(float x) 
{
	if (x>0) return (1);
	if (x<0) return (-1);
	else return (0);
}


void Trapezy(float Dist)
{
	float DC = (DistR + DistL)/2.0f ;
	if (DC<(0.5*Dist)) {VLeft = ((Vmax - Vmin)/(0.5*Dist)) * DC;VRight = ((Vmax - Vmin)/(0.5*Dist)) * DC;}
	//else if (DistR>(0.7*Dist)) {VLeft -= (Vmax/(0.3*Dist)) * DC;VRight -= (Vmax/(0.3*Dist)) * DC;}
	else {VLeft = Vmax;VRight = Vmax;}
	
	if (VLeft > Vmax) VLeft = Vmax;
	if (VRight > Vmax) VRight = Vmax;

	
}
void VelocityAsserv(float V1 ,float V2 )
{
	//PMWbaseR*=signe(PMWbaseR);
	//PMWbaseL*=signe(PMWbaseL);
	PMWR = PMWbaseR;
	PMWL = PMWbaseL;

		RunToGoal(PMWbaseR,PMWbaseL);
	  while (1)//(DistR < 1000)
		{ //Trapezy(500);
			DfVR =  V1-VelocityRight  ;
			DfVL = V2-VelocityLeft ;
			if (DfVR>10) testV = 12;
			else testV = 1;
			if (DfVL>10) testV=12 ;
			else testV=1;
			erreurIR += DfVR;
			erreurIL += DfVL;
			difDerivaR =  erreurNowR-erreurPrevDR ;
			difDerivaL =  erreurNowL-erreurPrevDL ;
			Rv = KV*DfVR + KI * erreurIR +KD * difDerivaR;
			Lv = KV*DfVL + KI * erreurIL +KD * difDerivaL;
			
			PMWR = PMWR + Rv ; 
			PMWL = PMWL + Lv ;
			VelocityControl(PMWR,PMWL);
			RunToGoal(PMWR,PMWL);
			
		}
		stopp();

	
}
void Move(int Distance,int VR,int VL)
{
	TargetDistance = Distance;
	while ( checkStop())
	{
		erreur = DistR-DistL ;
		CrtlR = -coefK*erreur ;
		CrtlL = -CrtlR;
		
		PMWR = PMWbaseR+CrtlR;
		PMWL = PMWbaseL+CrtlL;
		
		//VelocityControl(PMWR,PMWL);
		
		RunToGoal(PMWR,PMWL);
	}
	stopp();
}

int CompareString(uint8_t* Data,char* ch,int length)
{
	DataConverting(Data);
	 for(int i =0 ;i<length;i++)  if (DataOut[i] != ch[i]) return (0) ;
	return(1);
}
void DataConverting(uint8_t* Data)
{
	
		for (int i =0;i<10;i++)
		{
			DataOut[i]=(char)Data[i];
		}
		
}
void ExtractInfo(uint8_t* Data,int length)
{
	DataConverting(Data);
	int offsetInf = 0;
	int offsetSup = 0;
	int index = 0;
	int k =0;
	for(int i=0;i<length;i++)
	{
		if (DataOut[i] =='a')
		{
			
						switch (index)
						{
							case 0 :
							{
								for (int j = offsetInf;j<offsetSup;j++) 
									{
										ExtInfo1[k] = DataOut[j];
										k++;
									}
									k=0;
									offsetInf = offsetSup+1;
									index++;
									break;
							}
							case 1 :
							{
								for (int j = offsetInf;j<offsetSup;j++) 
									{
										ExtInfo2[k] = DataOut[j];
										k++;
									}
									k=0;
									offsetInf = offsetSup;
									index++;
									break;
							}
							case 2 :
							{
								for (int j = offsetInf;j<offsetSup;j++) 
									{
										ExtInfo3[k] = DataOut[j];
										k++;
									}
									k=0;
									offsetInf = offsetSup;
									index++;
									break;
							}
						}
			offsetSup+=1;
		}
		else 
			offsetSup++;
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
