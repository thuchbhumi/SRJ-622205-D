/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EndeffAddress (0x23<<1)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

uint8_t ButtonArray[2] 	= {1,1};  //[Now, Last] = {UP, UP}
uint64_t _micros 		= 0;
uint64_t Timestamp 		= 0;
uint64_t LEDTimeStamp 	= 0;
uint8_t ProxiArray[2]   = {1,1};
int   PWMOut 			= 0;
int32_t PWMOutABS        =0;

float RobotArm_Position = 0;
float RobotArm_Velocity = 0;
float Desired_Position 	= 0;
float Desired_Velocity 	= 0;
uint8_t RobotArmTrigger = 0;

float Error_Position  	= 0;
float Error_Position_Sum= 0;
float Error_Position_Div= 0;
float Error_Position_Old= 0;
float dt    = 0.001;
float K_P	= 1200;
float K_I	= 650;
float K_D	= 0;

//tee
float a=8;float w=1;
float rad=0;float raw=0;
float Q=0;float R=0;float theta_est=0;float omega_est=0;float dt1=0.01;float theta_pd=0;float y=0;float p11=0;
float p12=0;float p21=0;float p22=0;float omega_pd=0;float a0=0;float a1=0;float a2=0;float a3=0;float a4=0;float a5=0;
float sb=0;float sa=0;float tf=0;float vb=0;float sbf=0;float t=0;float Vmax=0;float vcon =0;
float p=0;
float i=0;
float d=0;
float pre_error=0;float error=0;
uint8_t push=0;float n=0;float angle=0;float kalman_theta=0;float rb_pos=0;

typedef enum
{
	Set_Home,
	UART_Commu,
	Traj_Plan,
	Arm_Move,
	I2C_Endeff
}Transition_State;

uint8_t LaserOpenTrigger = 0;
uint8_t LaserOpenCommand = 0x45;
uint8_t LaserReadCommand = 0x23;
uint8_t LaserStatus = 0x78;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */

void NucleoCheck();
uint64_t micros();
void MicroCheck();
void ProxiCheck();
void MotorDrive();
void SetHome();
uint32_t EncoderPosition_Update();
void Control();
void I2C_Laser();
void I2C_Check();
float EncoderVelocity_Update();
void pid();
void kalmanfilter();
void planning();
void togo();
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
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

  PWMOut = 0;
  MotorDrive();
  HAL_Delay(100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  /////////////////////////////////////////////////////////////////////////////////////
  while (1)
  {
	  if(push==1)
	  		{push=0;
	  		HAL_TIM_Base_Start_IT(&htim4);}


	  if (micros() - Timestamp >= 1000) //1000us = 0.001s = 1kHz
	  {
		  Timestamp = micros();
		  NucleoCheck();
//		  MicroCheck();
//		  ProxiCheck();
//		  I2C_Check();


		  RobotArm_Position = EncoderPosition_Update();
		  rb_pos = (float)(RobotArm_Position*360.00/7200.00);

		  static Transition_State State = Set_Home;

		  switch(State)
		  {
		  	  case Set_Home:
		  		  PWMOut = 2000;
		  		  MotorDrive();
		  		  HAL_Delay(2000);
		  		  SetHome();
		  		  State = UART_Commu;
		  		  break;

		  	  case UART_Commu:
		  		  if(Desired_Position != RobotArm_Position && RobotArmTrigger != 0)
		  		  {
		  			  State = Traj_Plan;
		  		  }
		  		  break;

		  	  case Traj_Plan:
		  		  State = Arm_Move;
		  		  break;

		  	  case Arm_Move:
		  		  if(Desired_Position == RobotArm_Position)
		  		  {
		  			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 10000);
		  			  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 10000);
		  			  HAL_Delay(500);

		  			  PWMOut = 0;
		  			  MotorDrive();
		  			  HAL_Delay(500);

		  			  RobotArmTrigger = 0;

		  			  Error_Position  	= 0;
		  			  Error_Position_Sum= 0;
		  			  Error_Position_Div= 0;
		  			  Error_Position_Old= 0;

		  			  State = I2C_Endeff;
		  		  }
		  		  else
		  		  {
		  			  Control();
		  			  MotorDrive();
		  		  }
		  		  break;

		  	  case I2C_Endeff:
		  		  I2C_Laser();
		  		  State = UART_Commu;
		  		  break;
		  	  default:
		  		  break;
			}
	  }


/////////////////////////////////////////////////////////////////////////////////////
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 7199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 10000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  huart2.Init.BaudRate = 115200;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//********************************************************************************

void NucleoCheck()
{
	ButtonArray[1] = ButtonArray[0];
	ButtonArray[0] = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

	if(ButtonArray[0]==1 && ButtonArray[1]==0) //When Released Button
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}
#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 3600
#define  MAX_ENCODER_PERIOD 7200

uint32_t EncoderPosition_Update()
{
	return HTIM_ENCODER.Instance->CNT;
}
float EncoderVelocity_Update()
{   static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();
	int32_t EncoderPositionDiff;
	uint64_t EncoderTimeDiff;
	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;
	if (EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{EncoderPositionDiff -= MAX_ENCODER_PERIOD;}
	else if (-EncoderPositionDiff >= MAX_SUBPOSITION_OVERFLOW)
	{EncoderPositionDiff += MAX_ENCODER_PERIOD;}
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;
	//raw =(float)(EncoderPositionDiff * 1000000.00*60.00/3072.00) / (float) EncoderTimeDiff;
	raw =(float)(EncoderPositionDiff * 1000000.00) / (float) EncoderTimeDiff;
	rad = raw* 0.05*2.00*3.141592/360.00;
	//rad = (float)(0.10472*raw);
	return  rad;
}


void kalmanfilter()
{    Q = pow(a,2);
	 R = pow(w,2);
	 theta_est = theta_pd + omega_pd*dt1;
	 omega_est = omega_pd;
	 y = (rad-omega_est);

    p11 = p11 + dt1*p21+ (Q*pow(dt1,4))/4 + (pow(dt1,2))*(p12+dt1*p22)/dt1;
    p12 = p12 + dt1*p22 + (Q*dt1*pow(dt1,2))/2;
    p21 = (2*dt1*p21+Q*pow(dt1,4) + 2*p22*pow(dt1,2))/(2*dt1);
    p22 = Q*pow(dt1,2)+p22;

    theta_est+= (p12*y)/(R+p22);
    omega_est+= (p22*y)/(R+p22);

    p11=p11-(p12*p21)/(R+p22);
    p12=p12-(p22*p21)/(R+p22);
    p21=-p21*(p22/(R+p22)-1);
    p22=-p22*(p22/(R+p22)-1);

    theta_pd=theta_est;
    omega_pd=omega_est;

    kalman_theta=(float)(theta_est*57.2958);
}
void togo()
{ //use for calculate station A->B
}

void planning()

{ t=t+0.01;
  Vmax = 0.400;               //rad/s
  sb=angle*0.0174533;            //degree 2 rad
  sa=0;

  tf = 15.00*(sb-sa)/(8.00*Vmax);     //get tf from vmax
  if(0.5>=(5.7335*(sb-sa)/(pow(tf,2))))  //check accerelation
  {tf=tf;}
  else{tf=pow((5.7335*(sb-sa)/0.5),0.5);}
  a0=0;
  a1=0;
  a2=0;
  a3= 10.00*(sb-sa)/(pow(tf,3));
  a4= -15.00*(sb-sa)/(pow(tf,4));
  a5= 6.00*(sb-sa)/(pow(tf,5));
  //sbf =  a3*pow(tf,3)+a4*pow(tf,4)+a5*pow(tf,5);
  if(t<=tf){
  sbf =  a3*pow(t,3)+a4*pow(t,4)+a5*pow(t,5);
  vb= (float)((3*a3*pow(t,2))+(4*a4*pow(t,3))+(5*a5*pow(t,4)));}
  //vcon = 0.10472*vb;}
  else{t=tf;vb=0;PWMOut=0;}
  //HAL_TIM_Base_Stop_IT(&htim4);}
}

void pid()
{
 error = vb-omega_est;
 p = (error);
 i = i+error;
 d = error - pre_error;
 pre_error = error;
 PWMOut =200+( (p*K_P)+(i*K_I)+(d*K_D));
 if(vb==0)
 {PWMOut=0;}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
	if (htim==&htim4)
		{
		  EncoderVelocity_Update();
		  //togo();
		  planning();
		  kalmanfilter();
		  pid();
		  MotorDrive();
		}
}

uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
}

void MicroCheck()
{
	if (micros() - LEDTimeStamp >= 1000000) //1000000us = 1s = 1Hz for Half
	{
		LEDTimeStamp = micros();
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	}
}

void ProxiCheck()
{
	ProxiArray[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5);
	ProxiArray[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
}

void MotorDrive()
{
	if(PWMOut >= 0)
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, PWMOut);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -PWMOut);
	}
}

void SetHome()
{
	PWMOut = -6500;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == 1)
	{
		MotorDrive();
	}

	PWMOut = -500;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5) == 1)
	{
		MotorDrive();
	}

	MotorDrive();
	HAL_Delay(200);

	PWMOut = 0;
	MotorDrive();
	HAL_Delay(500);

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

void Control()
{
	Error_Position = Desired_Position - RobotArm_Position;
	Error_Position_Sum += (Error_Position * dt);
	Error_Position_Div = (Error_Position - Error_Position_Old) / dt;

	PWMOut = (K_P * Error_Position) + (K_I * Error_Position_Sum) + (K_D * Error_Position_Div);

	Error_Position_Old = Error_Position;
}

void I2C_Laser()
{
	HAL_I2C_Master_Transmit(&hi2c1, EndeffAddress, &LaserOpenCommand, 1, 500);
	for (int j=0; j<11; j++)
	{
		HAL_I2C_Master_Transmit(&hi2c1, EndeffAddress, &LaserReadCommand, 1, 500);
		HAL_I2C_Master_Receive(&hi2c1, EndeffAddress, &LaserStatus, 1, 500);
		HAL_Delay(500);
	}
}

void I2C_Check()
{
	if(LaserOpenTrigger == 1)
	{
		I2C_Laser();
		LaserOpenTrigger = 0;
	}
}

//********************************************************************************
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
