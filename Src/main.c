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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define mpu_add 0xD0
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

//MPU
uint8_t trans_data[6];
uint8_t high , low ;
uint8_t rdata_i2c[2];
int16_t data_i2c;
float ax , ay , az , pitch , roll;
uint8_t check =0;

//SPI 
uint8_t o_char = 0x7E;
uint8_t e_char = 0b01001111;
uint8_t p_char = 0x67;
uint8_t n_char = 0x76;
uint8_t c_char = 0x4E;
uint8_t s_char = 0x5B;
uint8_t l_char = 0xE;
uint8_t gach = 0x01;
uint8_t a_char = 0b01110111;




// password
char password[]="123";
char user_key[3] ;
uint8_t count =0;
uint8_t state = 0 ;
uint8_t isopen =0;
// count time to 5s
uint16_t count_time =0 ;


// uart bluetooth
uint8_t warming_buff[] = "Warming : YOU FORGET TO CLOSE A DOOR  !!!\n";
uint8_t requird_pass [] = "PassWord :";
uint8_t ch =0x13;
uint8_t RxData[100],rxData;
uint8_t index =0;
uint8_t flag =0 ;
uint8_t open[] = "open";
uint8_t close [] = "close";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

////// Count Time to 5s ///////////
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
 if(htim->Instance == htim3.Instance)
 {
   count_time++; // count_time = 250 <==> time from 0s to 5s 
 }
}

///// interput uart
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART2)
	{
    if(rxData!=13) 
    {
    	RxData[index++] = rxData;
    }
    else if (rxData==13) 
    {
    	index = 0 ;
			flag = 1 ;
    }
		
	}
  HAL_UART_Receive_IT(&huart2,&rxData,1); // Enabling interrupt receive again
}


//////// MPU_I2C peripherals   /////////

void CHeck_I2C(){
	HAL_I2C_Mem_Read(&hi2c1,mpu_add,0x75,1,&check,1,1000);
}

void MPU6050_Init()
{
	// sample rate 0x19
	trans_data[0]=0x07;
	// config 0x1A
	trans_data[1]=0x00;
	// GYRO_CONFIG 0x1B	
	trans_data[2]=0x08;
	// ACCEL_CONFIG 0x1C
	trans_data[3]=0x10;
	// interrupt
	trans_data[4]=0x09;
	// clock and pwr 
	trans_data[5]=0x00;
	
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x19,I2C_MEMADD_SIZE_8BIT,&trans_data[0],1,1000);
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x1A,I2C_MEMADD_SIZE_8BIT,&trans_data[1],1,1000);
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x1B,I2C_MEMADD_SIZE_8BIT,&trans_data[2],1,1000);
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x1C,I2C_MEMADD_SIZE_8BIT,&trans_data[3],1,1000);
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x38,I2C_MEMADD_SIZE_8BIT,&trans_data[4],1,1000);
	HAL_I2C_Mem_Write(&hi2c1,mpu_add,0x6B,I2C_MEMADD_SIZE_8BIT,&trans_data[5],1,1000);
	
}

int16_t ReadRegis16bits(uint8_t add)
{
	 
	
	HAL_I2C_Mem_Read(&hi2c1,mpu_add,add,I2C_MEMADD_SIZE_8BIT,&high,1,1000);
	HAL_I2C_Mem_Read(&hi2c1,mpu_add,add+1,I2C_MEMADD_SIZE_8BIT,&low,1,1000);
	data_i2c = (int16_t)(high<<8)|low;	
	return data_i2c;
}

void ReadDataAcc()
{
	
	ax = (float)ReadRegis16bits(0x3B)/4096.0;
	ay = (float)ReadRegis16bits(0x3D)/4096.0;
	az = (float)ReadRegis16bits(0x3F)/4096.0;
	
	pitch = atan2(-ax, sqrt(pow(ay,2)+pow(az,2)))*180.0/3.14;
	roll = atan2(ay, sqrt(pow(ax,2)+pow(az,2)))*180.0/3.14;
}



////////////////////// SPI /////////////////////

void sendData_SPI(uint8_t add , uint8_t data)
{
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,&add,1,100);
	HAL_SPI_Transmit(&hspi2,&data,1,100);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET);
}


void MAX7219_Init()
{

	// shutdown format 
	sendData_SPI(0x0C,0x01);

	// decode mode 
	sendData_SPI(0x09,0x00);
	
	// Intensity Register Forma	
	sendData_SPI(0x0A,0x09);
	
	//Scan-Limit Register Format
	sendData_SPI(0x0B,0x07);
	
	// Display-Test Register Format 
	sendData_SPI(0x0F,0x00);
	
}

void DisplayClose()
{
	sendData_SPI(0x0B,3);
	sendData_SPI(4,c_char);
	sendData_SPI(3,0b01111110);
	sendData_SPI(2,l_char);
	sendData_SPI(1,s_char);
}

void DisplayOpen()
{
	sendData_SPI(0x0B,3);
	sendData_SPI(1,n_char);
	sendData_SPI(2,e_char);
	sendData_SPI(3,p_char);
	sendData_SPI(4,o_char);
}

void DisplayPassWordClear()
{
	sendData_SPI(0x0B,7);
	sendData_SPI(8,p_char);
	sendData_SPI(7,a_char);
	sendData_SPI(6,s_char);
	sendData_SPI(5,s_char);
	sendData_SPI(4,0b10000000);
	sendData_SPI(3,0b10000000);
	sendData_SPI(2,0b10000000);
	sendData_SPI(1,0b10000000);

}

void DisplayPassWord(int c)
{
	sendData_SPI(0x0B,7);
	sendData_SPI(8,p_char);
	sendData_SPI(7,a_char);
	sendData_SPI(6,s_char);
	sendData_SPI(5,s_char);
	
	
	if (c == 1)
	{
		sendData_SPI(3,gach);
	}
	else if( c==2)
	{
		sendData_SPI(3,gach);
		sendData_SPI(2,gach);
	}
	else if (c==3)
	{
		sendData_SPI(3,gach);
		sendData_SPI(2,gach);
		sendData_SPI(1,gach);
	}
	
}


//////// SerVo ///////////////////
void SerVo_Open(){
	
	for(int i =75 ;i<=125 ;i++)
	{	
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(100);
	}
}

void SerVo_Close(){
	for(int i =125 ;i>=75 ;i--)
	{	
		__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, i);
		HAL_Delay(100);
	}
}


////////Sound ////////
void Sound(){
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);
	
}

////// PassWord ////////////
void CheckPassWord(){
	
	
	// type password if door close 
	while( count<3 && state == 0)
	{	
		
		
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==SET){
				DisplayPassWord(count+1);
				user_key[count++] = '1' ;
				Sound();
		}
		else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_3)==SET){
				DisplayPassWord(count+1);
				user_key[count++] = '2' ;
				Sound();
		}
		else if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)==SET){
				DisplayPassWord(count+1);
				user_key[count++] = '3' ;
				Sound();
		}
		
		if (count == 3)
		{
			
			
			if ( strcmp(user_key,password)==0)
			{
				count_time = 0;
				state = 1;
				SerVo_Open();
				DisplayOpen();
				
			}
			else
			{ 
				count=0;
				DisplayPassWordClear();
			}

		}
	}
	
	// check  if door still open in 10s 	
	
	if(state == 1)
	{
		int count_warming =0 ;
		// check if door open	over 10s	
		if(count_time >=800 ) // 
		{				
			if (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==SET) // still open
			{
				
				while (count_warming ++ < 10)
				{
					HAL_UART_Transmit(&huart3,warming_buff,sizeof(warming_buff)-1,1000);
					Sound();
					
					if (strcmp((char*)RxData,(char*)open)==0)
					{	
						
						DisplayOpen();
						count =0 ;
						state = 0;
						HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
						rxData = 0;
						memset(RxData,0,sizeof(RxData));
						isopen = 1;
						break ;
					}
					else if( strcmp( (char*)RxData,(char*)close)==0)
					{
						SerVo_Close();
						DisplayClose();
						count =0 ;
						DisplayPassWordClear();
						state = 0;
						rxData = 0;
						rxData = 0;
						memset(RxData,0,sizeof(RxData));
						break;
					}
					HAL_Delay(1000);
				}
					
			}
			
		}
		
		
	}
	
	if(isopen == 1 && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==SET)
	{
			SerVo_Close();
			DisplayClose();
			count =0 ;
			DisplayPassWordClear();
			state = 0;
			rxData = ' ';
			isopen = 0;
	}
	
	if(strcmp((char*)RxData,(char*)open)==0 && state ==0 )
	{
		HAL_UART_Transmit(&huart3,requird_pass,sizeof(requird_pass)-1,1000);
		memset(RxData,0,sizeof(RxData));
		
		if ( strcmp(user_key,password)==0)
			{
				count_time = 0;
				state = 1;
				SerVo_Open();
				DisplayOpen();
				
			}
		else
			{ 
				count=0;
				DisplayPassWordClear();
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
  MX_I2C1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_SPI2_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	
	//Servo peripherals
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(&htim3);
	SerVo_Close();
	
	// MPU_I2C  peripherals
	CHeck_I2C();
	if(check == 104){
		MPU6050_Init();
	}
	
	// SPI
	MAX7219_Init();
	DisplayPassWordClear();
	
	//UART 
	HAL_UART_Receive_IT(&huart2,&rxData,1);
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		CheckPassWord();
		//ReadDataAcc();
			
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
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 159;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 159;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB12 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB3 PB4 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
