/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2015 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */

#include "math.h"

#define Wn 0.98f 						// natural frequncy

#define M_PI 3.142857142857143f
#define Deg2Rad 0.0174603174603175f

#define sampling 200.0f
#define Revo_Per_Step 0.25f
#define limit_angle 5.0f

#define step_clock  2000000.0f
#define step_Per_mm 50.0f      			// step per mm 6.25, 12.5, 25, 50, 100 (full - 1/16)

#define max_acc  2500.0f     			// mm/s^2
#define max_velo 700.0f   				// mm/s
#define max_displacement 380.0f  		// mm

#define acc_swing_up     1200.0f  		// mm

#define start_cart_position -190.0f * step_Per_mm * 2.0f 

#define start_swing_upPoint  0

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

float Debug;

double time = 0;
uint8_t block = 0;									// block swing-up direction													
uint8_t Mode = 0;								// operation mode 
uint8_t inte_cart_enable = 0;					// enable or disable intregetion
float Angle_pen = 180;
float Angle_pen_dot = 0;

int32_t raw_position_cart = start_cart_position;
float position_cart = 0;
float position_cart_velo = 0;
float position_cart_acc = 0;

float energy = 0;

static float y_data, y_prev_1, y_prev_2;
static float x_prev_1, x_prev_2;

int8_t _limit_state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void Initial_encoder(void);
void Update_encoder(void);
float Moving_average(float new_data, float old_data);
float Butterworth_filter(float new_data);

void Sampling_update(void);
void Motor_enable(void);
void Motor_disable(void);
void Motor_lock(void);
void Motor_drive(float tmp_acc, float velo_limit);

void Idle(void);
void Homing(void);
void pre_Swing_up(void);
void Swing_up(void);
void stabilizer(void);

void _drive_step_pin(void);

void blink_green(void);
void blink_red(void);
void Limit_ws_check(void);
void Limit_ws_unlock(void);
void reset_filter(void);
void Print_Debug(void);
void test(void);

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
	position_cart = raw_position_cart / (step_Per_mm *2.0f);	
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM3_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */


	Initial_encoder();
	Motor_disable();
	HAL_Delay(100);
	

	
	Motor_enable();
	//Motor_lock();
	
	Mode = 1 ; 													// homing 
	HAL_TIM_Base_Start_IT(&htim14);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

		HAL_Delay(0xffff);
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0xffff;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  HAL_TIM_Encoder_Init(&htim3, &sConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* TIM14 init function */
void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 4999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim14);

}

/* TIM16 init function */
void MX_TIM16_Init(void)
{

  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 23;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 0;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim16);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONEBIT_SAMPLING_DISABLED ;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  HAL_UART_Init(&huart1);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void) 
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOF_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pins : PF0 PF1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void Sampling_update (void)
{
	time += 0.005;
	Limit_ws_check();
	
	Update_encoder();
	
	if (Mode == 1) Homing();	
	if (Mode == 2) pre_Swing_up();
	if (Mode == 3) Swing_up();	
	if (Mode == 4) stabilizer();	
	if (Mode == 5) test();	
}

// mode 0
void Idle(void)									// free 
{
	Motor_disable();
	Angle_pen = 0;
	Angle_pen_dot = 0;

	position_cart = 0;
	position_cart_velo = 0;
	position_cart_acc = 0;
}

// mode 1
void Homing(void)								// go to zero  
{
	static float tmp;
	
	blink_green();  
	
	if (_limit_state != 1 && position_cart < start_swing_upPoint)
	{
		inte_cart_enable = 0;
		Motor_drive(-100, 100); 			// homing
	}else{
		  inte_cart_enable = 1;
			if (position_cart < start_swing_upPoint)
			{
				Motor_enable();						// enable motor
				Motor_drive(100, 100);
			}else{
				Limit_ws_unlock();				// unlock limit 
				Motor_lock();
				
				tmp = Moving_average(Angle_pen_dot, tmp);
				if (tmp < 0.2f && tmp > -0.2f)
				{
					time = 0;							// reset time
					Mode = 2;									// goto mode 2 Swing_up
					Motor_enable();						// enable motor
					reset_filter();
					position_cart_velo = 0;
				}
			}
	}
	block = 1;
}

// mode 2
void pre_Swing_up(void)							// Feed forward
{
	blink_red();
	float force = -900.0f * cosf ( time * M_PI * 2.0f / Wn ) ;
	Motor_drive(force, max_velo);
	Debug =  force;
	
	if (Angle_pen > 220 || Angle_pen < 135) Mode = 3 ;	 
}

// mode 3
void Swing_up(void)							// energy control
{

	float poten = cosf(Angle_pen*Deg2Rad);
	float kine  = fabsf(Angle_pen_dot);
	float bangbang ;
	
	blink_red();
	
	energy =  poten + kine;  // mgh + (1/2)mv^2
	bangbang = poten * Angle_pen_dot;//* Angle_pen_dot ;//* kine;
	
//	if (block == 0)
	{
		if (bangbang > 0.005f || bangbang < -0.005f)
		{
				if (bangbang > 0)
				{
					Motor_drive(acc_swing_up, max_velo); 
				}else{
					Motor_drive( -acc_swing_up, max_velo);
				}
		}else{
				Motor_drive(0 , max_velo);
		}
	}
	
//	if (Angle_pen > 360 || Angle_pen < 0) Mode = 4;
}

// mode 4
void stabilizer(void)						// balencing
{
	blink_green();  
	blink_red();
}
			
void test(void)						// balencing
{
	float force = -2000.0f * cosf ( time * M_PI * 1.0f *( 1.0f + time/20.0f) ) ;
	Motor_drive(force, max_velo);
	Debug =  force;
}

void Initial_encoder(void)
{
	TIM_Encoder_InitTypeDef hEncoder;
	hEncoder.EncoderMode = TIM_ENCODERMODE_TI12 ;
	HAL_TIM_Encoder_Init(&htim3,&hEncoder);
	HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
	TIM3->CNT = 0x3fff ;
}

void Update_encoder(void)
{
	float prev_angle = Angle_pen;
	int16_t tmp;

	tmp = TIM3->CNT - 0x3fff ;
	TIM3->CNT = 0x3fff ;
	
	Angle_pen += (float)tmp * Revo_Per_Step;
	Angle_pen_dot = Butterworth_filter((Angle_pen - prev_angle) * sampling);
//	Debug = (Angle_pen - prev_angle) * sampling;
}

float Moving_average(float new_data, float old_data)
{
	float tmp =0.9f*old_data + 0.2f*(new_data - old_data);
	return tmp;
}
float Butterworth_filter(float x_data)
{
	// lowpass filter butterworth order 2nd fc 20 hz sampling 200hz
	
	x_prev_2 = x_prev_1;
	x_prev_1 = x_data ;
	
	y_prev_2 = y_prev_1;
	y_prev_1 = y_data ;
	
	float nume = (x_data + 2.0f * x_prev_1 +  x_prev_2);
	float denom = (- 1.1429804563522339f * y_prev_1 +  0.412801593542099f * y_prev_2);
	y_data = 0.067455276846885681f * nume - denom;
	return y_data;
}

void Motor_disable(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
}
void Motor_enable(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	TIM16->CNT = 0;
	HAL_TIM_Base_Start_IT(&htim16);
}
void Motor_lock(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_TIM_Base_Stop_IT(&htim16);
	position_cart_velo = 0;
}

void Motor_drive(float tmp_acc, float velo_limit)
{
	float tmp_velo = 0;
	float tmp_abs_velo = 0;
	float velo_trim  = 0;

	
		if (Mode == 2 || Mode == 3 || Mode == 5)
		{
			velo_trim = (- position_cart * 0.5f) - position_cart_velo *0.35f   ;


		}else{
			velo_trim = 0;
		}
	
	position_cart_velo += (tmp_acc + velo_trim) / sampling; 								// intregrated
	    
	if (position_cart_velo > max_velo)  position_cart_velo = max_velo;		// protection
	if (position_cart_velo < -max_velo) position_cart_velo = -max_velo;   // protection
	
	if (position_cart_velo > 0)
	{
		tmp_abs_velo = position_cart_velo;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);							// +x
	}else{
		tmp_abs_velo = -position_cart_velo;
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);								// -x
	}

	if (tmp_abs_velo > velo_limit)
	{
		tmp_abs_velo = velo_limit;
		
		if (position_cart_velo > 0)
		{
			position_cart_velo = velo_limit;		// protection
		}else{
			position_cart_velo = -velo_limit; 
		}
	}

	position_cart =  raw_position_cart / (step_Per_mm *2.0f);	

	tmp_velo = step_clock / (step_Per_mm *2.0f) /  tmp_abs_velo;							// changing to period 
	

	if (tmp_velo > 0xffff)  tmp_velo = 0xffff;														// protection													// protection
	TIM16->CNT = 0;	
	TIM16->ARR = tmp_velo;																								// write to register
	
}




void Limit_ws_check(void)
{
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_RESET);		// turn off led
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);		// turn off led
	
	if( _limit_state == 0)
	{
		if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET )
		{
			Motor_lock();
			_limit_state = 1; 
			raw_position_cart = start_cart_position;
			position_cart     = raw_position_cart / (step_Per_mm *2.0f);	;
			position_cart_velo = 0;
		}
		
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_SET)
		{
			Motor_lock();
			_limit_state = 2;
			position_cart_velo = 0;
		}
	}
	
	if ( HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_SET)
	{
		_limit_state = 0;
	}
}
void Limit_ws_unlock(void)
{
	_limit_state = 0;
}

void blink_green(void)
{
	static uint8_t count;
	count++;
	if (count > 20)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0, GPIO_PIN_SET);
		count = 0;
	}
}

void blink_red(void)
{
	static uint8_t count;
	count++;
	if (count > 21)
	{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		count = 0;
	}
}
void _drive_step_pin(void)
{
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_9);
	if (inte_cart_enable != 0)
	{
		if(position_cart_velo >= 0)
		{
			raw_position_cart++;
		}else{
			raw_position_cart--;
		}
	}
}

void reset_filter(void)
{
	y_data = 0;
	y_prev_1 = 0;
	y_prev_2 = 0;
	x_prev_1 = 0;
	x_prev_2 = 0;
}
void Print_Debug(void)
{
    uint8_t header[2] = {0x7e, 0x7e};
    uint8_t terminator[2] = {0xe7, 0xe7};

    HAL_UART_Transmit(&huart1, header, 2, 1);    // sent header
//    HAL_UART_Transmit(&huart1, (uint8_t *)&force, 4, 1);
//    HAL_UART_Transmit(&huart1, (uint8_t *)&angle, 4, 1);
//    HAL_UART_Transmit(&huart1, (uint8_t *)&position, 4, 1);
    HAL_UART_Transmit(&huart1, terminator, 2, 1);    // sent header
}
/* USER CODE END 4 */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
