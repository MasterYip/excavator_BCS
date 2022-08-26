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
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();//不可删除！DMA初始化需要在USART之前才能正常工作 但CUBE生成在后边 因此特地提至前面
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  
	/******IMU初始化*******/	
	HAL_Delay(500);			// 必须加延时！  否则上电时 mpu6050 初始化会失败
	//成功初始化后 uart_node节点正常运行 LED红灯闪烁
	icm20602_init(ICM20602_CS_Pointer[0]);
	icm20602_init(ICM20602_CS_Pointer[1]);
	icm20602_init(ICM20602_CS_Pointer[2]);
	icm20602_init(ICM20602_CS_Pointer[3]);
	
	HAL_Delay(500);	
	/******获取加电时角速度的静态误差*******/		
	
	Get_Gyro_Static_Error(ICM20602_CS_Pointer[0], &ICM20602_Data[0]);
	Get_Gyro_Static_Error(ICM20602_CS_Pointer[1], &ICM20602_Data[1]);
	Get_Gyro_Static_Error(ICM20602_CS_Pointer[2], &ICM20602_Data[2]);
	Get_Gyro_Static_Error(ICM20602_CS_Pointer[3], &ICM20602_Data[3]);
	//初始化四元数数据
	IMU_Data_Init(ICM20602_Data);
	
	/******定时器6中断初始化*******/
	__HAL_TIM_CLEAR_IT(&htim6,TIM_IT_UPDATE);
	HAL_TIM_Base_Start_IT(&htim6);
	
	/******PWM输出初始化*******/
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_4);
	
	/******串口中断UART1初始化*******/
	
	HAL_UART_Receive_IT(&huart2,buff,1);
	
	/******ROSsetup*******/
	setup();
	
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	//ROS serial loop
	loop();
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
}

/* USER CODE BEGIN 4 */
//定时器周期5ms
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance==TIM6)
	{
		char data[200]="";
		static int k=0;
		static int flag = 1;
		//系统时间
		systime+=TimePeriod;
		k = (k+1)%10;
		

		IMUs_update(ICM20602_CS_Pointer, ICM20602_Data, Filters);
		
		//用于修正四元数标定（调整至最大角度）
		if(systime < 4.0)
		{
			Vector_Control(RevisionMotion);
		}
		else if (systime < 5.0)
		{
			Vector_Control(STOPVector);
		}
		
		if(systime > 5.0 && flag ==1)
		{
			flag=0;
			QuaternionReviser(ICM20602_Data, AngleforRevision);
		}
		
		if(systime > 5.5)
		{
			double pos[4] = {1.57, 0.8, 0, -1};
			double vel[4] = {0};
			
			//机械臂角度解算
			RobotArm_Resolver(ICM20602_Data, Angles);
			urdfAngles[0] = Angles[0];
			  urdfAngles[1] = Angles[1];
			  urdfAngles[2] = Angles[2]-1.5708;
			  urdfAngles[3] = Angles[3]-4.7124;
			

			
			
			//输出关节角度
			if(k==0)
			{
				sprintf(data, "\r\n%.0f\t%.0f\t%.0f\t%.0f", Angles[0]*57.3,Angles[1]*57.3,Angles[2]*57.3, Angles[3]*57.3);
//				HAL_UART_Transmit_DMA(&huart2, (uint8_t*)data, sizeof(data));
//				HALprintf(data);
			}
			

			//匿名飞控信息发送
//			ANOTC_Attitude_Display_DMA(&huart2, ICM20602_Data[2]);//未修正
//			ANOTC_RevisedAttitude_Display_DMA(&huart2, &ICM20602_Data[0]);
//			ANOTC_RevisedAttitude_Display_DMA(&huart2, &ICM20602_Data[1]);
//			ANOTC_RevisedAttitude_Display_DMA(&huart2, &ICM20602_Data[2]);
//			ANOTC_RevisedAttitude_Display_DMA(&huart2, &ICM20602_Data[3]);
		
			
//			PID_arm_controller(pos, vel, urdfAngles);
			
			//电机控制函数
			Vector_Control_with_Time_Limited(MOTOR_Continue);
			
			//串口马达控制
			key_teleop_ctrl(&cmd);
		

		}

	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == (&huart1))
	{
		USART_TxCplt_ROSCallback();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == (&huart1))
	{
		USART_RxCplt_ROSCallback();
	}
	
	if(huart == (&huart2))
	{
		cmd = buff[0];
		HAL_UART_Receive_IT(huart,buff,1);
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
