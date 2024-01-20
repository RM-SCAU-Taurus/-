/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @project        : Taurus_IMU
	* @version        : 3.0
	* @date 					: 2020.1.5
	*	@author 				: YY
  ******************************************************************************
  * @attention
  *													    !!! READ ME !!!
	
  * This project is the driving algorithm of attitude sensor developed by Taurus.
	* Only for Taurus research & development and learning.
	* Without Taurus permission is forbidden to gaiden ! ! !
  * 
  * This version is a transitional version and has not yet been fully developed.
  * This version needs to be used with the WT901.
  * Palstance is provided by BMI088 and Angle is provided by WT901.
  * The palstance update frequency is 1000 hz and the Angle update frequency is 200 hz.
  *	Both of them communicate with the master controller through the can bus.
  *
	*	Any enquiry please contact us directly!
	*
	*																								South China Agricultural University
	*																														 RoboMaster
	*																															 Taurus
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bsp_spi.h"
#include "bsp_iic.h"
#include "bsp_uart.h"
#include "bsp_can.h"
#include "bsp_delay.h"
#include "bsp_bmi088.h"
//#include "bsp_ist8310.h"
#include "Filter.h"
#include "Calibration.h"
#include "Status_Task.h"
#include "IMU_AHRSupdate.h"
#include "KalmanFilter.h"
#include "Comm_task.h"
#include "data_processing.h"
#include "bsp_flash.h"
#include "bsp_imu.h"
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
float test1;
float test2;
float read_data;
FLASH_EraseInitTypeDef My_Flash;
float write_data=12.3456781;
uint8_t write_msg[4];
uint8_t read_msg[4];
uint32_t addres = 0x08007890;
extern float  Temperature;	 //IMU温度
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
  MX_SPI3_Init();
  MX_CAN_Init();
  MX_TIM16_Init();
  MX_USART1_UART_Init();
  MX_TIM17_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	My_Flash.TypeErase = FLASH_TYPEERASE_PAGES;  //标明Flash执行页面只做擦除操作
	 HAL_FLASH_Unlock();  
   My_Flash.PageAddress = addres;  //声明要擦除的地址
   My_Flash.NbPages = 1;   
	 uint32_t PageError = 0;                    //设置PageError,如果出现错误这个变量会被设置为出错的FLASH地址
    
	Data_Not_Ready();	//数据未就绪状态
	HAL_Delay(500);
	IIC_Soft_Init();
	user_uart_init();
	can_device_init();
	//HAL_Delay(200);
	BMI088_Init();
//	IST8310_Init();
  BMI088_original_data_read();
	if(imu_org_data.Accel.X==0)
	{
	  BMI088_Init();
	}
	Butterworth_Parameter_Init();
	BMI088_Calibration();
	
 	Kalman_init();
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_TIM_Base_Start_IT(&htim16);
	HAL_TIM_Base_Start_IT(&htim17);
	HAL_Delay(300);
	BMI088_temp_data_read();
	Data_Ready();
	flash_erase_address(&My_Flash,BIAS_GYRO_X_ADDRESS,1);
	flash_erase_address(&My_Flash,BIAS_GYRO_Y_ADDRESS,1);
	flash_erase_address(&My_Flash,BIAS_GYRO_Z_ADDRESS,1);
	flash_erase_address(&My_Flash,BIAS_INIT_TEMPTURE,1);
	
	flash_erase_address(&My_Flash,BIAS_ACCEL_X_ADDRESS,1);
	flash_erase_address(&My_Flash,BIAS_ACCEL_Y_ADDRESS,1);
	flash_erase_address(&My_Flash,BIAS_ACCEL_Z_ADDRESS,1);
	
	flash_write_single_address(&My_Flash,BIAS_GYRO_X_ADDRESS,Bias.Gyro.X);
	flash_write_single_address(&My_Flash,BIAS_GYRO_Y_ADDRESS,Bias.Gyro.Y);
	flash_write_single_address(&My_Flash,BIAS_GYRO_Z_ADDRESS,Bias.Gyro.Z);
	flash_write_single_address(&My_Flash,BIAS_INIT_TEMPTURE,Temperature);
	
	flash_write_single_address(&My_Flash,BIAS_ACCEL_X_ADDRESS,Bias.Accel.X);
	flash_write_single_address(&My_Flash,BIAS_ACCEL_Y_ADDRESS,Bias.Accel.Y);
	flash_write_single_address(&My_Flash,BIAS_ACCEL_Z_ADDRESS,Bias.Accel.Z);
	//flash_read(addres,&read_data);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		// arm_lms_f32_test2();
	   if(bias_gyro_mode==Calibration_error_mode)
		 {
		    Status_LED_PWM_Set(999);
			  HAL_Delay(100);
			 Status_LED_PWM_Set(0);
			  HAL_Delay(100);
		 
		 }
		 else if(bias_gyro_mode==Calibration_successful_mode)
		 {
		     Status_LED_PWM_Set(999);
			  HAL_Delay(1000);
			 Status_LED_PWM_Set(0);
			  HAL_Delay(1000);
		 }
		// HAL_Delay(50);
	//	status_task();
	//	HAL_Delay(1);
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_TIM16|RCC_PERIPHCLK_TIM17;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim16ClockSelection = RCC_TIM16CLK_HCLK;
  PeriphClkInit.Tim17ClockSelection = RCC_TIM17CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if(htim->Instance==TIM1)
	{ 
	 // BMI_Read();
	//	IMU_AHRSupdate_task(); 
		
		
	}
	else if(htim->Instance==TIM16)
	{
	  //IMU_AHRSupdate_task();
	 // can_msg_send_task();
		//kalman_R_calcu();
	
//		 status_task();
	}
//	else if(htim->Instance==TIM17)
//	{
//	  //can_msg_send_task();
//    // kalman_R_calcu();		
//	}
  /* USER CODE END Callback 1 */
}

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
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
