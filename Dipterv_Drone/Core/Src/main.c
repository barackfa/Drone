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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32f4xx_hal.h"
#include "BMI088.h"
#include "BMP388.h"
#include "BMM150.h"
#include "ESC.h"
#include "Orifilter.h"
#include "oriIMU.h"
#include "Fusion.h"
//#include "Altitude_filter.h"
#include <stdbool.h>
#include <stdio.h>

#include "bmm150.h"
#include "bmm150_common.h"
#include "bmm150_defs.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SAMPLE_PERIOD 0.005f // replace this with actual sample period
#define ground_pressure 101352
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart1_rx;

osThreadId defaultTaskHandle;
osThreadId Data_ReadingHandle;
osThreadId Orientation_calHandle;
/* USER CODE BEGIN PV */



/* Initialise inertial measurement unit */
  //BMI088_Init(&imu, &hspi2, CS_GYRO_GPIO_Port, CS_GYRO_Pin, GPIOC, SPI1_NCS_GYR_Pin);

BMI088 imu;
BMP388_HandleTypeDef bmp;
//BMM150 bmm;
//BMM150_trim_data trim_data;
int16_t field_x;
int16_t field_y;
int16_t field_z;
uint16_t Rhall;
float mag_data_x = 0;
float mag_data_y = 0;
float mag_data_z = 0;

uint8_t billent = 0;


uint32_t raw_press;
uint32_t raw_temp;
uint32_t raw_time;
float press;
float temp;
float hz;
float h0;

//filter objects
float gyro_x_degree = 0;
float gyro_y_degree = 0;
float gyro_z_degree = 0;
quaternion q;
flux f;
w_err w;
float roll;
float pitch;
float yaw;
float prev_yaw = 0;
float yaw_offset = 0;
float yaw_zero_offset = 0;
float abs_yaw = 0;
FusionAhrs ahrs;
FusionEuler euler;
FusionMatrix ERS;
FusionVector aE;

//KF_Matrix31 prev_state;
//KF_Matrix31 current_state;
//KF_Matrix33 P_prev;
//KF_Matrix33 P;
//KF_Matrix21 meas;

//distance measurement
uint32_t IC_Val1 = 0;
uint32_t IC_Val2 = 0;
uint32_t Difference = 0;
uint8_t Is_First_Captured = 0;  // is the first value captured ?
uint8_t Distance  = 0;

//telemetria
uint8_t telem[8] = {15};
uint8_t uart_telemetria = 0;
float telem_P = 0;
float telem_D = 0;
uint8_t new_P = 0;
uint8_t new_D = 0;
uint8_t telemetria_data_sent = 0;


//order flags
uint8_t readstart = 1;
uint8_t oricalc = 0;


// debug variables
float gyro_debug[500] = {15};
int gyro_i = 0;
int debug_i = 0;
uint8_t timerse = 0;
uint8_t state = 1;
int mytimer;

float m_alfa = 0;
float m_beta = 0;
float m_gamma = 0;
float xy2 = 0;
int offset_i = 0;
double gyro_offset_x_calc = 0;
double gyro_offset_y_calc = 0;
double gyro_offset_z_calc = 0;
double acc_offset_x_calc = 0;
double acc_offset_y_calc = 0;
double acc_offset_z_calc = 0;
double gyro_offset_x = 0;
double gyro_offset_y = 0;
double gyro_offset_z = 0;
double acc_offset_x = 0;
double acc_offset_y = 0;
double acc_offset_z = 0;
float mag_debug_x[1000];
float mag_debug_y[1000];
int i_mag = 0;

//ESC ref throttle params
uint16_t ref1;
uint16_t ref2;
uint16_t ref3;
uint16_t ref4;

uint8_t CRSF_debug[64] = {0};
uint8_t CRSF_i = 0;
uint8_t UART1_rxBuffer[1] = {0};
uint8_t bytetoread = 1;
uint16_t RX_roll = 0;
uint16_t RX_pitch = 0;
uint16_t RX_yaw = 0;
uint16_t RX_throttle = 0;
uint16_t RX_arm = 0;

float M_throttle, M_pitch, M_roll, M_yaw;
float debug_control1, debug_control2;

QueueHandle_t telemetria_Queue;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
void StartDefaultTask(void const * argument);
void Start_Data_Reading(void const * argument);
void Start_Orientation(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static int8_t set_config(struct bmm150_dev *dev);
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
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */




  __HAL_SPI_ENABLE(&hspi2);
  HAL_TIM_PWM_Start  ( &htim3,  TIM_CHANNEL_1  );
  HAL_TIM_PWM_Start  ( &htim3,  TIM_CHANNEL_2  );
  HAL_TIM_PWM_Start  ( &htim3,  TIM_CHANNEL_3  );
  HAL_TIM_PWM_Start  ( &htim3,  TIM_CHANNEL_4  );

  HAL_TIM_Base_Start  ( &htim10 );




  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  telemetria_Queue = xQueueCreate( 3, 3*sizeof( float ) );
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityBelowNormal, 0, 500);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Data_Reading */
  osThreadDef(Data_Reading, Start_Data_Reading, osPriorityNormal, 0, 600);
  Data_ReadingHandle = osThreadCreate(osThread(Data_Reading), NULL);

  /* definition and creation of Orientation_cal */
  osThreadDef(Orientation_cal, Start_Orientation, osPriorityBelowNormal, 0, 200);
  Orientation_calHandle = osThreadCreate(osThread(Orientation_cal), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {


	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_3);

	  HAL_Delay(3);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV6;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

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
  hi2c1.Init.ClockSpeed = 400000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 400000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  htim6.Init.Prescaler = 84-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65000;
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
  htim7.Init.Prescaler = 85-1;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 10000-1;
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
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 168-1;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 420000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.BaudRate = 115200;
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
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, TRIG_Pin|GNSS_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CS_GYRO_Pin|GPIO_PIN_3|GPIO_PIN_4, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_ACC_GPIO_Port, CS_ACC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : IT_MAGN_Pin BUTTON_Pin */
  GPIO_InitStruct.Pin = IT_MAGN_Pin|BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : TRIG_Pin GNSS_RST_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin|GNSS_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : IT_PRESS_Pin */
  GPIO_InitStruct.Pin = IT_PRESS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IT_PRESS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CS_GYRO_Pin PB3 PB4 */
  GPIO_InitStruct.Pin = CS_GYRO_Pin|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_ACC_Pin */
  GPIO_InitStruct.Pin = CS_ACC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_ACC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : IT_GYRO_Pin IT_ACC_Pin */
  GPIO_InitStruct.Pin = IT_GYRO_Pin|IT_ACC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
static int8_t set_config(struct bmm150_dev *dev) {
    /* Status of api are returned to this variable. */
    int8_t rslt;

    struct bmm150_settings settings;

    /* Set powermode as normal mode */
    settings.pwr_mode = BMM150_POWERMODE_NORMAL;
    rslt = bmm150_set_op_mode(&settings, dev);
    bmm150_error_codes_print_result("bmm150_set_op_mode", rslt);

    if (rslt == BMM150_OK) {
        /* Setting the preset mode as Low power mode
         * i.e. data rate = 10Hz, XY-rep = 1, Z-rep = 2
         */
        settings.preset_mode = BMM150_PRESETMODE_HIGHACCURACY;                  // TODO Change it to the desired preset
        rslt = bmm150_set_presetmode(&settings, dev);
        settings.data_rate = BMM150_DATA_RATE_30HZ;                             // TODO Change it to the desired ODR
        bmm150_set_sensor_settings(BMM150_SEL_DATA_RATE, &settings, dev);
        bmm150_error_codes_print_result("bmm150_set_presetmode", rslt);

        if (rslt == BMM150_OK) {
            /* Map the data interrupt pin */
            settings.int_settings.drdy_pin_en = 0x01;
            rslt = bmm150_set_sensor_settings(BMM150_SEL_DRDY_PIN_EN, &settings, dev);
            bmm150_error_codes_print_result("bmm150_set_sensor_settings", rslt);
        }
    }

    return rslt;
}




uint16_t Reverseuint16( uint16_t nonreversed )
{
    uint16_t reversed = 0;

    for ( uint16_t i = 0; i < 16; i++ )
    {
        reversed |= ( nonreversed >> ( 16 - i - 1 ) & 1 ) << i;
    }

    return reversed;
}

uint8_t Reverseuint8( uint8_t nonreversed )
{
    uint8_t reversed = 0;

    for ( uint8_t i = 0; i < 8; i++ )
    {
        reversed |= ( nonreversed >> ( 8 - i - 1 ) & 1 ) << i;
    }

    return reversed;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == BUTTON_Pin) {
		if(state == 4){state = 5;}
		if(state == 3){state = 4;}
		if(state == 2){state = 3;}
		if(state == 1){state = 2;}
	  } else {
	      __NOP();
	  }

	if(GPIO_Pin == IT_ACC_Pin) {

	 readstart = 1;
  } else {
      __NOP();
  }
  if(GPIO_Pin == IT_MAGN_Pin) {
  	 //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);

  	 billent = 1;
    } else {
        __NOP();
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2 )
	  {
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  // if the interrupt source is channel1
		{
			if (Is_First_Captured==0) // if the first value is not captured
			{
				IC_Val1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1); // read the first value
				Is_First_Captured = 1;  // set the first captured as true
				// Now change the polarity to falling edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_FALLING);
			}

			else if (Is_First_Captured==1)   // if the first is already captured
			{
				IC_Val2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);  // read second value
				__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

				if (IC_Val2 > IC_Val1)
				{
					Difference = IC_Val2-IC_Val1;
				}

				else if (IC_Val1 > IC_Val2)
				{
					Difference = (0xffff - IC_Val1) + IC_Val2;
				}

				Distance = Difference * .34/2;
				Is_First_Captured = 0; // set it back to false

				// set polarity to rising edge
				__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_1, TIM_INPUTCHANNELPOLARITY_RISING);
				//__HAL_TIM_DISABLE_IT(&htim2, TIM_IT_CC1);
			}
		}
	  }
}

// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim6 )
  {
	  if(timerse == 1){timerse =0;}
	  if(timerse == 0){timerse =1;}
  }
  if(htim == &htim7){

  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart2){
			telemetria_data_sent = 1;
	}
}



void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart ==&huart1){
		debug_i = __HAL_TIM_GET_COUNTER(&htim6);
		if(debug_i < 100){
			CRSF_debug[CRSF_i] = UART1_rxBuffer[0];
			CRSF_i++;
		}
		else{
			if(CRSF_debug[2] == 0x16){
				RX_roll = (((uint16_t)(CRSF_debug[4] & 0b00000111)) << 8) + (uint16_t)(CRSF_debug[3]);
				RX_pitch = (((uint16_t)(CRSF_debug[5] & 0b00111111)) << 5) + (((uint16_t)(CRSF_debug[4] & 0b11111000)) >> 3);
				RX_throttle = (((uint16_t)(CRSF_debug[7] & 0b00000001)) << 10)+ (((uint16_t)(CRSF_debug[6])) << 2) + (((uint16_t)(CRSF_debug[5] & 0b11000000)) >> 6);
				RX_yaw = (((uint16_t)(CRSF_debug[8] & 0b00001111)) << 7) + (((uint16_t)(CRSF_debug[7] & 0b11111110)) >> 1);
				RX_arm = (((uint16_t)(CRSF_debug[9] & 0b01111111)) << 4) + (((uint16_t)(CRSF_debug[8] & 0b11110000)) >> 4);
			}
			CRSF_debug[0] = UART1_rxBuffer[0];
			CRSF_i = 1;
		}
		htim6.Instance->CNT = 0;
	}
	if(huart == &huart2){
		if(uart_telemetria == 1){
			HAL_UART_Receive_IT(&huart2, telem, 11);
		}
		else
			HAL_UART_Receive_IT(&huart2, telem, 11);
	}
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	//HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
//	uint8_t telemetria[8];
	uint8_t telemetria_data[40] = "HELLO WORLD \r\n";
	extern QueueHandle_t telemetria_Queue;
	float telemetria_send[3];


  /* Infinite loop */
  for(;;)
  {
	  if(uart_telemetria == 1){
		  if(telem[0] == 'P'){
			  telem_P = (float)(((uint32_t)telem[1]-48)*1000000+((uint32_t)telem[2]-48)*100000+((uint32_t)telem[3]-48)*10000+((uint32_t)telem[4]-48)*1000+((uint32_t)telem[5]-48)*100+((uint32_t)telem[6]-48)*10+((uint32_t)telem[7]-48))/100000;
			  new_P = 1;
		  }
		  if(telem[0] == 'D'){
			  telem_D = (float)(((uint32_t)telem[1]-48)*1000000+((uint32_t)telem[2]-48)*100000+((uint32_t)telem[3]-48)*10000+((uint32_t)telem[4]-48)*1000+((uint32_t)telem[5]-48)*100+((uint32_t)telem[6]-48)*10+((uint32_t)telem[7]-48))/100000;
			  new_D = 1;
		  }
	  }
	  if (xQueueReceive(telemetria_Queue, (void*)&telemetria_send, 0) == pdTRUE){
		  /*sprintf((char*)telemetria_data, "Yaw: %4.2f\r\n", drone_angle[0]); //%5.2f
//		  sprintf((char*)telemetria_data, "Raw:0,0,0,0,0,0,%d,%d,%d\r\n", (int)((drone_angle[0])*10), (int)((drone_angle[1])*10), (int)(drone_angle[2])*10); //%5.2f
	//	  sprintf((char*)telemetria_data, "Yaw: 115.47\r\n");
		  HAL_UART_Transmit (&huart2, telemetria_data, sizeof (telemetria_data), 200);
*/

		  if(telemetria_data_sent == 1){
		  			  sprintf((char*)telemetria_data, "%4.3f,%4.3f,%4.3f\r\n", telemetria_send[0], telemetria_send[1], telemetria_send[2]); //%5.2f
//		  			  sprintf((char*)telemetria_data, "%2.2f, %2.2f, %2.2f, %3.2f, %3.2f, %3.2f, %4.1f, %4.1f, %4.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f, %3.1f\r\n", telemetria_send[0], telemetria_send[1], telemetria_send[2], telemetria_send[3], telemetria_send[4], telemetria_send[5], telemetria_send[6], telemetria_send[7], telemetria_send[8], telemetria_send[9], telemetria_send[10], telemetria_send[11], telemetria_send[12],telemetria_send[13], telemetria_send[14], telemetria_send[15], telemetria_send[16], telemetria_send[17], telemetria_send[18]); //%5.2f
		  			  HAL_UART_Transmit_IT(&huart2, telemetria_data, sizeof (telemetria_data));
		  			  telemetria_data_sent = 0;
		  		  }
	  }


	  osDelay(10);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_Data_Reading */
/**
* @brief Function implementing the Data_Reading thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Data_Reading */
void Start_Data_Reading(void const * argument)
{
  /* USER CODE BEGIN Start_Data_Reading */
	extern QueueHandle_t telemetria_Queue;

	//magnetometer calibration
	//MATLAB calibration
//	FusionVector magneto_offset = {-8.8555, 1.5088, -5.1718};//{-11.8, -5.68, 3.08};
//	FusionMatrix magneto_transform = {0.0022,    0.0038,    0.0205, -0.0210,    0.0041,    0.0015, -0.0038,   -0.0210,    0.0043};

	//Magneto 1.2 calibration
	FusionVector magneto_offset = {-10.254290, 1.8038, -4.628919};
	FusionMatrix magneto_transform = {1.030904, 0.011754, -0.008844, 0.011754, 1.040290, -0.000902, -0.008844, -0.000902, 1.008504};

	//no calibration
//	FusionVector magneto_offset = {0, 0, 0};
//	FusionMatrix magneto_transform = {1,0,0,0,1,0,0,0,1};
	FusionVector magneto_data;

	//pitch angle velocity control params
	float err_pitch = 0;
	float errd_pitch = 0;
	float prev_err_pitch = 0;
	float control_pitch = 0;
	float P_pitch = 20;
	float D_pitch = 0.1;

	//pitch angle control params
	float err_angle_pitch = 0;
	float errd_angle_pitch = 0;
	float prev_err_angle_pitch = 0;
	float angle_control_pitch = 0;
	float P_angle_pitch = 0.04;
	float D_angle_pitch = 0.0001;//0.005;

	//roll angle velocity control params
	float err_roll = 0;
	float errd_roll = 0;
	float prev_err_roll = 0;
	float control_roll = 0;
	float P_roll = 5;
	float D_roll = 0.1;//0.4

	//roll angle control params
	float P_angle_roll = 0.2;
	float D_angle_roll = 0.0001;//0.005;
	float err_angle_roll = 0;
	float errd_angle_roll = 0;
	float prev_err_angle_roll = 0;
	float angle_control_roll = 0;

	//yaw angle velocity control params
	float err_yaw = 0;
	float errd_yaw = 0;
	float prev_err_yaw = 0;
	float control_yaw = 0;
	float P_yaw = 20;
	float D_yaw = 0.1;

	//yaw angle control params
	float P_angle_yaw = 0.8;//0.04;
	float D_angle_yaw = 0.000;//0.005;
	float err_angle_yaw = 0;
	float errd_angle_yaw = 0;
	float prev_err_angle_yaw = 0;
	float angle_control_yaw = 0;

	float yaw_angle = 0;
	float prev_euler_yaw = 0;
	int n = 0;



	//imu init function
	BMI088_Init(&imu, &hspi2, CS_ACC_GPIO_Port, CS_ACC_Pin, CS_GYRO_GPIO_Port, CS_GYRO_Pin);


	//bmp388 pressure sensor init
	bmp._hi2c = &hi2c2;


	BMP388_SetTempOS(&bmp, 0);
	HAL_Delay(10);
	BMP388_SetPressOS(&bmp, 0x03); //0 volt, de adatlap alapján 8x-nek megfelelő 0x03 beírva
	HAL_Delay(10);
	BMP388_SetIIRFilterCoeff(&bmp, 2);
	HAL_Delay(10);
	BMP388_SetOutputDataRate(&bmp, 0x02);
	HAL_Delay(10);
	BMP388_Init(&bmp);

	for(int i_init = 0; i_init<2000; i_init++ ){
	  BMP388_ReadRawPressTempTime(&bmp, &raw_press, &raw_temp, &raw_time);
	  BMP388_CompensateRawPressTemp(&bmp, raw_press, raw_temp, &press, &temp);
	  h0 += BMP388_FindAltitude(ground_pressure, press);
	  BMI088_ReadGyroscope(&imu);
	  gyro_offset_x_calc += imu.gyr_rps[0];
	  gyro_offset_y_calc += imu.gyr_rps[1];
	  gyro_offset_z_calc += imu.gyr_rps[2];
	  HAL_Delay(1);
	}
	h0 /= 2000;
	gyro_offset_x = gyro_offset_x_calc/2000;
	gyro_offset_y = gyro_offset_y_calc/2000;
	gyro_offset_z = gyro_offset_z_calc/2000;

	//magneto sensor init
//	bmm.hi2c_handle = &hi2c1;
//
//	BMM150_Init(&bmm);
//	HAL_Delay(10);
//	BMM150_Get_TrimData(&bmm, &trim_data);




	 //BOSCH API
	 /* Sensor initialization configuration. */
	struct bmm150_dev dev;
	struct bmm150_mag_data mag_data;

	/* Status of api are returned to this variable */
	int8_t rslt;

	rslt = bmm150_interface_selection(&dev);
	bmm150_error_codes_print_result("bmm150_interface_selection", rslt);

	if (rslt == BMM150_OK) {
	        rslt = bmm150_init(&dev);
	        bmm150_error_codes_print_result("bmm150_init", rslt);

	        if (rslt == BMM150_OK) {
	            rslt = set_config(&dev);
	            bmm150_error_codes_print_result("set_config", rslt);

	//            if (rslt == BMM150_OK) {
	//                rslt = get_data(&dev);
	//                bmm150_error_codes_print_result("get_data", rslt);
	//            }
	        }
	    }

	//BOSCH API






	uint8_t transmit_data[40];
	float telemetria_float[3];



	q.SEq_1=1;
	q.SEq_2=0;
	q.SEq_3=0;
	q.SEq_4=0;

	w.w_bx=0;
	w.w_by=0;
	w.w_bz=0;



//	prev_state.a11=0;
//	prev_state.a21=0;
//	prev_state.a31=0;
//	current_state.a11=0;
//	current_state.a21=0;
//	current_state.a31=0;
//	P_prev.a11 = 0;
//	P_prev.a12 = 0;
//	P_prev.a13 = 0;
//	P_prev.a21 = 0;
//	P_prev.a22 = 0;
//	P_prev.a23 = 0;
//	P_prev.a31 = 0;
//	P_prev.a32 = 0;
//	P_prev.a33 = 0;
//	meas.a11=0;
//	meas.a21=0;

	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);
	FusionAhrsInitialise(&ahrs);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_UART_Receive_DMA(&huart1, UART1_rxBuffer, bytetoread);
	HAL_UART_Transmit_IT(&huart2, transmit_data, sizeof (transmit_data));
//	HAL_UART_Receive_IT(&huart2, telem, 11);


	vTaskResume( defaultTaskHandle );

  /* Infinite loop */
  for(;;)
  {

	  	  mytimer = __HAL_TIM_GET_COUNTER(&htim7);
	  	  htim7.Instance->CNT = 0;
	  	  //BOSCH API magneto begin
	  	  bmm150_get_interrupt_status(&dev);
	  	  if (dev.int_status & BMM150_INT_ASSERTED_DRDY) {
	  		  /* Read mag data */
	  		  bmm150_read_mag_data(&mag_data, &dev);
	  	  }

	  	  //BOSCH API magneto end

//	  	  BMM150_Set_OpMode(&bmm, 0x02); //280 us - 100kHz,

		  // opmode start a measurement, because of the set preset mode, the results will be available in the next loop,
		  // with nXY = 5, nZ = 6 delay is -> 4.16 ms ~240Hz -> 200 Hz control loop available
//		  BMM150_GetRawData(&bmm, &field_x, &field_y, &field_z, &Rhall, 8); // all time 1.31 ms magnetometer i2c 100kHz, 330 us with 400 kHz



		  // magnetic field data in uT
//		  mag_data_x = BMM150_Compensate_x(field_x, Rhall,  &trim_data); //magn data compensation 33.4 us
//		  mag_data_y = BMM150_Compensate_y(field_y, Rhall,  &trim_data);
//		  mag_data_z = BMM150_Compensate_z(field_z, Rhall,  &trim_data);
		  magneto_data.axis.x = mag_data.y;
		  magneto_data.axis.y = -mag_data.x;
		  magneto_data.axis.z = mag_data.z;

		  if(i_mag < 1000){
			  mag_debug_x[i_mag] = mag_data_x;
			  mag_debug_y[i_mag] = mag_data_y;
			  i_mag++;
		  }

		  //read IMU
		  BMI088_ReadGyroscope(&imu);	// imu read 119 us
		  BMI088_ReadAccelerometer(&imu);


		  //BMP388_ReadRawPressTempTime(&bmp, &raw_press, &raw_temp, &raw_time); //2.46 ms - 400kHz


		  //BMP388_CompensateRawPressTemp(&bmp, raw_press, raw_temp, &press, &temp); //2.7 us
		  //full loop time 2.94 ms, without time read 2.48 ms
		  //hz = BMP388_FindAltitude(ground_pressure, press)-h0;


		  //filterUpdateIMU(imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2], imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], &q);
		  //filterUpdate((imu.gyr_rps[0]-gyro_offset_x), (imu.gyr_rps[1]-gyro_offset_y), imu.gyr_rps[2], imu.acc_mps2[0], imu.acc_mps2[1], (imu.gyr_rps[2]-gyro_offset_z), mag_data_y, -mag_data_x, mag_data_z, &q, &f, &w);


		  //eulerAngles(q, &roll, &pitch, &yaw);

		  gyro_x_degree = ((imu.gyr_rps[0]-gyro_offset_x)*57.29);
		  gyro_y_degree = ((imu.gyr_rps[1]-gyro_offset_x)*57.29);
		  gyro_z_degree = ((imu.gyr_rps[2]-gyro_offset_x)*57.29);


//		  magneto_data = FusionVectorSubtract(magneto_data, magneto_offset);
		  magneto_data = FusionMatrixMultiplyVector(magneto_transform, FusionVectorSubtract(magneto_data, magneto_offset));

		  const FusionVector gyroscope = {gyro_x_degree, gyro_y_degree, gyro_z_degree};
		  const FusionVector accelerometer = {imu.acc_mps2[0]/9.81, imu.acc_mps2[1]/9.81, imu.acc_mps2[2]/9.81};
		  const FusionVector magnetometer = {magneto_data.axis.x, magneto_data.axis.y, magneto_data.axis.z};


		  //no magnetometer AHRS
//		  FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, SAMPLE_PERIOD);

		  //magnetometer AHRS
		  FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer, SAMPLE_PERIOD);

		  euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
		  // Rotation matrix from sensor frame to earth(NWU) frame
		  ERS = FusionQuaternionToMatrix(FusionAhrsGetQuaternion(&ahrs));
		  aE = FusionMatrixMultiplyVector(ERS, FusionVectorMultiplyScalar(accelerometer, 9.81));
		  aE.axis.z -=9.85173;



		  // calculate rotation around yaw axis
		  if(prev_euler_yaw > 170 && euler.angle.yaw < 0){
			  n++;
		  }
		  if(prev_euler_yaw < -170 && euler.angle.yaw > 0){
			  n--;
		  }
		  yaw_angle = euler.angle.yaw + n * 360.0;
		  abs_yaw = yaw_angle;
		  prev_euler_yaw = euler.angle.yaw;


		  //python
//		  sprintf((char*)transmit_data, "Uni:0,0,0,0,0,0,%5.2f,%5.2f,%5.2f\r\n", mag_data_y, (-mag_data_x), mag_data_z); //%5.2f
//		  HAL_UART_Transmit (&huart2, transmit_data, sizeof (transmit_data), 100);
//		  HAL_Delay(1);

		  //motioncal
//		  sprintf((char*)transmit_data, "Raw:0,0,0,0,0,0,%d,%d,%d\r\n", (int)(magnetometer.axis.x*10), (int)((magnetometer.axis.y)*10), (int)(magnetometer.axis.z)*10); //%5.2f
//		  HAL_UART_Transmit (&huart2, transmit_data, sizeof (transmit_data), 500);



		  //altitudeKF(prev_state, &current_state, P_prev, &P, meas);
		  M_throttle = CRSFtoDuty(RX_throttle);
		  M_pitch = CRSFtoPitch(RX_pitch)*25;
		  M_roll = CRSFtoRoll(RX_roll)*15;
		  M_yaw += CRSFtoYaw(RX_yaw)*0.3;

		  //pitch angle control
		  err_angle_pitch = M_pitch - euler.angle.pitch;
		  errd_angle_pitch = (err_angle_pitch - prev_err_angle_pitch)/SAMPLE_PERIOD;
		  angle_control_pitch = P_angle_pitch * err_angle_pitch + D_angle_pitch * errd_angle_pitch;
		  prev_err_angle_pitch = err_angle_pitch;
		  //debug_control1 = err_angle_pitch;

		  //pitch angle velocity control
		  err_pitch = angle_control_pitch - imu.gyr_rps[1];
		  errd_pitch = (err_pitch - prev_err_pitch)/SAMPLE_PERIOD;
		  prev_err_pitch = err_pitch;
		  control_pitch = P_pitch * err_pitch + D_pitch * errd_pitch;

		  //roll angle control
		  err_angle_roll = M_roll - euler.angle.roll;
		  errd_angle_roll = (err_angle_roll - prev_err_angle_roll)/SAMPLE_PERIOD;
		  angle_control_roll = P_angle_roll * err_angle_roll + D_angle_roll * errd_angle_roll;
		  prev_err_angle_roll = err_angle_roll;
		  debug_control1 = err_angle_roll;


		  //roll angle velocity control
		  err_roll = angle_control_roll - imu.gyr_rps[0]; //M_roll
		  errd_roll = (err_roll - prev_err_roll)/SAMPLE_PERIOD;
		  prev_err_roll = err_roll;
		  control_roll = P_roll * err_roll + D_roll * errd_roll;
		  debug_control2 = control_roll;


		  //yaw angle control
		  err_angle_yaw = M_yaw - euler.angle.yaw;
		  errd_angle_yaw = (err_angle_yaw - prev_err_angle_yaw)/SAMPLE_PERIOD;
		  angle_control_yaw = P_angle_yaw * err_angle_yaw + D_angle_yaw * errd_angle_yaw;
		  prev_err_angle_yaw = err_angle_yaw;



		  //yaw angle velocity control
		  err_yaw = angle_control_yaw - imu.gyr_rps[2]; //angle_control_yaw
		  errd_yaw = (err_yaw - prev_err_yaw)/SAMPLE_PERIOD;
		  prev_err_yaw = err_yaw;
		  control_yaw = P_yaw * err_yaw + D_yaw * errd_yaw;



		  if(RX_arm > 1000){
			  uart_telemetria = 0;
			  //pitch
//			  ref1 = (uint16_t)(M_throttle - control_pitch);
//			  ref2 = (uint16_t)(M_throttle - control_pitch);
//			  ref3 = (uint16_t)(M_throttle + control_pitch);
//			  ref4 = (uint16_t)(M_throttle + control_pitch);

			  //roll
//			  ref1 = (uint16_t)(M_throttle + control_roll);
//			  ref2 = (uint16_t)(M_throttle - control_roll);
//			  ref3 = (uint16_t)(M_throttle - control_roll);
//			  ref4 = (uint16_t)(M_throttle + control_roll);

			  //yaw
//			  ref1 = (uint16_t)(M_throttle - control_yaw);
//			  ref2 = (uint16_t)(M_throttle + control_yaw);
//			  ref3 = (uint16_t)(M_throttle - control_yaw);
//			  ref4 = (uint16_t)(M_throttle + control_yaw);

			  //all together
			  ref1 = (uint16_t)(M_throttle - control_yaw - control_pitch + control_roll);
			  ref2 = (uint16_t)(M_throttle + control_yaw - control_pitch - control_roll);
			  ref3 = (uint16_t)(M_throttle - control_yaw + control_pitch - control_roll);
			  ref4 = (uint16_t)(M_throttle + control_yaw + control_pitch + control_roll);

//			  ref1 = (uint16_t)(M_throttle - control_pitch + control_roll);
//			  ref2 = (uint16_t)(M_throttle - control_pitch - control_roll);
//			  ref3 = (uint16_t)(M_throttle + control_pitch - control_roll);
//			  ref4 = (uint16_t)(M_throttle + control_pitch + control_roll);

//			  ref1 = (uint16_t)(M_throttle);
//			  ref2 = (uint16_t)(M_throttle);
//			  ref3 = (uint16_t)(M_throttle);
//			  ref4 = (uint16_t)(M_throttle);

			  if(ref1<550) ref1 = 550;
			  if(ref2<550) ref2 = 550;
			  if(ref3<550) ref3 = 550;
			  if(ref4<550) ref4 = 550;

		  }
		  else{
			  uart_telemetria = 1;
			  if(new_P == 1){
				  P_yaw = telem_P;
				  new_P = 0;
			  }
			  if(new_D == 1){
				  D_yaw = telem_D;
				  new_D = 0;
			  }
			  ref1 = 550;
			  ref2 = 550;
			  ref3 = 550;
			  ref4 = 550;
		  }

		  //telemetria
//		  telemetria_float[0] = (float)mag_data.x;
//		  telemetria_float[1] = (float)mag_data.y;
//		  telemetria_float[2] = (float)mag_data.z;

		  telemetria_float[0] = magneto_data.axis.x;
		  telemetria_float[1] = magneto_data.axis.y;
		  telemetria_float[2] = magneto_data.axis.z;

//		  telemetria_float[0] = euler.angle.roll;
//		  telemetria_float[1] = euler.angle.pitch;
//		  telemetria_float[2] = euler.angle.yaw;
		  xQueueSendToFront(telemetria_Queue, (void*)&telemetria_float, 0);



//		  set_duty_Oneshot42(&htim3, 550, 550, 550, 550);
		  set_duty_Oneshot42(&htim3, ref1, ref2, ref3, ref4);
	osDelay(3);
//	osDelay(48);
  }
  /* USER CODE END Start_Data_Reading */
}

/* USER CODE BEGIN Header_Start_Orientation */
/**
* @brief Function implementing the Orientation_cal thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_Orientation */
void Start_Orientation(void const * argument)
{
  /* USER CODE BEGIN Start_Orientation */
//	q.SEq_1=1;
//	q.SEq_2=0;
//	q.SEq_3=0;
//	q.SEq_4=0;
  /* Infinite loop */
  for(;;)
  {
//	  if(oricalc == 1){
//		//filterUpdateIMU(imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2], imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], &q );
//		filterUpdate(imu.gyr_rps[0], imu.gyr_rps[1], imu.gyr_rps[2], imu.acc_mps2[0], imu.acc_mps2[1], imu.acc_mps2[2], mag_data_y, -mag_data_x, mag_data_z, &q, &f);
//		eulerAngles(q, &roll, &pitch, &yaw);
//		oricalc = 0;
//	  }
    osDelay(1);
  }
  /* USER CODE END Start_Orientation */
}

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
