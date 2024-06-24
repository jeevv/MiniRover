/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int16.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

//I2C address of the MPU6050
#define MPU_ADDRESS 0x68
// Register to check if MPU is working
#define MPU_REG_AVAILABLE 0x75
// Register to turn on sensor and set clock rate
#define MPU_REG_PWR_MGMT_1 0x6b
// Register to reduce sampling rate
#define MPU_REG_SMPL_RT_DIV 0x19
// Accelerometer and gyroscope configuration register
#define MPU_REG_GYRO_CONFIG 0x1b
#define MPU_REG_ACC_CONFIG 0x1c
// Starting address of the six registers storing accelerometer values
#define MPU_REG_ACC_X_H 0x3b
// Starting address of the six registers storing gyroscope values
#define MPU_REG_GYRO_X_H 0x43
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 3000 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile int32_t LeftWheelEncoder=0;
volatile int32_t RightWheelEncoder=0;

double LeftWheelVelocity;
double RightWheelVelocity;

double LeftMotorSpeed;
double RightMotorSpeed;

const double Length = 0.225;    //distance between wheel and center of bot
const double WheelRadius = 0.07;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void subscription_cmd_vel_callback(const void * msgin)
{
	geometry_msgs__msg__Twist * msg = (geometry_msgs__msg__Twist *)msgin;

	LeftWheelVelocity = msg->linear.x - msg->angular.z*Length;
	RightWheelVelocity = msg->linear.x + msg->angular.z*Length;

	//w of motor in rpm
	LeftMotorSpeed = (int)(LeftWheelVelocity/WheelRadius) * 60/6.2831;
	RightMotorSpeed = (int)(RightWheelVelocity/WheelRadius) * 60/6.2831;

	//PWM2 Right motor PA6
	//PWM1 Right motor PA7
	//PWM2 Left motor PB1
	//PWM1 Left motor PB0

	// Fix the going beyond 1000 value in the ROS code

	if (LeftMotorSpeed>=0 && LeftMotorSpeed<=1000 && RightMotorSpeed>=0 && RightMotorSpeed<=1000)	//front
	{
 		TIM3->CCR1 = LeftMotorSpeed;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = RightMotorSpeed;
		TIM3->CCR4 = 0;
	}
	else if (LeftMotorSpeed<=0 && LeftMotorSpeed>=-1000 && RightMotorSpeed<=0 && RightMotorSpeed>=-1000)	//back
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = -LeftMotorSpeed;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = -RightMotorSpeed;
	}
	else if (LeftMotorSpeed<=0 && LeftMotorSpeed>=-1000 && RightMotorSpeed>=0 && RightMotorSpeed<=1000)		//left
	{
		TIM3->CCR1 = -LeftMotorSpeed;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = RightMotorSpeed;
	}
	else if (LeftMotorSpeed>=0 && LeftMotorSpeed<=1000 && RightMotorSpeed<=0 && RightMotorSpeed>=-1000)		//right
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = LeftMotorSpeed;
		TIM3->CCR3 = -RightMotorSpeed;
		TIM3->CCR4 = 0;
	}
	else
	{
		TIM3->CCR1 = 0;
		TIM3->CCR2 = 0;
		TIM3->CCR3 = 0;
		TIM3->CCR4 = 0;
	}

}

// Interrupts for the wheel encoders
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//PA0 LeftWheelEncoderChannelA  PE11 LeftWheelEncoderChannelB
	if (GPIO_Pin == GPIO_PIN_0)
	{
//		LeftWheelEncoder++;
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 1) LeftWheelEncoder++;
		else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == 0) LeftWheelEncoder--;

	}
	//PA1 RightWheelEncoderChannelA  PE12 RightWheelEncoderChannelB
	else if (GPIO_Pin == GPIO_PIN_1)
	{
//		RightWheelEncoder++;
		if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == 0) RightWheelEncoder++;
		else if (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_12) == 1) RightWheelEncoder--;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

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
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 9;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 5, 0);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PE11 PE12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
bool cubemx_transport_open(struct uxrCustomTransport * transport);
bool cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

void * microros_allocate(size_t size, void * state);
void microros_deallocate(void * pointer, void * state);
void * microros_reallocate(void * pointer, size_t size, void * state);
void * microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void * state);
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

	// Initialisation of MPU6050
	// Checking if sensor returns 0x68 to confirm correct functioning

	uint8_t check = 0;
	uint8_t data = 0;
	HAL_I2C_Mem_Read (&hi2c1, MPU_ADDRESS,MPU_REG_AVAILABLE,1, &check, 1, 1000);

	if (check!=0x68)
	{
		printf("MPU6050 connected incorrectly\n");
	}

	// Writing zeros to the register to wake up the sensor and
	// set clock frequency to 8 MHz

	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, MPU_REG_PWR_MGMT_1, 1,&data, 1, 1000);

	// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
	data = 0x07;
	HAL_I2C_Mem_Write(&hi2c1, MPU_ADDRESS, MPU_REG_SMPL_RT_DIV	, 1, &data, 1, 1000);

	// micro-ROS configuration

	rmw_uros_set_custom_transport(
	    true,
	    (void *) &huart2,
	    cubemx_transport_open,
	    cubemx_transport_close,
	    cubemx_transport_write,
	    cubemx_transport_read);

	 rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
	 freeRTOS_allocator.allocate = microros_allocate;
	 freeRTOS_allocator.deallocate = microros_deallocate;
	 freeRTOS_allocator.reallocate = microros_reallocate;
	 freeRTOS_allocator.zero_allocate =  microros_zero_allocate;

	 if (!rcutils_set_default_allocator(&freeRTOS_allocator)) {
	      printf("Error on default allocators (line %d)\n", __LINE__);
	  }

	  // micro-ROS app

	  rcl_publisher_t publisher1;
	  rcl_publisher_t publisher2;
	  rcl_publisher_t imu_publisher;
	  rcl_subscription_t subscriber_cmd_vel;
	  std_msgs__msg__Int16 msg1;
	  std_msgs__msg__Int16 msg2;
	  sensor_msgs__msg__Imu imu_data;
	  geometry_msgs__msg__Twist sub_cmd_vel_msg;
	  rclc_support_t support;
	  rcl_allocator_t allocator;
	  rcl_node_t node;

	  allocator = rcl_get_default_allocator();

	  //create init_options
	  rclc_support_init(&support, 0, NULL, &allocator);

	  // create node
	  rclc_node_init_default(&node, "stm32_microros", "", &support);

	  // create publisher
	  rclc_publisher_init_default(
	    &publisher1,
	    &node,
	    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
	    "lwheel");

	  rclc_publisher_init_default(
	  	&publisher2,
	  	&node,
	  	ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
	  	"rwheel");

	  rclc_publisher_init_default(
	  	&imu_publisher,
	  	&node,
	  	ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
	  	"Imu");

	  // create subscriber

	  rclc_subscription_init_default(
	  	     &subscriber_cmd_vel,
	  	     &node,
	  	     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
	  	     "cmd_vel");

	  // create executor
	  rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	  rclc_executor_init(&executor, &support.context, 2, &allocator);
	  rclc_executor_add_subscription(&executor, &subscriber_cmd_vel, &sub_cmd_vel_msg, &subscription_cmd_vel_callback, ON_NEW_DATA);

	  for(;;)
	  {

		// Reading values using IMU

		// Reading 6 elements of 1 byte each
		// x, y, z values occupy 16 bits each, divided into two registers for
		// higher and lower bytes
		uint8_t rec_data[6] = {0,0,0,0,0,0};
		HAL_I2C_Mem_Read (&hi2c1, MPU_ADDRESS, MPU_REG_ACC_X_H, 1, rec_data, 6, 1000);

		// Converting two separate 8-bit values into a single 16-bit value
		int16_t accel_x_raw = (int16_t)(rec_data[0] << 8 | rec_data [1]);
		int16_t accel_y_raw = (int16_t)(rec_data[2] << 8 | rec_data [3]);
		int16_t accel_z_raw = (int16_t)(rec_data[4] << 8 | rec_data [5]);

		// Dividing by 16384 to obtain actual value and storing it in Imu message
		imu_data.linear_acceleration.x = (double) accel_x_raw/16384.0;
		imu_data.linear_acceleration.y = (double) accel_y_raw/16384.0;
		imu_data.linear_acceleration.z = (double) accel_z_raw/16384.0;

		// Doing the same for the gyroscope values
		HAL_I2C_Mem_Read (&hi2c1, MPU_ADDRESS, MPU_REG_GYRO_X_H, 1, rec_data, 6, 1000);

		int16_t gyro_x_raw = (int16_t)(rec_data[0] << 8 | rec_data [1]);
		int16_t gyro_y_raw  = (int16_t)(rec_data[2] << 8 | rec_data [3]);
		int16_t gyro_z_raw  = (int16_t)(rec_data[4] << 8 | rec_data [5]);

		// Dividing by 131.0 to obtain actual values and storing in Imu message
		imu_data.angular_velocity.x = (double) gyro_x_raw/131.0;
		imu_data.angular_velocity.y = (double) gyro_y_raw/131.0;
		imu_data.angular_velocity.z = (double) gyro_z_raw/131.0;

		msg1.data = LeftWheelEncoder;
		msg2.data = RightWheelEncoder;

	    rcl_ret_t ret1 = rcl_publish(&publisher1, &msg1, NULL);
	    rcl_ret_t ret2 = rcl_publish(&publisher2, &msg2, NULL);
	    rcl_ret_t ret3 = rcl_publish(&imu_publisher, &imu_data, NULL);

	    // waits for 1000ns for ros data, theres no data it continues,
	    // if there is data then it executes subscription callback
	    rclc_executor_spin_some(&executor, 1000);

	    if ((ret1 | ret2 | ret3) != RCL_RET_OK)
	    {
	      printf("Error publishing (line %d)\n", __LINE__);
	    }
	    osDelay(10);
	  }

//	  rclc_executor_spin(&executor);
  /* USER CODE END 5 */
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
