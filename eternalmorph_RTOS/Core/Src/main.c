/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "mpu6050.h"
#include "madgwick.h"
#include "pid.h"
#include "lowpass.h"
#include "mixer.h"
#include "state_machine.h"
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float roll;
	float pitch;
	uint16_t m1, m2, m3, m4;
	uint32_t cycle_time;
}TelemetryData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
DMA_HandleTypeDef hdma_i2c1_rx;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart3;

/* Definitions for FlightTask */
osThreadId_t FlightTaskHandle;
const osThreadAttr_t FlightTask_attributes = {
  .name = "FlightTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TelemetryTask */
osThreadId_t TelemetryTaskHandle;
const osThreadAttr_t TelemetryTask_attributes = {
  .name = "TelemetryTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TelemetryQueue */
osMessageQueueId_t TelemetryQueueHandle;
const osMessageQueueAttr_t TelemetryQueue_attributes = {
  .name = "TelemetryQueue"
};
/* USER CODE BEGIN PV */
MPU6050_t mpu_data;          // Sensördeki tüm verileri tutacak çantamız
quaternion madgwick_data;
uint32_t loop_timer = 0;     // 250Hz kontrol döngüsü için kronometre
uint32_t telemetry_timer = 0;// 10Hz veri gönderimi için kronometre
float dt = 0.001f;             // Geçen süre (Delta Time)
char tx_buffer[128];         // Ekrana yazdıracağımız metin kutusu
uint32_t cycle_time;

volatile uint8_t imu_flag = 0; //bu değişkeni her seferinde RAM'den oku, aklında tutma, çünkü benim haberim olmadan değişebilir

PID_t pid_angle_roll; 		//Outer loop
PID_t pid_rate_roll; 		//Inner loop
float target_rate_roll = 0.0f; 		//Outer loop output
float roll_output = 0.0f; 		//Inner loop output

PID_t pid_angle_pitch; 		//Outer loop
PID_t pid_rate_pitch; 		//Inner loop
float target_rate_pitch = 0.0f; 	//Outer loop output
float pitch_output = 0.0f; 		//Inner loop output

PID_t pid_rate_yaw;
float target_rate_yaw = 0.0f;
float yaw_output = 0.0f;

LowPass_t lpf_gyro_x;
LowPass_t lpf_gyro_y;
LowPass_t lpf_gyro_z;
float gyro_x_filt = 0.0f;
float gyro_y_filt = 0.0f;
float gyro_z_filt = 0.0f;

Motor_Output_t motor_outputs;
RobotModel_t current_mode = MODE_AIR;					// Test mode: Air Mode
uint8_t is_armed = 1;									// Test mode: Motor lock disabled
float test_throttle = 1500.0f;							// Test mode: 50% throttle

FlightState_t state;
uint8_t btn_prev = GPIO_PIN_RESET;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM3_Init(void);
void StartFlightTask(void *argument);
void StartTelemetryTask(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DWT_Init(){
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

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
  MX_TIM6_Init();
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  DWT_Init();
  if (MPU6050_Init(&hi2c1) == 1) {
        char *msg = "Sensör Basariyla Uyandirildi!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
    } else {
        char *msg = "HATA: Sensör Bulunamadi!\r\n";
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), 100);
        while(1){
        	//LED BLINK
        }
    }
  MPU6050_Calibrate_Gyro(&hi2c1, &mpu_data);
  telemetry_timer = HAL_GetTick();							//Hal delay kullanmamak icin

  PID_Init(&pid_angle_roll, 4.0f, 0.0f, 0.0f, -300.0f, 300.0f, 0.0f);
  PID_Init(&pid_angle_pitch, 4.0f, 0.0f, 0.0f, -300.0f, 300.0f, 0.0f);

  PID_Init(&pid_rate_roll, 0.7f, 0.3f, 0.02f, -100.0f, 100.0f, 30.0f);
  PID_Init(&pid_rate_pitch, 0.7f, 0.3f, 0.02f, -100.0f, 100.0f, 30.0f);

  PID_Init(&pid_rate_yaw, 2.0f, 0.5f, 0.0f, -100.0f, 100.0f, 30.0f);

  LowPass_Init(&lpf_gyro_x, 90.0f, 1000.0f);
  LowPass_Init(&lpf_gyro_y, 90.0f, 1000.0f);
  LowPass_Init(&lpf_gyro_z, 90.0f, 1000.0f);

  Mixer_Init(&motor_outputs);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

  State_Init(&state, 60.0f, 50, 10);
  State_Arm(&state);


  Madgwick_Init(&madgwick_data, 0.04f);
  HAL_TIM_Base_Start_IT(&htim6);
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

  /* Create the queue(s) */
  /* creation of TelemetryQueue */
  TelemetryQueueHandle = osMessageQueueNew (1, sizeof(TelemetryData_t), &TelemetryQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of FlightTask */
  FlightTaskHandle = osThreadNew(StartFlightTask, NULL, &FlightTask_attributes);

  /* creation of TelemetryTask */
  TelemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &TelemetryTask_attributes);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  hi2c1.Init.Timing = 0x00501E6C;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
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
  htim3.Init.Prescaler = 119;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2499;
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
  htim6.Init.Prescaler = 119;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
extern uint8_t mpu_rx_buffer[32];
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){					//I2C Hafıza Okuma İşlemi Tamamlandı Fonksiyonu
	if(hi2c->Instance == I2C1){												// Gelen kargo I2C1'e (MPU6050) mi ait?
		SCB_InvalidateDCache_by_Addr((uint32_t*)mpu_rx_buffer, 32);   		//DCache (eski veriyi sil)
		MPU6050_Process_DMA_Data(&mpu_data);

		//FlightTask

		BaseType_t xHigherPriorityTaskWoken = pdFALSE;						// Yüksek öncelikli görev uyandı mı
		vTaskNotifyGiveFromISR(FlightTaskHandle,&xHigherPriorityTaskWoken);	// FlightTask'a bildirim ver (zile bas), bayrağı güncelle
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);						// Eski işe dönme, kesmeden çıkar çıkmaz anında FlightTask'a geç!
	}
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) {
        // Hattın kilitlendiğini anlarsak I2C'yi kapatıp yeniden kuruyoruz
        HAL_I2C_DeInit(hi2c);
        MX_I2C1_Init();
    }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartFlightTask */
/**
  * @brief  Function implementing the FlightTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartFlightTask */
void StartFlightTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	uint32_t last_dwt_time = DWT->CYCCNT;
  /* Infinite loop */
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);						//Veri gelene kadar uyu

    state.last_imu_tick = HAL_GetTick();

    uint32_t current_dwt_time = DWT->CYCCNT;
    dt = (float)(current_dwt_time - last_dwt_time) / (float)SystemCoreClock; // MHz işlemciye göre saniye cinsinden süre
    last_dwt_time = current_dwt_time;


    // İlk cycle'da dt boot süresini kapsıyor olabilir, clamp et
    if(dt > 0.01f) dt = 0.001f;   // Max 10ms, aksi halde 1ms varsay

    uint32_t dwt_start = DWT->CYCCNT;
    float deg_to_rad = 3.14159265f / 180.0f;

    gyro_x_filt = LowPass_Update(&lpf_gyro_x, mpu_data.gyro_x_dps);
    gyro_y_filt = LowPass_Update(&lpf_gyro_y, mpu_data.gyro_y_dps);
    gyro_z_filt = LowPass_Update(&lpf_gyro_z, mpu_data.gyro_z_dps);

    Madgwick_Update(&madgwick_data,
    	          mpu_data.gyro_x_dps * deg_to_rad,
    	          mpu_data.gyro_y_dps * deg_to_rad,
    	          mpu_data.gyro_z_dps * deg_to_rad,
    	          mpu_data.accel_x_g,
    	          mpu_data.accel_y_g,
    	          mpu_data.accel_z_g,
    	          dt);

    State_Update(&state, madgwick_data.roll, madgwick_data.pitch, dt);

    if(state.mode == STATE_ARMED){
    	      target_rate_roll = PID_Update(&pid_angle_roll, 0.0F, madgwick_data.roll, dt);
    	      roll_output = PID_Update(&pid_rate_roll, target_rate_roll, gyro_x_filt, dt);

    	      target_rate_pitch = PID_Update(&pid_angle_pitch, 0.0f, madgwick_data.pitch, dt);
    	      pitch_output = PID_Update(&pid_rate_pitch, target_rate_pitch, gyro_y_filt, dt);

    	      yaw_output = PID_Update(&pid_rate_yaw, target_rate_yaw, gyro_z_filt, dt);

    	      Mixer_Update(&motor_outputs, current_mode, test_throttle, roll_output, pitch_output, yaw_output, is_armed);

    	      __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1 , motor_outputs.m1);
    	      __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_2 , motor_outputs.m2);
    	      __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3 , motor_outputs.m3);
    	      __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 , motor_outputs.m4);
    	      }
    	      else{
    	    	  __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_1 , 1000);
    	    	  __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_2 , 1000);
    	    	  __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_3 , 1000);
    	    	  __HAL_TIM_SET_COMPARE(&htim3 , TIM_CHANNEL_4 , 1000);

    	    	  // PID integral birikimini sıfırla
    	    	  PID_Reset(&pid_angle_roll);
    	    	  PID_Reset(&pid_angle_pitch);
    	    	  PID_Reset(&pid_rate_roll);
    	    	  PID_Reset(&pid_rate_pitch);
    	    	  PID_Reset(&pid_rate_yaw);
    	    	  // Mixeri sıfırla
    	    	  Mixer_Init(&motor_outputs);

    }

    uint32_t dwt_finish = DWT->CYCCNT;
    cycle_time =(dwt_finish - dwt_start) / (SystemCoreClock / 1000000UL);

    TelemetryData_t t_data;

    t_data.roll = madgwick_data.roll;
    t_data.pitch = madgwick_data.pitch;
    t_data.m1 = motor_outputs.m1;
    t_data.m2 = motor_outputs.m2;
    t_data.m3 = motor_outputs.m3;
    t_data.m4 = motor_outputs.m4;
    t_data.cycle_time = cycle_time;

    xQueueOverwrite(TelemetryQueueHandle,&t_data);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTelemetryTask */
/**
* @brief Function implementing the TelemetryTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTelemetryTask */
void StartTelemetryTask(void *argument)
{
  /* USER CODE BEGIN StartTelemetryTask */
  TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));

    TelemetryData_t rx_data;

    if(xQueueReceive(TelemetryQueueHandle, &rx_data, 0) == pdPASS){
    	sprintf(tx_buffer, "R:%.1f | P:%.1f | M1:%u | M2:%u | M3:%u | M4:%u | CT:%lu us\r\n",
    	        rx_data.roll, rx_data.pitch,
    	        rx_data.m1, rx_data.m2, rx_data.m3, rx_data.m4,
    	        rx_data.cycle_time);

    	HAL_UART_Transmit(&huart3, (uint8_t*)tx_buffer, strlen(tx_buffer),10);
    	}
    }
  }
  /* USER CODE END StartTelemetryTask */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */
  if(htim->Instance == TIM6){
	  if(HAL_I2C_GetState(&hi2c1) == HAL_I2C_STATE_READY){
		  MPU6050_Read_All_DMA(&hi2c1); // timer tetiklendiginde hat dolu değilse veri oku
	  }
  }
  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

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
