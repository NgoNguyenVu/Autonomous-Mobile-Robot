/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

#include "motor_control.h"

#include "usb_device.h"

#include "usbd_cdc_if.h"

#include "usb_ring_buffer.h"

#include "pid.h"

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
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;

/* USER CODE BEGIN PV */

volatile float final_target_fl = 0.0f, final_target_fr = 0.0f, final_target_bl = 0.0f, final_target_br = 0.0f;
volatile float current_target_fl = 0.0f, current_target_fr = 0.0f, current_target_bl = 0.0f, current_target_br = 0.0f;

volatile float actual_fl = 0.0f, actual_fr = 0.0f, actual_bl = 0.0f, actual_br = 0.0f;

volatile int32_t latest_delta_fl = 0, latest_delta_fr = 0, latest_delta_bl = 0, latest_delta_br = 0;
volatile uint8_t odom_data_ready = 0;

// Các biến lưu vận tốc đã được lọc
volatile float filtered_actual_fl = 0.0f;
volatile float filtered_actual_fr = 0.0f;
volatile float filtered_actual_bl = 0.0f;
volatile float filtered_actual_br = 0.0f;

#define FILTER_ALPHA 0.8f

// Biến lưu giá trị encoder cũ
int16_t prev_enc_fl = 0, prev_enc_fr = 0, prev_enc_bl = 0, prev_enc_br = 0;

// Khai báo 4 bộ điều khiển PID
PID_Controller pid_fl, pid_fr, pid_bl, pid_br;

// Các hằng số của robot (!!! QUAN TRỌNG: SỬA LẠI CHO ĐÚNG VỚI ROBOT CỦA BẠN !!!)
#define SAMPLING_TIME_S  0.02f   // Thời gian lấy mẫu 20ms (vì timer 50Hz)
#define PI               3.14159265359f
#define PPR              1320.0f // Pulse Per Revolution - Số xung/vòng của encoder
// Hằng số để chuyển từ xung/thời gian lấy mẫu sang rad/s
#define TICKS_TO_RAD_PER_S ((2.0f * PI) / (PPR * SAMPLING_TIME_S))

#define ACCELERATION_LIMIT 20.0f


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM5_Init(void);
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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_USB_DEVICE_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */

  // KHỞI TẠO PID

    PID_Init(&pid_fl);


    PID_Init(&pid_fr);


    PID_Init(&pid_bl);


    PID_Init(&pid_br);


  Motor_Init();

  // Start Encoders
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

  // Start PWM
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

  // Reset counters
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  __HAL_TIM_SET_COUNTER(&htim2, 0);
  __HAL_TIM_SET_COUNTER(&htim3, 0);
  __HAL_TIM_SET_COUNTER(&htim4, 0);

  // BẮT ĐẦU NGẮT TIMER PID
  HAL_TIM_Base_Start_IT(&htim5);


  char msg[128];


  char uart_buffer[128];
  uint8_t idx = 0;
  uint8_t byte;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


//    final_target_fl = 5.0f;
//  final_target_fr = 5.0f;
//  final_target_bl = 5.0f;
//    final_target_br = 5.0f;

  while (1)
  {
	  if (CDC_Receive_Byte(&byte))
	  	  {
			  if (byte == '\n')
			  {
				  uart_buffer[idx] = '\0';
				  if (uart_buffer[0] == 's') {
					  float temp_fl, temp_fr, temp_bl, temp_br;

					  if (sscanf(uart_buffer, "s,%f,%f,%f,%f",
								 &temp_fl, &temp_fr, &temp_bl, &temp_br) == 4)
					  {
						  __disable_irq();
						  final_target_fl = temp_fl;
						  final_target_fr = temp_fr;
						  final_target_bl = temp_bl;
						  final_target_br = temp_br;
						  __enable_irq();
					  }
				  }

				  // --- ⭐ THÊM ĐOẠN CODE NÀY VÀO ⭐ ---
				          else if (uart_buffer[0] == 'p') { // 'p' cho PID
				              float temp_kp, temp_ki, temp_kd;

				              // Parse lệnh, ví dụ: "p,3.5,0.15,0.02"
				              if (sscanf(uart_buffer, "p,%f,%f,%f", &temp_kp, &temp_ki, &temp_kd) == 3)
				              {
				                  // Cập nhật các thông số PID
				                  // QUAN TRỌNG: Phải tắt ngắt khi cập nhật
				                  // để tránh ngắt TIM5 đọc giá trị đang bị ghi dở.
				                  __disable_irq();
				                  pid_fl.Kp = temp_kp; pid_fl.Ki = temp_ki; pid_fl.Kd = temp_kd;
				                  pid_fr.Kp = temp_kp; pid_fr.Ki = temp_ki; pid_fr.Kd = temp_kd;
				                  pid_bl.Kp = temp_kp; pid_bl.Ki = temp_ki; pid_bl.Kd = temp_kd;
				                  pid_br.Kp = temp_kp; pid_br.Ki = temp_ki; pid_br.Kd = temp_kd;
				                  __enable_irq();

				                  // (Tùy chọn) Gửi lại tin nhắn xác nhận
				                  char ack_msg[64];
				                  sprintf(ack_msg, "OK: Kp=%.3f, Ki=%.3f, Kd=%.3f\n", temp_kp, temp_ki, temp_kd);
				                  CDC_Transmit_FS((uint8_t*)ack_msg, strlen(ack_msg));
				              }
				          }
				          // --- KẾT THÚC ĐOẠN CODE MỚI ---
	  			  idx = 0;
	  		  }
	  		  else if (idx < sizeof(uart_buffer) - 1)
	  		  {
	  			  uart_buffer[idx++] = byte;
	  		  }
	  	  }


	  if (odom_data_ready)
	            {
	                int32_t d_fl, d_fr, d_bl, d_br;

	                // Tắt ngắt để đọc dữ liệu an toàn
	                __disable_irq();
	                d_fl = latest_delta_fl;
	                d_fr = latest_delta_fr;
	                d_bl = latest_delta_bl;
	                d_br = latest_delta_br;
	                odom_data_ready = 0;
	                __enable_irq();

	                // Gửi "d" (delta) thay vì "e" (encoder)
	                sprintf(msg, "d,%ld,%ld,%ld,%ld\n", d_fl, d_fr, d_bl, d_br);
	                CDC_Transmit_FS((uint8_t*)msg, strlen(msg));

	            }

  /* USER CODE END 3 */
  }
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
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 192;
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
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
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
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 8399;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 199;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4799;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

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

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 4799;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 4799;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                          |GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15
                        |GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}


/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
    	// Định nghĩa giá trị max của counter 16-bit
    	const int32_t ENCODER_MAX_COUNT = 65536;

    	float max_velocity_change = ACCELERATION_LIMIT * SAMPLING_TIME_S;

    	current_target_fl += fmaxf(-max_velocity_change, fminf(max_velocity_change, final_target_fl - current_target_fl));
    	current_target_fr += fmaxf(-max_velocity_change, fminf(max_velocity_change, final_target_fr - current_target_fr));
    	current_target_bl += fmaxf(-max_velocity_change, fminf(max_velocity_change, final_target_bl - current_target_bl));
    	current_target_br += fmaxf(-max_velocity_change, fminf(max_velocity_change, final_target_br - current_target_br));

        // 1. ĐỌC ENCODER
        int16_t enc_fl = (int16_t)__HAL_TIM_GET_COUNTER(&htim1);
        int16_t enc_fr = (int16_t)__HAL_TIM_GET_COUNTER(&htim2);
        int16_t enc_bl = (int16_t)__HAL_TIM_GET_COUNTER(&htim3);
        int16_t enc_br = (int16_t)__HAL_TIM_GET_COUNTER(&htim4);

        // 2. TÍNH CHÊNH LỆCH
        int32_t delta_fl = enc_fl - prev_enc_fl;
        if (delta_fl > ENCODER_MAX_COUNT / 2) delta_fl -= ENCODER_MAX_COUNT;
        else if (delta_fl < -ENCODER_MAX_COUNT / 2) delta_fl += ENCODER_MAX_COUNT;

        int32_t delta_fr = enc_fr - prev_enc_fr;
        if (delta_fr > ENCODER_MAX_COUNT / 2) delta_fr -= ENCODER_MAX_COUNT;
		else if (delta_fr < -ENCODER_MAX_COUNT / 2) delta_fr += ENCODER_MAX_COUNT;

		int32_t delta_bl = enc_bl - prev_enc_bl;
		if (delta_bl > ENCODER_MAX_COUNT / 2) delta_bl -= ENCODER_MAX_COUNT;
		else if (delta_bl < -ENCODER_MAX_COUNT / 2) delta_bl += ENCODER_MAX_COUNT;

		int32_t delta_br = enc_br - prev_enc_br;
		if (delta_br > ENCODER_MAX_COUNT / 2) delta_br -= ENCODER_MAX_COUNT;
		else if (delta_br < -ENCODER_MAX_COUNT / 2) delta_br += ENCODER_MAX_COUNT;

        // 3. LƯU GIÁ TRỊ CŨ
        prev_enc_fl = enc_fl;
        prev_enc_fr = enc_fr;
        prev_enc_bl = enc_bl;
        prev_enc_br = enc_br;

        // 4. TÍNH VẬN TỐC THỰC TẾ (rad/s)

        actual_fl = (float)delta_fl * TICKS_TO_RAD_PER_S;

        actual_fr = (float)delta_fr * TICKS_TO_RAD_PER_S;

        actual_bl = (float)delta_bl * TICKS_TO_RAD_PER_S;

        actual_br = (float)delta_br * TICKS_TO_RAD_PER_S;

        const float ONE_MINUS_ALPHA = 1.0f - FILTER_ALPHA;

		filtered_actual_fl = (FILTER_ALPHA * filtered_actual_fl) + (ONE_MINUS_ALPHA * actual_fl);
		filtered_actual_fr = (FILTER_ALPHA * filtered_actual_fr) + (ONE_MINUS_ALPHA * actual_fr);
		filtered_actual_bl = (FILTER_ALPHA * filtered_actual_bl) + (ONE_MINUS_ALPHA * actual_bl);
		filtered_actual_br = (FILTER_ALPHA * filtered_actual_br) + (ONE_MINUS_ALPHA * actual_br);

        // 5. TÍNH TOÁN PID
        float pid_out_fl = PID_Compute(&pid_fl, current_target_fl, filtered_actual_fl, SAMPLING_TIME_S);
        float pid_out_fr = PID_Compute(&pid_fr, current_target_fr, filtered_actual_fr, SAMPLING_TIME_S);
        float pid_out_bl = PID_Compute(&pid_bl, current_target_bl, filtered_actual_bl, SAMPLING_TIME_S);
        float pid_out_br = PID_Compute(&pid_br, current_target_br, filtered_actual_br, SAMPLING_TIME_S);

        // 6. ĐIỀU KHIỂN MOTOR
        setMotorPWM(normalize_speed(pid_out_fl), MOTOR_FL);
        setMotorPWM(normalize_speed(pid_out_fr), MOTOR_FR);
        setMotorPWM(normalize_speed(pid_out_bl), MOTOR_BL);
        setMotorPWM(normalize_speed(pid_out_br), MOTOR_BR);

        __disable_irq();
        latest_delta_fl = delta_fl;
        latest_delta_fr = delta_fr;
        latest_delta_bl = delta_bl;
        latest_delta_br = delta_br;
        odom_data_ready = 1;
        __enable_irq();
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
#ifdef USE_FULL_ASSERT
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
