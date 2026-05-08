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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"  // porque si no, lo borra
#include "stdio.h"
#include "stdbool.h"
#include "ssd1306/ssd1306.h"
#include "stdio.h"
#include "CANSPI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN &hspi1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for led_parpadeo */
osThreadId_t led_parpadeoHandle;
const osThreadAttr_t led_parpadeo_attributes = {
  .name = "led_parpadeo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for recepcion_pseri */
osThreadId_t recepcion_pseriHandle;
const osThreadAttr_t recepcion_pseri_attributes = {
  .name = "recepcion_pseri",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for transmision_pse */
osThreadId_t transmision_pseHandle;
const osThreadAttr_t transmision_pse_attributes = {
  .name = "transmision_pse",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for motor_pp */
osThreadId_t motor_ppHandle;
const osThreadAttr_t motor_pp_attributes = {
  .name = "motor_pp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for lectura_temp */
osThreadId_t lectura_tempHandle;
const osThreadAttr_t lectura_temp_attributes = {
  .name = "lectura_temp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for trasmit_temp */
osThreadId_t trasmit_tempHandle;
const osThreadAttr_t trasmit_temp_attributes = {
  .name = "trasmit_temp",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for time_of_fly */
osThreadId_t time_of_flyHandle;
const osThreadAttr_t time_of_fly_attributes = {
  .name = "time_of_fly",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for servo */
osThreadId_t servoHandle;
const osThreadAttr_t servo_attributes = {
  .name = "servo",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for I2C_pantalla */
osThreadId_t I2C_pantallaHandle;
const osThreadAttr_t I2C_pantalla_attributes = {
  .name = "I2C_pantalla",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_TX */
osThreadId_t CAN_TXHandle;
const osThreadAttr_t CAN_TX_attributes = {
  .name = "CAN_TX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CAN_RX */
osThreadId_t CAN_RXHandle;
const osThreadAttr_t CAN_RX_attributes = {
  .name = "CAN_RX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for uart_rx_queue */
osMessageQueueId_t uart_rx_queueHandle;
const osMessageQueueAttr_t uart_rx_queue_attributes = {
  .name = "uart_rx_queue"
};
/* Definitions for uart_tx_sem */
osSemaphoreId_t uart_tx_semHandle;
const osSemaphoreAttr_t uart_tx_sem_attributes = {
  .name = "uart_tx_sem"
};
/* Definitions for CAN_sem */
osSemaphoreId_t CAN_semHandle;
const osSemaphoreAttr_t CAN_sem_attributes = {
  .name = "CAN_sem"
};
/* USER CODE BEGIN PV */
uint16_t frencled = 1000;
char motorPP_dir = 'D'; 			//direccion del motor paso a paso
uint16_t motorPP_tep  = 2 ; 		//tiempo entre pasos en ms
uint8_t motorPP_vel ;		        //velocidad recibida de la GIU
uint8_t data_uart_rx; 				// datos recibidos por la cola
float temperatura; 					// tipo float porque tiene decimales
uint16_t  periodo_temp; 			// periodo de lectura temp
uint8_t isFirstCapture = 1; 		// si es la primera captura, listo para ver el flanco de subida
float distancia;
int enviar = 0;						// para cuando se solicita el envio de temp
float posicion_servo = 0;
int espera_A = 2000;
char actuacion_servo = 0; 			//parametros que llegan cuando recibe A
int tpm_velocidad = 33;				//para que no se haga un osdelay(0)
uint8_t velo; 						// velocidad leida del puerto serie y que se manda directamente
int soy ;							// nos dice cuando se ha solicitado mensaje del can
bool rd_rec = false;
bool rd_status = false;
bool rd_tec = false;
bool soyA = false;
bool soyB = false;
bool soyAB = false;
uCAN_MSG txMessage;
uCAN_MSG rxMessage;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM14_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
void StartDefaultTask(void *argument);
void led_parpadeo_task(void *argument);
void recepcion_pserie_task(void *argument);
void transmision_pserie_task(void *argument);
void motor_pp_task(void *argument);
void lectura_temp_task(void *argument);
void transmit_temp_task(void *argument);
void time_of_fly_task(void *argument);
void servo_task(void *argument);
void I2C_pantalla_task(void *argument);
void CAN_TX_Task(void *argument);
void CAN_RX_Task(void *argument);

/* USER CODE BEGIN PFP */
void mi_delay_us(uint16_t delay);		// prototipo de la funcion
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
  MX_USART2_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  MX_TIM6_Init();
  MX_TIM14_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of uart_tx_sem */
  uart_tx_semHandle = osSemaphoreNew(1, 0, &uart_tx_sem_attributes);

  /* creation of CAN_sem */
  CAN_semHandle = osSemaphoreNew(1, 0, &CAN_sem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of uart_rx_queue */
  uart_rx_queueHandle = osMessageQueueNew (8, sizeof(uint8_t), &uart_rx_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of led_parpadeo */
  led_parpadeoHandle = osThreadNew(led_parpadeo_task, NULL, &led_parpadeo_attributes);

  /* creation of recepcion_pseri */
  recepcion_pseriHandle = osThreadNew(recepcion_pserie_task, NULL, &recepcion_pseri_attributes);

  /* creation of transmision_pse */
  transmision_pseHandle = osThreadNew(transmision_pserie_task, NULL, &transmision_pse_attributes);

  /* creation of motor_pp */
  motor_ppHandle = osThreadNew(motor_pp_task, NULL, &motor_pp_attributes);

  /* creation of lectura_temp */
  lectura_tempHandle = osThreadNew(lectura_temp_task, NULL, &lectura_temp_attributes);

  /* creation of trasmit_temp */
  trasmit_tempHandle = osThreadNew(transmit_temp_task, NULL, &trasmit_temp_attributes);

  /* creation of time_of_fly */
  time_of_flyHandle = osThreadNew(time_of_fly_task, NULL, &time_of_fly_attributes);

  /* creation of servo */
  servoHandle = osThreadNew(servo_task, NULL, &servo_attributes);

  /* creation of I2C_pantalla */
  I2C_pantallaHandle = osThreadNew(I2C_pantalla_task, NULL, &I2C_pantalla_attributes);

  /* creation of CAN_TX */
  CAN_TXHandle = osThreadNew(CAN_TX_Task, NULL, &CAN_TX_attributes);

  /* creation of CAN_RX */
  CAN_RXHandle = osThreadNew(CAN_RX_Task, NULL, &CAN_RX_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  osThreadSuspend(motor_ppHandle); //suspendo la tarea para que solo esté la de recepcion
  osThreadSuspend(time_of_flyHandle);
  osThreadSuspend(trasmit_tempHandle);
  osThreadSuspend(servoHandle);
  osThreadSuspend(lectura_tempHandle);
  osThreadSuspend(defaultTaskHandle);
  osThreadSuspend(transmision_pseHandle);
  osThreadSuspend(I2C_pantallaHandle);
  ssd1306_Fill(Black);
  ssd1306_UpdateScreen();
  osSemaphoreRelease(uart_tx_semHandle); // porqeu siempre se pone el semoforo a 0
  osThreadSuspend(CAN_RXHandle);
  osThreadSuspend(CAN_TXHandle);
  osSemaphoreRelease(CAN_semHandle);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
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
  hi2c2.Init.Timing = 0x2010091A;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 47;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 65535;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 47;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 19999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

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
  huart2.Init.BaudRate = 38400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, in2_Pin|in3_Pin|in1_Pin|trigger_Pin
                          |in4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|CAN1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CAN2_CS_GPIO_Port, CAN2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : boton_Pin */
  GPIO_InitStruct.Pin = boton_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(boton_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : in2_Pin in3_Pin in1_Pin trigger_Pin
                           in4_Pin */
  GPIO_InitStruct.Pin = in2_Pin|in3_Pin|in1_Pin|trigger_Pin
                          |in4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin CAN1_CS_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|CAN1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CAN2_CS_Pin */
  GPIO_InitStruct.Pin = CAN2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CAN2_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ican1_Pin */
  GPIO_InitStruct.Pin = Ican1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Ican1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	osMessageQueuePut(uart_rx_queueHandle, &data_uart_rx, 0, 0); // cojo de la cola Porque no se usa get
	HAL_UART_Receive_IT(huart, &data_uart_rx, 1);				// lectura de 1 byte para la isr
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	osSemaphoreRelease(uart_tx_semHandle);						//libera el semaforo
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	uint16_t adc_value = HAL_ADC_GetValue(hadc);
	HAL_ADC_Stop_IT(&hadc);
	osMessageQueuePut(lectura_tempHandle, &adc_value, 0, 0);
}
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == Ican1_Pin) {
		osSemaphoreRelease(CAN_semHandle);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3) {

		if (isFirstCapture == 1) {
			//Flanco de subida de la señal echo detectado

			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_FALLING);
		    __HAL_TIM_SET_COUNTER(htim, 0); //Establecer cuenta a 0
		    isFirstCapture = 0;

		}
		else {
			//Flanco de bajada de la señal echo detectado
			uint32_t tiempo_us = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_3);
			distancia = ((tiempo_us) * 0.03432) / 2.0;  // Vel. sonido 0.03432 cm/us
			__HAL_TIM_SET_CAPTUREPOLARITY(htim, TIM_CHANNEL_3, TIM_INPUTCHANNELPOLARITY_RISING);
			isFirstCapture = 1;
		}
	}
}

void mi_delay_us(uint16_t delay) {
	HAL_TIM_Base_Start(&htim6);
	uint16_t start = TIM6->CNT;  // contador de la cuenta del timmer 6
	uint16_t contador = start;

	while ((contador - start) < delay) { 		// mientras la diferencia sea menor que 10 sigue. cuando es 10 se para
		contador = TIM6->CNT;
	}
	HAL_TIM_Base_Stop(&htim6);
}

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_led_parpadeo_task */
/**
* @brief Function implementing the led_parpadeo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_led_parpadeo_task */
void led_parpadeo_task(void *argument)
{
  /* USER CODE BEGIN led_parpadeo_task */
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      osDelay(frencled);
  }
  /* USER CODE END led_parpadeo_task */
}

/* USER CODE BEGIN Header_recepcion_pserie_task */
/**
* @brief Function implementing the recepcion_pseri thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_recepcion_pserie_task */
void recepcion_pserie_task(void *argument)
{
  /* USER CODE BEGIN recepcion_pserie_task */
	//uint8_t i; //puntero
	HAL_UART_Receive_IT(&huart2, &data_uart_rx, 1); //lee interrupcion
	uint8_t command_rx[4];
	uint8_t i = 0;
	//int espera_A; // es global, se usa en el servo
	//uint8_t velo = 0; // global para mandar para la gui la velociadad
	//uint8_t vel;
	int cancelar = 0;
	uint8_t velo_ante;
  /* Infinite loop */
  for(;;)
  {
	  for (i=0; i<4; i++) 											/* va guardando en los diferentes byte de commando lo que recibe*/
	  {
		  osMessageQueueGet(uart_rx_queueHandle, &command_rx[i], 0, osWaitForever); //leo de la cola
	  }
	  if (command_rx[3]==0xe0 )						/*^comprueba que el ultimo elemento de la trama es correcto para que sea robusto*/
	  {														//metemos condicion de actualizar frecuencia de led
		  switch (command_rx[0]){
		  case 0x4C:						// activar led
		  	  frencled = ((uint16_t)command_rx[1]*256 + command_rx[2]);  //hacer con switch para practica 5 los estados del motor
//		  	  osThreadResume(led_parpadeoHandle);
		  	  break;							// sale del case
		  													//se puede en el if &&command_rx[0]
		  case 0x4D:
			  velo = command_rx[1];
			  velo_ante = velo;
			  motorPP_tep = 60000.0/(command_rx[1]*2048.0); // si la velocidad es 0 da un error
			  motorPP_vel = motorPP_tep;
			  osThreadResume(motor_ppHandle);
			  osThreadResume(transmision_pseHandle);
			  cancelar = 0;
			  break; // tiene que ser obligatorio para el case

		  case 0x44:
			  motorPP_dir = 'D';
			  motorPP_tep = motorPP_vel;
			  //velo = command_rx[1];
			  osThreadResume(motor_ppHandle);
			  osThreadResume(transmision_pseHandle);
			  if (cancelar == 1){
				  velo = velo_ante;
			  }else{
				  cancelar = 0;
			  }
			  //pone la velocidadn   que tenia de antes
	  		  break;

		  case 0x49:
			  motorPP_dir = 'I';
			  motorPP_tep = motorPP_vel;
			  if (cancelar == 1){
				  velo = velo_ante;
			  }else{
				  cancelar = 0;
			  }
			  //velo = command_rx[1];
			  osThreadResume(motor_ppHandle);
			  osThreadResume(transmision_pseHandle);
			  break;

		  case 0X45:
			  velo = 0;
			  cancelar = 1;
			  osThreadSuspend(motor_ppHandle);
			  //osThreadSuspend(lectura_tempHandle);			// se reactiva cuando llega una T
			  HAL_GPIO_WritePin(GPIOC, in1_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in2_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in3_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in4_Pin, 0);
			  break;

		  case 0x54:
			  periodo_temp = command_rx[1] <<8;
			  periodo_temp = periodo_temp + command_rx[2] ; //suma al valor anterior este valor
			  osThreadResume(lectura_tempHandle); // activa la tarea del adc
			  //osThreadResume(trasmit_tempHandle);	// que no la transmita hasta que no se solicita
			  break;


		  case 'S':   		//si es S
			  osThreadResume(servoHandle);
			  osThreadResume(transmision_pseHandle);  //no hace falta mandar nada
			  posicion_servo = command_rx[1];
			  actuacion_servo = 1;
			  break;

		  case 'A':
			  osThreadResume(servoHandle);
			  osThreadResume(transmision_pseHandle);  	//su hace falta mandar
			  espera_A = command_rx[1];					//lee la espera que viene
			  tpm_velocidad = (1.0/command_rx[2])*1000;	//lee la velocidad que llega
			  actuacion_servo = 2;
			  posicion_servo = 0;
			  break;									//es como si

		  case 0x30:
			  osThreadResume(lectura_tempHandle);		// si llega enviar, se activa el envio y lectura
			  osThreadResume(trasmit_tempHandle);		// va coger de periodo el que tuviese antes
			  break;

		  case 0x60:
			  enviar = 1;								// manda la distancia
			  osThreadResume(time_of_flyHandle);
			  osThreadResume(transmision_pseHandle);
			  break;

		  case 0x61:
			  enviar = 0;								// detiene el envio de la distancia
			  osThreadSuspend(time_of_flyHandle);
			  break;
		  case 0x58:
			  osThreadResume(I2C_pantallaHandle);		// activa la pantalla
			  break;
		  case 0x43:
			  velo = 0;
			  temperatura = 0.0;
			  soyA = false;
			  rd_status = false;
			  rd_rec = false;
			  osThreadSuspend(motor_ppHandle); 			// lo paramos all
			  ssd1306_Fill(Black);  					// limpia la pantalla
			  ssd1306_UpdateScreen();
			  osThreadSuspend(time_of_flyHandle);
			  osThreadSuspend(trasmit_tempHandle);
			  osThreadSuspend(servoHandle);
			  osThreadSuspend(lectura_tempHandle);
			  osThreadSuspend(defaultTaskHandle);
			  osThreadSuspend(transmision_pseHandle);
			  osThreadSuspend(I2C_pantallaHandle);
			  osThreadSuspend(CAN_TXHandle);
			  HAL_GPIO_WritePin(GPIOC, in1_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in2_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in3_Pin, 0);
			  HAL_GPIO_WritePin(GPIOC, in4_Pin, 0);
			  break;
		  case 0x52:									// soy a y quiero leer b
			  osThreadResume(CAN_RXHandle);
			  osThreadResume(CAN_TXHandle);
			  osThreadResume(transmision_pseHandle);
			  soyA = true;
			  //osThreadResume(lectura_tempHandle);
			  //osThreadResume(servoHandle);
			  //osThreadResume(motor_ppHandle);
		      //osThreadResume(time_of_flyHandle);
			  //osSemaphoreRelease(CAN_semHandle);
			  break;
		  case 0x55:									//mostrar estadso
			  osThreadResume(CAN_TXHandle);
			  rd_status = true;
			  rd_rec = true;
			  //rd_tec = true;
			  break;

		  default:
			  break;
			  }
	  }
		osDelay(100);

  }
  /* USER CODE END recepcion_pserie_task */
}

/* USER CODE BEGIN Header_transmision_pserie_task */
/**
* @brief Function implementing the transmision_pse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmision_pserie_task */
void transmision_pserie_task(void *argument)
{
  /* USER CODE BEGIN transmision_pserie_task */
	uint8_t trama_tx[4];
	trama_tx[1] = 0;							// para el contador del pulsador
	GPIO_PinState estado_boton;
	//uint8_t i;
  /* Infinite loop */
  for(;;)
  {
	  estado_boton = HAL_GPIO_ReadPin(boton_GPIO_Port, boton_Pin); 		//lee boton
	  if (!estado_boton){
		  trama_tx[0] = 0x08;
		  trama_tx[1]++;												//aumenta el contador
		  trama_tx[2] = 0xFF;
		  trama_tx[3] = 0xE0;
		  //for (i = 0 ; i < 4 ;i++){
			  osSemaphoreAcquire(uart_tx_semHandle, osWaitForever);		//coge semaforo
			  HAL_UART_Transmit_IT(&huart2, trama_tx, 4);		// manda pa la uart , 4 es los byte que manda
		  //}
	  }
	  osSemaphoreAcquire(uart_tx_semHandle, osWaitForever);			// todavia no se sabe si para el trabajo final hace falta
	  trama_tx[0] = 0x40;
	  trama_tx[1] = velo;	  								//Para enviar la velocidad del motor
	  trama_tx[2] = 0xFF;
	  trama_tx[3] = 0xE0;	  										//transmisión por la USART de 4 bytes con ISR
	  HAL_UART_Transmit_IT(&huart2, trama_tx, 4);

	  if (enviar == 1){												//variable que se pone a 1 si se pide mandar la distancia desde VS
		  osSemaphoreAcquire(uart_tx_semHandle, osWaitForever);			//enviar la distancia
		  trama_tx[0] = 0x50;
		  trama_tx[1] = distancia;
		  trama_tx[2] = 0xFF; //(distancia - trama_tx[1])*100;			// no se manda los decimales
		  trama_tx[3] = 0xE0;
	  	  //enviar = 0;	  	  	  	  	  	  	  	  	  	  // Lanzar transmisión por la USART de 4 bytes con ISR
	  	  HAL_UART_Transmit_IT(&huart2, trama_tx, 4);
	  }
	  osSemaphoreAcquire(uart_tx_semHandle, osWaitForever);
	  trama_tx[0] = 0x20;
	  trama_tx[1] = posicion_servo;
	  trama_tx[2] = 0xFF;
	  trama_tx[3] = 0xE0;
	  HAL_UART_Transmit_IT(&huart2, trama_tx, 4);

    osDelay(100);
  }
  /* USER CODE END transmision_pserie_task */
}

/* USER CODE BEGIN Header_motor_pp_task */
/**
* @brief Function implementing the motor_pp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_motor_pp_task */
void motor_pp_task(void *argument)
{
  /* USER CODE BEGIN motor_pp_task */
  /* Infinite loop */
	//osThreadSuspend(motor_ppHandle);
	char numfase = 1;
  for(;;)
  {
	  switch(numfase) {
	   case 1:
	   HAL_GPIO_WritePin(GPIOC,in1_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in2_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in3_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in4_Pin,0);
	   break;
	   case 2:
	   HAL_GPIO_WritePin(GPIOC,in1_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in2_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in3_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in4_Pin,0);
	   break;
	   case 3:
	   HAL_GPIO_WritePin(GPIOC,in1_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in2_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in3_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in4_Pin,1);
	   break;
	   case 4:
	   HAL_GPIO_WritePin(GPIOC,in1_Pin,1);
	   HAL_GPIO_WritePin(GPIOC,in2_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in3_Pin,0);
	   HAL_GPIO_WritePin(GPIOC,in4_Pin,1);
	   break;
	   }
	   if(motorPP_dir=='D')
	   {
	  numfase++;
	   }
	   if(numfase>=5){
	   numfase=1;
	   }
	   if(motorPP_dir=='I')
	   {
	   numfase--;
	   }
	   if(numfase<=0){
	   numfase=4;
	   }
    osDelay(motorPP_tep);
  }
  /* USER CODE END motor_pp_task */
}

/* USER CODE BEGIN Header_lectura_temp_task */
/**
* @brief Function implementing the lectura_temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_lectura_temp_task */
void lectura_temp_task(void *argument)
{
  /* USER CODE BEGIN lectura_temp_task */
	//calibraicon se hace con un hacl adcex_calibration
	// temp = adc_in_voltage * 1000 /10 porque mide en 10mv/ºC
	//mandar la parte entera es command[1]= temperatura
	//la parte decimal es command[2] = (temperatura - command[1]) * 100
	//ejercicio 3 por eventos, no hace falta hacerlo
	float adc_in_voltage;
	float adc_value;
	HAL_ADCEx_Calibration_Start(&hadc);
  /* Infinite loop */
  for(;;)
  {
	HAL_ADC_Start(&hadc);
	while(HAL_ADC_PollForConversion(&hadc, 1) != HAL_OK);
	adc_value = HAL_ADC_GetValue(&hadc);
	HAL_ADC_Stop(&hadc);
	adc_in_voltage = (adc_value/4095.0) * 3.3; //pasamos la lectura a tension
	temperatura = (adc_in_voltage*1000.0) /10.0 ; // de tension a valor numerico
    osDelay(100);
  }
  /* USER CODE END lectura_temp_task */
}

/* USER CODE BEGIN Header_transmit_temp_task */
/**
* @brief Function implementing the trasmit_temp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_transmit_temp_task */
void transmit_temp_task(void *argument)
{
  /* USER CODE BEGIN transmit_temp_task */
	uint8_t command_tx[4];
  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(uart_tx_semHandle, osWaitForever);
	  command_tx[0] = 0x30;
	  command_tx[1] = temperatura;
	  command_tx[2] = (temperatura - command_tx[1])*100;
	  command_tx[3] = 0xE0;
	  HAL_UART_Transmit_IT(&huart2, command_tx, 4);
      osDelay(periodo_temp);
  }
  /* USER CODE END transmit_temp_task */
}

/* USER CODE BEGIN Header_time_of_fly_task */
/**
* @brief Function implementing the time_of_fly thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_time_of_fly_task */
void time_of_fly_task(void *argument)
{
  /* USER CODE BEGIN time_of_fly_task */
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3);
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 1);
	  mi_delay_us(10);		// espera los 10 ms
	  HAL_GPIO_WritePin(trigger_GPIO_Port, trigger_Pin, 0);
	  osDelay(500); // titene que ser mayor a 1
  }
  /* USER CODE END time_of_fly_task */
}

/* USER CODE BEGIN Header_servo_task */
/**
* @brief Function implementing the servo thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_servo_task */
void servo_task(void *argument)
{
  /* USER CODE BEGIN servo_task */
	uint16_t ccr;
	float ta; // mide el tiempo en alto
	int estado = 0;
	HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);

  /* Infinite loop */
  for(;;)
  {
/*	  posicion_servo = 0;
	  ta = posicion_servo/90.0 + 1;			// esto esta entre 1 y 2ms
	  ccr = ta/20.0 *20000;					// lo dice el enunciado
	  htim14.Instance -> CCR1 = ccr;
	  osDelay(1000);

	  posicion_servo = 45;
	  ta = posicion_servo/90.0 + 1;			// esto esta entre 1 y 2ms
	  ccr = ta/20.0 *20000;					// lo dice el enunciado
	  htim14.Instance -> CCR1 = ccr;
	  osDelay(2000);

	  posicion_servo = 90;
	  ta = posicion_servo/90.0 + 1;			// esto esta entre 1 y 2ms
	  ccr = ta/20.0 *20000;					// lo dice el enunciado
	  htim14.Instance -> CCR1 = ccr;
	  osDelay(3000);
*/
	  if (actuacion_servo == 1)			// si hay que hacer el comando S
	  {
		  ta = posicion_servo/90.0 + 1;
		  ccr = ta/20.0*20000;
		  htim14.Instance -> CCR1 = ccr;
		  actuacion_servo  = 0; 			//solo se hace una vez, si no estaria constantenemente poniendolo a 10
		  osDelay(100);
	  }
	  else if (actuacion_servo == 2){
		  switch (estado)
		  {
		  	  case 0: 							//reposo
		  		  posicion_servo = 0;			//ponemos a 0 como posicion inicial
		  		  estado = 1;
		  		  osDelay(100);
		  	  break;
		  	  	  	  	  	  	  // el comando a manda la s que viene en segundos, se cambia a ms *1000
		  	  	  	  	  	  	  // tmp es 1/velociddad *1000

		  	  case 1:				// abrir
		  		  //posicion_servo = 0;
		  		  if (posicion_servo <90){
		  			  ta = posicion_servo/90 +1.0;
		  			  ccr = ta/20.0*20000;
		  			  htim14.Instance -> CCR1 = ccr;
		  			  posicion_servo = posicion_servo + 1; // cuando llegue a 90 cambia de estado
		  		  }
		  		  else			//cuando llega a 90
		  		  {
		  			  ta = posicion_servo/90 +1.0;
		  			  ccr = ta/20.0*20000;
		  			  htim14.Instance -> CCR1 = ccr;
		  			  estado = 2;
		  		  }
		  		  osDelay(tpm_velocidad);
		  		  break;
		  	  case 2:
		  		  osDelay(espera_A*1000);		//la espera que viene por el rx en ms
		  		  estado  = 3 ;
		  		  break;
		  	  case 3:
		  		//posicion_servo = 0;
		  		if (posicion_servo > 0)
		  		{
		  			ta = posicion_servo/90 +1.0;
		  			ccr = ta/20.0*20000;
		  			htim14.Instance -> CCR1 = ccr;
		  			posicion_servo = posicion_servo - 1; // cuando llegue a 90 cambia de estado
		  		}
		  		else			//cuando llega a 90
		  		{
		  			ta = posicion_servo/90 +1.0;
		  			ccr = ta/20.0*20000;
		  			htim14.Instance -> CCR1 = ccr;
		  			estado = 0;
		  			actuacion_servo = 0;
		  		}
		  		osDelay(tpm_velocidad);
		  		break;
		  	  default:
		  		  osDelay(50);
		  		  break;
		  }
	  }



    //osDelay(1);
  }
  /* USER CODE END servo_task */
}

/* USER CODE BEGIN Header_I2C_pantalla_task */
/**
* @brief Function implementing the I2C_pantalla thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_I2C_pantalla_task */
void I2C_pantalla_task(void *argument)
{
  /* USER CODE BEGIN I2C_pantalla_task */

	ssd1306_Init();
	char msg[100];
	uint8_t temperatura_entera;
	uint8_t temperatura_decimal;
  /* Infinite loop */
  for(;;)
  {
	  ssd1306_Fill(Black);										//color inicial

	  ssd1306_SetCursor(5, 20);
	  temperatura_entera = temperatura;
	  temperatura_decimal = (temperatura - temperatura_entera)*100;
	  sprintf(msg,"Temp: %d.%d Centigrados", temperatura_entera,temperatura_decimal);
	  ssd1306_WriteString(msg, Font_6x8, White);
	  //osDelay(periodo_temp); 			//esto hace que el resto tenga que espera este tiempo para actualizarse
	  // no se puede porqeu acumula el retraso
	  // si hago un if si tengo que actualizar, temp meto un osdelay temp dentro pero tambien la "bloqeua"
	  // si hago una maquina de estados estamos en las mismas
	  ssd1306_SetCursor(5, 30);
	  sprintf(msg,"V_Motor: %d rpm", velo);
	  ssd1306_WriteString(msg, Font_6x8, White);

	  ssd1306_SetCursor(5, 40);
	  sprintf(msg,"Servo: %d Grados", (uint8_t)posicion_servo);
	  ssd1306_WriteString(msg, Font_6x8, White);

	  ssd1306_SetCursor(5, 50);
	  sprintf(msg,"Dist. tof: %d cm", (int)distancia);
	  ssd1306_WriteString(msg, Font_6x8, White);


	  ssd1306_UpdateScreen();
	  osDelay(100);
  }
  /* USER CODE END I2C_pantalla_task */
}

/* USER CODE BEGIN Header_CAN_TX_Task */
/**
* @brief Function implementing the CAN_TX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_TX_Task */
void CAN_TX_Task(void *argument)
{
  /* USER CODE BEGIN CAN_TX_Task */
	uint8_t tx_buffer[4];
	//osSemaphoreRelease(uart_tx_semHandle);

	// los resume se hacen cuando se vea el id del mensaje que llega, a ver qué dato solicita

	if(CANSPI_Initialize(&hspi1, CAN1_CS_Pin))
	{
	osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);			//manda mensaje de que se ha incializado
	tx_buffer[0] = 0x13; //CAN1 initialized OK
	tx_buffer[1] = 0x0;
	tx_buffer[2] = 0x0;
	tx_buffer[3] = 0xE0;
	HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	}
	else{
	osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);			//manda mensaje de error
	tx_buffer[0] = 0x14; //Error CAN1 initialization
	tx_buffer[1] = 0x2;
	tx_buffer[2] = 0x0;
	tx_buffer[3] = 0xE0;
	HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	}
  /* Infinite loop */
  for(;;)
	  // tal como está, si quito el mensaje can funciona la inicializacion de ambos, y el envio de los estados de ambos
	  // 09/12 se envia la prueba por A, poner recepcion por b y una vez se reciba, entonces manda qeu se ha inicializado
	  // 10/12 se añade el envio de datos de b
  {

		if (soyA == true )
		{
		  	CANSPI_CL_Flag_Int(CAN, CAN1_CS_Pin);
		  	txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
		  	txMessage.frame.id = 0x15;
		  	txMessage.frame.dlc = 8;
		  	txMessage.frame.data0 = temperatura;
		  	txMessage.frame.data1 = (temperatura - txMessage.frame.data0)*100;
		  	txMessage.frame.data2 = posicion_servo;
		  	txMessage.frame.data3 = velo;
		  	txMessage.frame.data4 = distancia;
		  	txMessage.frame.data5 = 0;					//(distancia - txMessage.frame.data4)*100;
		  	txMessage.frame.data6 = 0xC;
	    	txMessage.frame.data7 = 0x1;
	    	osSemaphoreAcquire(CAN_semHandle, portMAX_DELAY);
	    	CANSPI_Transmit(CAN, &txMessage, CAN1_CS_Pin);
	    	osSemaphoreRelease(CAN_semHandle);
			osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);		// avisa de que ha mandao un mensaje al can
			tx_buffer[0] = 0x09; //CAN1 sending Msg...
			tx_buffer[1] = 0x1;
			tx_buffer[2] = 0x0;
			tx_buffer[3] = 0xE0;
		  	HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		}
	    if (rd_status == true ) {
	    	osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
	    	tx_buffer[0] = 0x10;
	    	tx_buffer[1] = CANSPI_isTxErrorPassive(CAN, CAN1_CS_Pin)*4+		// daba warning porque falta en el canpsi.h
	    				   CANSPI_isRxErrorPassive(CAN, CAN1_CS_Pin)*2+
						   CANSPI_isBusOff(CAN, CAN1_CS_Pin);
	    	tx_buffer[2] = 0;
	    	tx_buffer[3] = 0xE0;
	    	HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	    }
	    if (rd_rec == true) {
	    	osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
	    	tx_buffer[0] = 0x11;
	    	tx_buffer[1] = CANSPI_REC(CAN, CAN1_CS_Pin);
	    	tx_buffer[2] = CANSPI_TEC(CAN, CAN1_CS_Pin);
	    	tx_buffer[3] = 0xE0;
	    	HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
	    }
	    osDelay(1000);


  }
  /* USER CODE END CAN_TX_Task */
}

/* USER CODE BEGIN Header_CAN_RX_Task */
/**
* @brief Function implementing the CAN_RX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CAN_RX_Task */
void CAN_RX_Task(void *argument)
{
  /* USER CODE BEGIN CAN_RX_Task */
	uint8_t tx_buffer[4]  ;
	//osThreadResume(I2C_pantallaHandle);
	osThreadResume(uart_tx_semHandle);

  /* Infinite loop */
  for(;;)
  {
	  osSemaphoreAcquire(CAN_RXHandle, osWaitForever);
	  while(!CANSPI_Receive(&hspi1, &rxMessage, CAN2_CS_Pin));
	  osSemaphoreRelease(CAN_RXHandle);
	  if (rxMessage.frame.id == 0x25 && rxMessage.frame.data7 == 0x1)
	  {
		  temperatura = rxMessage.frame.data0 + (rxMessage.frame.data1/100);
		  posicion_servo = rxMessage.frame.data3;
		  velo = rxMessage.frame.data4;
		  distancia = rxMessage.frame.data5;

		  osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		  tx_buffer[0] = 0x21;
		  tx_buffer[1] = posicion_servo;			//la remota que me llega de b
		  tx_buffer[2] = 0xFF;
		  tx_buffer[3] = 0xE0;
		  HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		  osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		  tx_buffer[0] = 0x31;
		  tx_buffer[1] = temperatura;			//la remota que me llega de b
		  tx_buffer[2] = (temperatura - tx_buffer[1])/100;
		  tx_buffer[3] = 0xE0;
		  HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		  osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		  tx_buffer[0] = 0x41;
		  tx_buffer[1] = velo;			//la remota que me llega de b
		  tx_buffer[2] = 0xFF;
		  tx_buffer[3] = 0xE0;
		  HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));
		  osSemaphoreAcquire(uart_tx_semHandle, portMAX_DELAY);
		  tx_buffer[0] = 0x51;
		  tx_buffer[1] = distancia;			//la remota que me llega de b
		  tx_buffer[2] = 0xFF;				//distancia_decimal
		  tx_buffer[3] = 0xE0;
		  HAL_UART_Transmit_IT(&huart2, tx_buffer, sizeof(tx_buffer));

	  }
    osDelay(1);
  }
  /* USER CODE END CAN_RX_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
