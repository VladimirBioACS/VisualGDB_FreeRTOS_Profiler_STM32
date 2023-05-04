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
#include "string.h"
#include "stdlib.h"
#include "stdio.h"
#include "stdbool.h" 
/* USER CODE END Includes */
#if (SYSPROG_PROFILER == 1)
#include <SysprogsProfiler.h>
#endif
/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

typedef struct {
	char msg_buf[120];
	uint16_t adc_val;
} QUEUE_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_RESOLUTION          (uint16_t)(4095)
#define ADC_VREF                (uint16_t)(3300)
#define TM35_MV_TO_C						(10)
#define LED_ISR_FLAG			(uint32_t)(0x51)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim13;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
	.name = "defaultTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for BlinkyTask */
osThreadId_t BlinkyTaskHandle;
const osThreadAttr_t BlinkyTask_attributes = {
	.name = "BlinkyTask",
	.stack_size = 128 * 4,
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for adcReadValueT */
osThreadId_t adcReadValueTHandle;
uint32_t adcReadValueTBuffer[128];
osStaticThreadDef_t adcReadValueTControlBlock;
const osThreadAttr_t adcReadValueT_attributes = {
	.name = "adcReadValueT",
	.cb_mem = &adcReadValueTControlBlock,
	.cb_size = sizeof(adcReadValueTControlBlock),
	.stack_mem = &adcReadValueTBuffer[0],
	.stack_size = sizeof(adcReadValueTBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for readTempT */
osThreadId_t readTempTHandle;
uint32_t myTask04Buffer[128];
osStaticThreadDef_t myTask04ControlBlock;
const osThreadAttr_t readTempT_attributes = {
	.name = "readTempT",
	.cb_mem = &myTask04ControlBlock,
	.cb_size = sizeof(myTask04ControlBlock),
	.stack_mem = &myTask04Buffer[0],
	.stack_size = sizeof(myTask04Buffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for pwmLedBrightnes */
osThreadId_t pwmLedBrightnesHandle;
uint32_t pwmLedBrightnesBuffer[128];
osStaticThreadDef_t pwmLedBrightnesControlBlock;
const osThreadAttr_t pwmLedBrightnes_attributes = {
	.name = "pwmLedBrightnes",
	.cb_mem = &pwmLedBrightnesControlBlock,
	.cb_size = sizeof(pwmLedBrightnesControlBlock),
	.stack_mem = &pwmLedBrightnesBuffer[0],
	.stack_size = sizeof(pwmLedBrightnesBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for usartDebugPrint */
osThreadId_t usartDebugPrintHandle;
uint32_t usartDebugPrintBuffer[128];
osStaticThreadDef_t usartDebugPrintControlBlock;
const osThreadAttr_t usartDebugPrint_attributes = {
	.name = "usartDebugPrint",
	.cb_mem = &usartDebugPrintControlBlock,
	.cb_size = sizeof(usartDebugPrintControlBlock),
	.stack_mem = &usartDebugPrintBuffer[0],
	.stack_size = sizeof(usartDebugPrintBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for checkISRStateT */
osThreadId_t checkISRStateTHandle;
uint32_t checkISRStateTBuffer[256];
osStaticThreadDef_t checkISRStateTControlBlock;
const osThreadAttr_t checkISRStateT_attributes = {
	.name = "checkISRStateT",
	.cb_mem = &checkISRStateTControlBlock,
	.cb_size = sizeof(checkISRStateTControlBlock),
	.stack_mem = &checkISRStateTBuffer[0],
	.stack_size = sizeof(checkISRStateTBuffer),
	.priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for rtosRuntimeStat */
osThreadId_t rtosRuntimeStatHandle;
uint32_t rtosRuntimeStatBuffer[128];
osStaticThreadDef_t rtosRuntimeStatControlBlock;
const osThreadAttr_t rtosRuntimeStat_attributes = {
	.name = "rtosRuntimeStat",
	.cb_mem = &rtosRuntimeStatControlBlock,
	.cb_size = sizeof(rtosRuntimeStatControlBlock),
	.stack_mem = &rtosRuntimeStatBuffer[0],
	.stack_size = sizeof(rtosRuntimeStatBuffer),
	.priority = (osPriority_t) osPriorityLow,
};
/* Definitions for msg_Queue */
osMessageQueueId_t msg_QueueHandle;
uint8_t msg_QueueBuffer[16 * sizeof(QUEUE_t)];
osStaticMessageQDef_t msg_QueueControlBlock;
const osMessageQueueAttr_t msg_Queue_attributes = {
	.name = "msg_Queue",
	.cb_mem = &msg_QueueControlBlock,
	.cb_size = sizeof(msg_QueueControlBlock),
	.mq_mem = &msg_QueueBuffer,
	.mq_size = sizeof(msg_QueueBuffer)
};
/* Definitions for ISREvent */
osEventFlagsId_t ISREventHandle;
osStaticEventGroupDef_t ISREventControlBlock;
const osEventFlagsAttr_t ISREvent_attributes = {
	.name = "ISREvent",
	.cb_mem = &ISREventControlBlock,
	.cb_size = sizeof(ISREventControlBlock),
};
/* USER CODE BEGIN PV */
#if (FREERTOS_PROFILER == 1)
volatile unsigned long ulHighFrequencyTimerTicks;
#endif

TaskHandle_t myTaskLedBlink = NULL;

osEventFlagsId_t EventGroup1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM13_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM10_Init(void);
static void MX_USART3_UART_Init(void);
void StartDefaultTask(void *argument);
void StartBlinkyTask(void *argument);
void adcReadValueTask(void *argument);
void readTempTask(void *argument);
void pwmLedBrightnessTask(void *argument);
void usartDebugPrintTask(void *argument);
void checkISRStateTask(void *argument);
void rtosRuntimeStatsTask(void *argument);

/* USER CODE BEGIN PFP */
void ledBlinkBlue(void *pvParameters);
static uint16_t getTemperature(uint16_t adc_raw_value);
static void printUsartMessage(char* message);

void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static TIM_HandleTypeDef s_TimerInstance = { 
    .Instance = TIM3
};
 
static unsigned g_TimerCounter;
 
extern void TIM3_IRQHandler()
{
    HAL_TIM_IRQHandler(&s_TimerInstance);
    g_TimerCounter++;
}
 
void StartDelayCountingTimer()
{
    __TIM3_CLK_ENABLE();
    s_TimerInstance.Init.Prescaler = 1;
    s_TimerInstance.Init.CounterMode = TIM_COUNTERMODE_UP;
    s_TimerInstance.Init.Period = 0xFFFF;
    s_TimerInstance.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    s_TimerInstance.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&s_TimerInstance);
    HAL_TIM_Base_Start_IT(&s_TimerInstance);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    HAL_NVIC_SetPriority(TIM3_IRQn, 7, 0);
}
 
unsigned long long SysprogsInstrumentingProfiler_ReadTimerValue()
{
    int primask = __get_PRIMASK();
    __set_PRIMASK(1);
    unsigned lowWord = __HAL_TIM_GET_COUNTER(&s_TimerInstance);
    unsigned highWord;
    if (lowWord < 1024)
    {
        highWord = g_TimerCounter;
        if (HAL_NVIC_GetPendingIRQ(TIM3_IRQn))
            highWord++;    
    }
    else
        highWord = g_TimerCounter;
    __set_PRIMASK(primask);
        
    return (((unsigned long long)highWord) << 16) | lowWord;
}
 
extern unsigned SysprogsInstrumentingProfiler_QueryAndResetPerformanceCounter()
{
    static unsigned long long s_PrevValue;
    unsigned long long value = SysprogsInstrumentingProfiler_ReadTimerValue();
    unsigned long long elapsed = value - s_PrevValue;
    s_PrevValue = value;
    if (elapsed > UINT32_MAX)
        return UINT32_MAX;
    else
        return (unsigned)elapsed;
}


#if (SWO_DEBUG == 1)

/* Override low-level _write system call */
int _write(int file, char *ptr, int len)
{
	/* Implement your write code here, this is used by puts and printf for example */
	int i = 0;
	for (i = 0; i < len; i++)
		ITM_SendChar((*ptr++));
	return len;
}

#endif
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
	//InitializeSamplingProfiler();
	//  vTraceEnable(TRC_START);

	  /* USER CODE END SysInit */

	  /* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_USB_OTG_FS_PCD_Init();
	MX_ADC1_Init();
	MX_TIM13_Init();
	MX_ADC2_Init();
	MX_TIM10_Init();
	MX_USART3_UART_Init();
	/* USER CODE BEGIN 2 */

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start(&hadc2);

	HAL_TIM_PWM_Start(&htim13, TIM_CHANNEL_1);


	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
#if (SYSPROG_PROFILER == 1)
	InitializeInstrumentingProfiler();
	StartDelayCountingTimer();

#endif
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the queue(s) */
	/* creation of msg_Queue */
	msg_QueueHandle = osMessageQueueNew(16, sizeof(QUEUE_t), &msg_Queue_attributes);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

	/* creation of BlinkyTask */
	BlinkyTaskHandle = osThreadNew(StartBlinkyTask, NULL, &BlinkyTask_attributes);

	/* creation of adcReadValueT */
	adcReadValueTHandle = osThreadNew(adcReadValueTask, NULL, &adcReadValueT_attributes);

	/* creation of readTempT */
	readTempTHandle = osThreadNew(readTempTask, NULL, &readTempT_attributes);

	/* creation of pwmLedBrightnes */
	pwmLedBrightnesHandle = osThreadNew(pwmLedBrightnessTask, NULL, &pwmLedBrightnes_attributes);

	/* creation of usartDebugPrint */
	usartDebugPrintHandle = osThreadNew(usartDebugPrintTask, NULL, &usartDebugPrint_attributes);

	/* creation of checkISRStateT */
	checkISRStateTHandle = osThreadNew(checkISRStateTask, NULL, &checkISRStateT_attributes);

	/* creation of rtosRuntimeStat */
	rtosRuntimeStatHandle = osThreadNew(rtosRuntimeStatsTask, NULL, &rtosRuntimeStat_attributes);

	/* USER CODE BEGIN RTOS_THREADS */

	EventGroup1 = osEventFlagsNew(NULL);
	/* add threads, ... */
	xTaskCreate(
		  ledBlinkBlue,
		"led_blinker",
		128,
		NULL,
		osPriorityLow,
		&myTaskLedBlink);

	/* USER CODE END RTOS_THREADS */

	/* Create the event(s) */
	/* creation of ISREvent */
	ISREventHandle = osEventFlagsNew(&ISREvent_attributes);

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
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure LSE Drive Capability
	*/
	HAL_PWR_EnableBkUpAccess();

	/** Configure the main internal regulator output voltage
	*/
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/** Initializes the RCC Oscillators according to the specified parameters
	* in the RCC_OscInitTypeDef structure.
	*/
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 15;
	RCC_OscInitStruct.PLL.PLLN = 144;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 5;
	RCC_OscInitStruct.PLL.PLLR = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	*/
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
	                            | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
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

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_10B;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_1;
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

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	*/
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc2.Init.ContinuousConvMode = ENABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;
	if (HAL_ADC_Init(&hadc2) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	*/
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

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
	htim10.Init.Prescaler = 0;
	htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim10.Init.Period = 956;
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
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

	/* USER CODE BEGIN TIM13_Init 0 */

	/* USER CODE END TIM13_Init 0 */

	TIM_OC_InitTypeDef sConfigOC = { 0 };

	/* USER CODE BEGIN TIM13_Init 1 */

	/* USER CODE END TIM13_Init 1 */
	htim13.Instance = TIM13;
	htim13.Init.Prescaler = 0;
	htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim13.Init.Period = 1023;
	htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim13) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM13_Init 2 */

	/* USER CODE END TIM13_Init 2 */
	HAL_TIM_MspPostInit(&htim13);

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
	huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart3) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USART3_Init 2 */

	/* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

	/* USER CODE BEGIN USB_OTG_FS_Init 0 */

	/* USER CODE END USB_OTG_FS_Init 0 */

	/* USER CODE BEGIN USB_OTG_FS_Init 1 */

	/* USER CODE END USB_OTG_FS_Init 1 */
	hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
	hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
	hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
	hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
	if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN USB_OTG_FS_Init 2 */

	/* USER CODE END USB_OTG_FS_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	  /* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, LD1_Pin | LD3_Pin | LD2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : USER_Btn_Pin */
	GPIO_InitStruct.Pin = USER_Btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
	GPIO_InitStruct.Pin = LD1_Pin | LD3_Pin | LD2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PowerSwitchOn_Pin */
	GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_OverCurrent_Pin */
	GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
#if (FREERTOS_PROFILER == 1)
void configureTimerForRunTimeStats(void)
{
	ulHighFrequencyTimerTicks = 0;
	HAL_TIM_Base_Start_IT(&htim10);
}

unsigned long getRunTimeCounterValue(void)
{
	return ulHighFrequencyTimerTicks;
}

#endif
/**
  * @brief User button IRQ callback function
  * @param uint16_t GPIO_Pin
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == USER_Btn_Pin)
	{
		HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		osEventFlagsSet(EventGroup1, LED_ISR_FLAG);
	}
}

/**
  * @brief calculates temperature from TM35 sensor ADC raw value
  * @param uint16_t adc_raw_value
  * @retval None
  */
static uint16_t getTemperature(uint16_t adc_raw_value)
{
	return (adc_raw_value / TM35_MV_TO_C); /* 10mV / 1C degree*/
}

/**
  * @brief print message via USART interface
  * @param const char* message
  * @retval None
  */
static void printUsartMessage(char* message)
{
	HAL_UART_Transmit(
	     &huart3,
		(uint8_t*)message,
		strlen(message),
		1);
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
	(void) argument;
	/* Infinite loop */
	for (;;)
	{
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartBlinkyTask */
/**
* @brief Function implementing the ledBlinkBlue thread (native FreeRTOS API).
* @param argument: Not used
* @retval None
*/
void ledBlinkBlue(void *pvParameters)
{
	(void) pvParameters;
	for (;;)
	{
		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		vTaskDelay(100);
	}
}

/**
* @brief Function implementing the BlinkyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartBlinkyTask */
void StartBlinkyTask(void *argument)
{
	/* USER CODE BEGIN StartBlinkyTask */
	(void) argument;
	/* Infinite loop */
	for (;;)
	{
		HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
		osDelay(500);
	}
	/* USER CODE END StartBlinkyTask */
}

/* USER CODE BEGIN Header_adcReadValueTask */
/**
* @brief Function implementing the adcReadValueT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_adcReadValueTask */
void adcReadValueTask(void *argument)
{
	/* USER CODE BEGIN adcReadValueTask */
	(void) argument;
	QUEUE_t message;
	/* Infinite loop */
	for (;;)
	{
		uint16_t adc_value = HAL_ADC_GetValue(&hadc1);

		message.adc_val = adc_value;

		osMessageQueuePut(msg_QueueHandle,
			&message,
			0,
			osWaitForever);

		osDelay(10);
	}
	/* USER CODE END adcReadValueTask */
}

/* USER CODE BEGIN Header_readTempTask */
/**
* @brief Function implementing the readTempT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readTempTask */
void readTempTask(void *argument)
{
	/* USER CODE BEGIN readTempTask */
	(void) argument;
	QUEUE_t message;
	/* Infinite loop */
	for (;;)
	{
		uint16_t adc_voltage = (ADC_VREF * (uint16_t)HAL_ADC_GetValue(&hadc2)) / ADC_RESOLUTION;
		uint16_t tempC = getTemperature(adc_voltage); /* 10mV / 1C degree*/

		itoa(tempC, message.msg_buf, 10);
		strcat(message.msg_buf, " C :[temperature value]\r\n\0");

		osMessageQueuePut(msg_QueueHandle,
			&message,
			0,
			osWaitForever);

		osDelay(10);
	}
	/* USER CODE END readTempTask */
}

/* USER CODE BEGIN Header_pwmLedBrightnessTask */
/**
* @brief Function implementing the pwmLedBrightnes thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_pwmLedBrightnessTask */
void pwmLedBrightnessTask(void *argument)
{
	/* USER CODE BEGIN pwmLedBrightnessTask */
	(void) argument;
	QUEUE_t message;

	/* Infinite loop */
	for (;;)
	{
		osMessageQueueGet(msg_QueueHandle,
			&message,
			0,
			osWaitForever);


		__HAL_TIM_SET_COMPARE(&htim13, TIM_CHANNEL_1, message.adc_val);


		itoa(message.adc_val, message.msg_buf, 10);
		strcat(message.msg_buf, " :[potentiometer value]\r\n\0");

		osMessageQueuePut(msg_QueueHandle,
			&message,
			0,
			osWaitForever);

		osDelay(10);
	}
	/* USER CODE END pwmLedBrightnessTask */
}

/* USER CODE BEGIN Header_usartDebugPrintTask */
/**
* @brief Function implementing the usartDebugPrint thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_usartDebugPrintTask */
void usartDebugPrintTask(void *argument)
{
	/* USER CODE BEGIN usartDebugPrintTask */
	(void) argument;
	QUEUE_t message;
	//	char msg[20] = "test\r\n\0";
	  /* Infinite loop */
	for (;;)
	{
		osMessageQueueGet(msg_QueueHandle,
			&message,
			0,
			osWaitForever);

		printUsartMessage(&message.msg_buf);

		osDelay(1);
	}
	/* USER CODE END usartDebugPrintTask */
}

/* USER CODE BEGIN Header_checkISRStateTask */
/**
* @brief Function implementing the checkISRStateT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_checkISRStateTask */
void checkISRStateTask(void *argument)
{
	/* USER CODE BEGIN checkISRStateTask */
	(void) argument;

	GPIO_PinState led_gpio_state;

	const char* led_on = "Pin state HIGH [Flag from ISR]\r\n\0";
	const char* led_off = "Pin state LOW [Flag from ISR]\r\n\0";
	const char* led_unknown = "Pin state UNKNOWN [Flag from ISR]\r\n\0";

	QUEUE_t message;
	/* Infinite loop */
	for (;;)
	{

		osEventFlagsWait(EventGroup1,
			LED_ISR_FLAG,
			osFlagsWaitAll,
			osWaitForever);

		led_gpio_state = HAL_GPIO_ReadPin(LD3_GPIO_Port, LD3_Pin);

		switch (led_gpio_state)
		{
		case GPIO_PIN_SET:
			strcpy(message.msg_buf, led_on);
			osMessageQueuePut(msg_QueueHandle,
				&message,
				0,
				osWaitForever);
			break;

		case GPIO_PIN_RESET:
			strcpy(message.msg_buf, led_off);
			osMessageQueuePut(msg_QueueHandle,
				&message,
				0,
				osWaitForever);
			break;

		default:
			strcpy(message.msg_buf, led_unknown);
			osMessageQueuePut(msg_QueueHandle,
				&message,
				0,
				osWaitForever);
			break;
		}

		osDelay(10);
	}
	/* USER CODE END checkISRStateTask */
}

/* USER CODE BEGIN Header_rtosRuntimeStatsTask */
/**
* @brief Function implementing the rtosRuntimeStat thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rtosRuntimeStatsTask */
void rtosRuntimeStatsTask(void *argument)
{
	/* USER CODE BEGIN rtosRuntimeStatsTask */
	/* Infinite loop */
	for (;;)
	{
		//	printUsartMessage(json_buf);
		osDelay(10);
	}
	/* USER CODE END rtosRuntimeStatsTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM8 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM8) {
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
