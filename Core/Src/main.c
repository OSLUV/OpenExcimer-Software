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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "globals.h" // global variables
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define enableUART 0

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum States {INIT, RUN,  IGNITE, IGN_FAIL,  ERROR_state};

volatile uint32_t arr_buffer;

volatile uint16_t dac_IsenseMOS=1220; // current setpoint for COMP2 in-
volatile uint16_t dutyMaxIgn = 110; // max. duty cycle for ignition
volatile uint16_t dutyMax = 100; // max. duty cycle for operation
volatile uint16_t ignFrequency = 320; // 50 kHz
volatile uint16_t operationFrequency = 320; // 50 kHz

uint16_t ignitionCounter = 0;
uint8_t ignitionFlag = 0;
uint16_t adc_iSenseLampIgnited = 2500;
uint16_t adc_uSenseLampIgnited = 1800;
uint16_t maxIgnitionTime = 3000;
uint16_t delayFailedIgnition = 5000;
uint8_t failedIgnitionCounter = 0;
uint8_t maxIgnitionAttempts = 4;

uint16_t adc_uSenseLampOpenCircuit = 2300;
uint8_t lampOCFlag = 0;

uint8_t numberADCchannels = 6;
uint8_t i_ADCchannels = 0;

volatile uint16_t upper_24Vsupply = 3159; // 27V
volatile uint16_t lower_24Vsupply = 2256; // 19V
volatile uint16_t upper_tempMOSFET = 1100; // ca. 60°C   0,6V = 25°C 790 = 43 °C
volatile uint16_t upper_iIn = 2600; //

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


 void sendInt16UART() // UART Transmit
{


uint8_t buffer[36];
sprintf(buffer, "%04d, %04d, %04d, %04d, %04d, %04d\r\n", adc_buffer[0], adc_buffer[1], adc_buffer[2], adc_buffer[3], adc_buffer[4], adc_buffer[5]);

HAL_UART_Transmit(&huart2, buffer, 36, 10);

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{

	/* USER CODE BEGIN 1 */
	//
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
	HAL_Delay(100);
	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */
	HAL_Delay(100);
	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_COMP2_Init();
	MX_DAC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM16_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	/* USER CODE BEGIN 2 */

	// synchronous timing interrupts
	TIM6->ARR = 1000;
	HAL_TIM_Base_Start_IT(&htim6);

	// DRV PWM output
	TIM1->CCMR1 |= TIM_CCMR1_OC1CE; // enable OCREF clear
	TIM1->CR1 |= TIM_CR1_ARPE; // auto-reload preload
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // start PWM
	__HAL_TIM_MOE_ENABLE(&htim1); //master enable

	// Start DAC
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1); // DAC for current setpoint (intput to Comp2 in-)
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 4095); // disable current limit for init
	HAL_COMP_Start(&hcomp2); // start comparator for peak current control

	// Start ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, numberADCchannels); // start ADC with DMA, 6 channels
	/*
	 * 0 = Ch0: 24V
	 * 1 = Ch1: Temperature MOSFET
	 * 2 = Ch3: Usense_Lamp
	 * 3 = Ch8: Isense_Lamp
	 * 4 = Ch9: Lamp Intensity
	 * 5 = Ch10: Isense_In
	 */


	// alive PWM output
	//HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
	//HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2); // DRV Mask PWM output



	// TIM3 for input capture - read PWM for power setting.
	//HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // Primary channel - rising edge
	//HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1);    // Secondary channel - falling edge


	enum States state = INIT;

	// blink LED
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(500);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);


	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// ------------ Interrupts -----------------------
		// interrupt 1 Hz for UART send
		if (tim6_slowIrq_request && enableUART) {
			tim6_slowIrq_request = 0;
			sendInt16UART();
		}


		// interrupt  1 kHz
		if (tim6_irq_request) {
			tim6_irq_request = 0;

			// Check enable flag
			enableFlag = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9); //pull low for enabling
			// check ignition
			if (enableFlag && !ignitionFlag) {
				ignitionCounter++;
			}

			// checked in all states
			// check lamp voltage

			// check supply voltage
			if ((adc_24V < upper_24Vsupply) && (adc_24V > lower_24Vsupply)) {
				supplyOKFlag = 1;
			}
			else {
				supplyOKFlag = 0;
			}
			// check temperature
			if (adc_tempMOSFET > upper_tempMOSFET) {
				OT_flag = 1;
			}
			else {
				OT_flag = 0;
			}
			// check input current
			if (adc_iSenseIn > upper_iIn) {
				OCPinFlag = 1;
			}
			else {
				OCPinFlag = 0;
			}


		}

		// ------------ ASM -----------------------
		switch (state) {
		case INIT:
			// init code
			ignitionFlag = 0;
			ignitionCounter = 0;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // status LED
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // UV LED

			// set DRV to zero
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			TIM1->ARR=ignFrequency; //50 kHz

			// exit conditions
			if (errorFlag) {
				state = ERROR_state;
			}
			if (enableFlag  && supplyOKFlag ) {
				state = IGNITE;
			}
			break;

		case IGNITE:
			if (failedIgnitionCounter > maxIgnitionAttempts) {
				state = ERROR_state;
			}
			else if (!ignitionFlag) {

				// ignition mode for three seconds
				if (ignitionCounter < maxIgnitionTime) { // try ignition
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // disable Status LED
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyMaxIgn); // high energy ignition
					TIM1->ARR = ignFrequency;
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET); // enable UV-LED
					//if (adc_iSenseLamp > adc_iSenseLampIgnited && adc_uSenseLamp < adc_uSenseLampIgnited) {
					if (adc_iSenseLamp > adc_iSenseLampIgnited && ignitionCounter > 100) { // minimum ignition time 100 ms
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // disable UV-LED
						ignitionFlag = 1;
						state = RUN;
					}

					/*if (ignitionCounter > 1000) { // start checking after 100 ms
						if (OT_flag || errorFlag || (adc_uSenseLamp>adc_uSenseLampOpenCircuit)) {
							state = ERROR_state;
						}
					}*/

				}
				else if (ignitionCounter > (maxIgnitionTime-1)) { // ignition failed
					state = IGN_FAIL;
				}
			}
			if (!enableFlag) {
				state = INIT;
			}


			break;
		case IGN_FAIL:
			ignitionFlag = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // disable UV-LED
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // off
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(delayFailedIgnition); // wait and restart
			failedIgnitionCounter++;
			state = INIT;
			break;


		case RUN:
			// RUN code
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // enable status LED
			TIM1->ARR=operationFrequency;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyMax); // max. duty cycle limiting
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_IsenseMOS); // set current limiting value

			// exit conditions

			if (enableFlag == 0 || supplyOKFlag == 0) {
				state = INIT;
			}

			/*if (OT_flag || errorFlag || (adc_uSenseLamp>adc_uSenseLampOpenCircuit)) {
				state = ERROR_state;
			}*/
			if (OT_flag ) {
				state = ERROR_state;
			}


			break;
		case ERROR_state:
			// ERROR code
			// set DRV to zero
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);


			// communicate error (blink LED, UART)
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); //  LED
			HAL_Delay(200);
			// no exit conditions, only power cycle
			break;
		}

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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void)
{
	/* TIM6_DAC_LPTIM1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM6_DAC_LPTIM1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_LPTIM1_IRQn);
	/* ADC1_COMP_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_COMP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC1_COMP_IRQn);
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* TIM3_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM3_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM3_IRQn);
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
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV10;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 6;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_7CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_7CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_6;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief COMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP2_Init(void)
{

	/* USER CODE BEGIN COMP2_Init 0 */

	/* USER CODE END COMP2_Init 0 */

	/* USER CODE BEGIN COMP2_Init 1 */

	/* USER CODE END COMP2_Init 1 */
	hcomp2.Instance = COMP2;
	hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC1_CH1;
	hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp2.Init.WindowOutput = COMP_WINDOWOUTPUT_EACH_COMP;
	hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp2.Init.Mode = COMP_POWERMODE_HIGHSPEED;
	hcomp2.Init.WindowMode = COMP_WINDOWMODE_DISABLE;
	hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN COMP2_Init 2 */

	/* USER CODE END COMP2_Init 2 */

}

/**
 * @brief DAC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC1_Init(void)
{

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = {0};

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

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

	TIM_ClearInputConfigTypeDef sClearInputConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIMEx_BreakInputConfigTypeDef sBreakInputConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 320;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
	{
		Error_Handler();
	}
	sClearInputConfig.ClearInputState = ENABLE;
	sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_COMP2;
	if (HAL_TIM_ConfigOCrefClear(&htim1, &sClearInputConfig, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_BKIN;
	sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_DISABLE;
	sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
	if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK, &sBreakInputConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

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

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 1000;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
	if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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

	TIM_SlaveConfigTypeDef sSlaveConfig = {0};
	TIM_IC_InitTypeDef sConfigIC = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
	{
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
	htim6.Init.Prescaler = 16;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
	{
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void)
{

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	TIM_IC_InitTypeDef sConfigIC = {0};

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 65535;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_TIM_IC_Init(&htim16) != HAL_OK)
	{
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim16, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
	{
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

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
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
	{
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
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
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Status_GPIO_Port, Status_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(UV_LED_GPIO_Port, UV_LED_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : MCU_Remote_Pin */
	GPIO_InitStruct.Pin = MCU_Remote_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MCU_Remote_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Status_Pin */
	GPIO_InitStruct.Pin = Status_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Status_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : GPIO_IN_ALIVE_Pin */
	GPIO_InitStruct.Pin = GPIO_IN_ALIVE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIO_IN_ALIVE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : UV_LED_Pin */
	GPIO_InitStruct.Pin = UV_LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(UV_LED_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
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
		HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
		HAL_Delay(500);
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
