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
#include "stdlib.h"
#include "stdio.h"
#include "string.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define enableUART 1
#define RX_BUFFER_SIZE 10
#define FREQ_MIN 10
#define FREQ_MAX 200
#define ARR_MAX 1000
#define ARR_MIN 60
#define CCR_MAX 80
#define CCR_MIN 20
#define PC_MAX 530
#define PC_MIN 250
#define TMR_COUNTER 16000
#define UART_DELAY 10
#define POWER_MIN 10

#define MIN_IGNITION_TIME 1000 // 1000 ms
#define maxIgnitionTime 2000 // 2000 ms
#define ULAMP_MAX 1100 // for open circuit detection / not used
#define ILAMP_IGNITED 600 //

#define UPPER_24VSUPPLY 3159 // 27V
#define LOWER_24VSUPPLY 2000 // 18V
#define UPPER_TEMP_MOSFET 500 // 400 mV = ca. 75 °C
#define UPPER_I_IN 2600

#define FOSC 16000 // 16 MHz
#define ditherRange_kHz 20 // 10 kHz

#define dutyIgnStart 60
#define dutyIgnMax 100
#define ignAmpltiudeStep 16

#define primInductance 20
#define peakCurrentControl 1
#define freqDithering 0

#define REPORT_STATE 1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

COMP_HandleTypeDef hcomp2;

DAC_HandleTypeDef hdac1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
enum States {
	INIT, RUN, IGNITE, IGN_FAIL, ERROR_state
};

volatile uint32_t arr_buffer;

//
volatile uint16_t ignFrequency = 320; // 58 kHz
volatile uint16_t operationFrequencyARR = 0; // 100 kHz // 164
volatile uint16_t operationFrequencyARR_raw = 160; //190 / 160 / 135 / 97 valley
volatile uint16_t currentFrequency = 0;
volatile uint16_t dac_IsenseMOS_ign = 1000; // current setpoint for ignition = off

volatile uint16_t dutyIgn;
volatile uint16_t dac_IsenseMOS; // current setpoint for COMP2 in- for closed loop
volatile uint16_t chargeTimeOperation; // duty cycle for operation open loop, optimized 78 für 26 uH, 80 für 33u

volatile uint16_t externalPowerSetDuty = 0;

volatile uint16_t externalPowerSetDutyCalc = 0;

char uart_rx_buffer[RX_BUFFER_SIZE];
char uart_rx_buffer_stripped[RX_BUFFER_SIZE];
uint8_t uart_index = 0;
uint8_t uart_rx_byte;
char msg[60];

int value = 0;
uint8_t uartEnableFlag = 1;
uint8_t powerLevel = 100; // linear dimming steps: 100 = full power, 15 = 15% power
// power setting by frequency control 0 - 105%, minimal power is 15% = 20 kHz
uint16_t freqPowerSetting[106] = { 1146, 1146, 1146, 1146, 1146, 1146, 1146,
		1146, 1146, 1146, 1065, 995, 934, 879, 831, 788, 750, 715, 683, 654,
		627, 603, 580, 559, 540, 522, 505, 489, 474, 460, 447, 435, 423, 412,
		402, 392, 382, 374, 365, 357, 349, 342, 335, 328, 321, 315, 309, 303,
		298, 293, 287, 283, 278, 273, 269, 265, 260, 256, 253, 249, 245, 242,
		238, 235, 232, 229, 226, 223, 220, 217, 214, 212, 209, 207, 204, 202,
		200, 197, 195, 193, 191, 189, 187, 185, 183, 181, 179, 178, 176, 174,
		172, 171, 169, 168, 166, 165, 163, 162, 160, 159, 158, 156, 155, 154,
		152, 151 };

uint16_t ignitionCounter = 0;
uint8_t ignitionFinishedFlag = 0;
uint16_t ignAmplitudeCounter = 0;
uint8_t lampOnFlag = 0;

uint16_t adc_uSenseLampIgnited = 1800;

uint16_t delayFailedIgnition = 5000;
uint8_t failedIgnitionCounter = 0;
uint8_t maxIgnitionAttempts = 4;

uint16_t adc_uSenseLampOpenCircuit = 2300;
uint8_t lampOCFlag = 0;

uint8_t i_ADCchannels = 0;

uint16_t ditherCounter = 0;
uint8_t ditherMode = freqDithering;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_COMP2_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_I2C2_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART2) {

		if (uart_rx_byte == '\n' || uart_rx_byte == '\r')  // End of input
				{
			uart_rx_buffer[uart_index] = '\0';

			if (uart_rx_buffer[0] == 'F') // Change frequency
					{
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if (value >= FREQ_MIN && value <= FREQ_MAX) { // limit frequency to between 10 - 120 kHz
					operationFrequencyARR_raw = (uint16_t) ((TMR_COUNTER
							+ value / 2) / value);
					snprintf(msg, sizeof(msg), "Freq:%d kHz ARR:%d\r\n", value,
							operationFrequencyARR_raw);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				}
			} else if (uart_rx_buffer[0] == 'O') // Change frequency dither mode
					{
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if (value < 2) {
					ditherMode = value;
				}
			} else if (uart_rx_buffer[0] == 'A') // Change ARR directly -> frequency
					{
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if (value >= ARR_MIN && value <= ARR_MAX) { // limit frequency to between 10 - 120 kHz
					operationFrequencyARR_raw = value;
					snprintf(msg, sizeof(msg), "ARR: %d Freq: %d\r\n", value,
							operationFrequencyARR_raw);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				}
			} else if (uart_rx_buffer[0] == 'C') // change CCR -> charge time
					{
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if (value >= CCR_MIN && value <= CCR_MAX) { // limit charge time to between 3 - 5 µs
					chargeTimeOperation = value;
					snprintf(msg, sizeof(msg), "Charge: %d\r\n", value);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				}

			} else if (uart_rx_buffer[0] == 'I') // change current limit
					{
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if (value >= PC_MIN && value <= PC_MAX) { // limit peak current
					dac_IsenseMOS = value;
					snprintf(msg, sizeof(msg), "Peak current: %d\r\n", value);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				}

			} else if (uart_rx_buffer[0] == 'D') // report ADC DATA
					{
				snprintf(msg, sizeof(msg),
						"Ui %04d, T %04d, Ul %04d, Il %04d, Ii %04d\r\n",
						adc_24V, adc_tempMOSFET, adc_uSenseLamp, adc_iSenseLamp,
						adc_iSenseIn);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						UART_DELAY);
			} else if (uart_rx_buffer[0] == 'T') // report TIMER data
					{
				value = (uint16_t) ((TMR_COUNTER + operationFrequencyARR / 2)
						/ operationFrequencyARR);
				snprintf(msg, sizeof(msg),
						"F: %d kHz, ARR: %d, CCR: %d, PeakCur: %d\r\n", value,
						operationFrequencyARR, chargeTimeOperation,
						dac_IsenseMOS);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						UART_DELAY);
			} else if (uart_rx_buffer[0] == 'X') // report external PWM data
					{
				snprintf(msg, sizeof(msg), "RE: %d , FE: %d, duty: %d \r\n",
						risingEdge, fallingEdge, externalPowerSetDuty);
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						UART_DELAY);
			} else if (uart_rx_buffer[0] == 'E') {  // On / OFF
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if ((value < 2) && (value >= 0)) {
					uartEnableFlag = value;
				}

			} else if (uart_rx_buffer[0] == 'P') {  // dimming levels
				strcpy(uart_rx_buffer_stripped, &uart_rx_buffer[1]);
				value = atoi(uart_rx_buffer_stripped);
				if ((value >= 0) && (value < 106)) {

					if (value < POWER_MIN) {
						operationFrequencyARR_raw = freqPowerSetting[POWER_MIN]; // clip lowest value
					} else {
						operationFrequencyARR_raw = freqPowerSetting[value];
					}
					snprintf(msg, sizeof(msg), "Set: %d percent\r\n", value);
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				} else {
					snprintf(msg, sizeof(msg), "Out of range!\r\n");
					HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
							UART_DELAY);
				}

			} else {
				snprintf(msg, sizeof(msg), "Wrong Command!\r\n");
				HAL_UART_Transmit(&huart2, (uint8_t*) msg, strlen(msg),
						UART_DELAY);
			}
			uart_index = 0;
		} else {
			if (uart_index < RX_BUFFER_SIZE - 1) {
				uart_rx_buffer[uart_index++] = uart_rx_byte;
			}
		}
		HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

	/* USER CODE BEGIN 1 */
	//
	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	if (primInductance == 26) {
		dutyIgn = dutyIgnStart;
		dac_IsenseMOS = 423;
		chargeTimeOperation = 68;
	} else if (primInductance == 15) {
		dutyIgn = dutyIgnStart;
		dac_IsenseMOS = 525; // 530
		chargeTimeOperation = 49;
	} else if (primInductance == 20) {
		dutyIgn = dutyIgnStart;
		dac_IsenseMOS = 420; // 490 / 420 / 380 /323 valley
		chargeTimeOperation = 60;
	}

	if (peakCurrentControl) {
		chargeTimeOperation = chargeTimeOperation + 5; // used as fall-back for maximum limiting
	} else {
		dac_IsenseMOS = dac_IsenseMOS + 100; // used as fall-back for maximum limiting
	}

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_COMP2_Init();
	MX_DAC1_Init();
	MX_TIM1_Init();
	MX_TIM3_Init();
	MX_TIM16_Init();
	MX_USART2_UART_Init();
	MX_TIM6_Init();
	MX_I2C2_Init();

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
	HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_IsenseMOS_ign); // disable current limit for init
	HAL_COMP_Start(&hcomp2); // start comparator for peak current control

	// Start ADC
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) adc_DMA, numberADCchannels); // start ADC with DMA, 6 channels
	/*
	 * 0 = Ch0: 24V
	 * 1 = Ch1: Temperature MOSFET
	 * 2 = Ch3: Usense_Lamp
	 * 3 = Ch8: Isense_Lamp
	 * 4 = Ch10: Isense_In
	 */

	HAL_UART_Receive_IT(&huart2, &uart_rx_byte, 1);

	// TIM3 for input capture - read PWM for power setting.
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // Primary channel - rising edge
	HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1); // Secondary channel - falling edge

	enum States state = INIT;

	// blink LED
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
	HAL_Delay(200);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		// ------------ Interrupts -----------------------
		// interrupt 100 Hz for external PWM read
		if (tim6_slowIrq_request) {
			if (risingEdge > 0) { // .. if external PWM method is used
				externalPowerSetDuty = (uint32_t) (fallingEdge * 100
						/ risingEdge) + 1;
				if (externalPowerSetDuty < 5) {
					uartEnableFlag = 0; // turn off if zero
				} else {
					uartEnableFlag = 1; // turn on for all other valuesw
					operationFrequencyARR_raw =
							freqPowerSetting[externalPowerSetDuty];
				}

			}

			tim6_slowIrq_request = 0;
		}

		// interrupt  1 kHz
		if (tim6_irq_request) {
			tim6_irq_request = 0;

			// Frequency dithering
			if (ditherMode) {
				currentFrequency = (uint16_t) (FOSC / operationFrequencyARR_raw
						- ditherRange_kHz / 2 + ditherCounter);
				operationFrequencyARR = (uint16_t) (FOSC / currentFrequency);
				if (ditherCounter > ditherRange_kHz + 1) {
					ditherCounter = 0;
				} else {
					ditherCounter++;
				}
			} else {
				operationFrequencyARR = operationFrequencyARR_raw;
			}

			// Check enable flag
			enableFlag = !HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) && uartEnableFlag; //pull low for enabling
			// check ignition
			if (enableFlag && !ignitionFinishedFlag) {
				ignitionCounter++;

				// dynamic ignition mode
				if (!lampOnFlag && dutyIgn < dutyIgnMax) {
					ignAmplitudeCounter++;
					if (ignAmplitudeCounter > ignAmpltiudeStep) {
						ignAmplitudeCounter = 0;
						dutyIgn++; // increase ignition voltage amplitude
					}
				}
			}

			// checked in all states, lower priority

			// check supply voltage
			if ((adc_24V < UPPER_24VSUPPLY) && (adc_24V > LOWER_24VSUPPLY)) {
				supplyOKFlag = 1;
			}
			else {
				supplyOKFlag = 0;
			}
			// check temperature

			if (adc_tempMOSFET > UPPER_TEMP_MOSFET) {
				OT_flag = 1;
			} else {
				OT_flag = 0;
			}

			// check input current
			if (adc_iSenseIn > UPPER_I_IN) {
				OCPinFlag = 1;
			} else {
				OCPinFlag = 0;
			}


		}

		// check in all states - high priority

		// ------------ ASM -----------------------
		switch (state) {
		case INIT:
			// init code
			ignitionFinishedFlag = 0;
			lampOnFlag = 0;
			ignitionCounter = 0;
			dutyIgn = dutyIgnStart;
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // status LED
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // UV LED

			// set DRV to zero
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			TIM1->ARR = ignFrequency;

			// exit conditions
			if (enableFlag && supplyOKFlag && !OT_flag) {
				//snprintf(msg, sizeof(msg), "IGNITE\r\n");
				//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_DELAY);
				state = IGNITE;
			}
			break;

		case IGNITE:

			if (failedIgnitionCounter > maxIgnitionAttempts) {
				state = ERROR_state;
			} else if (!ignitionFinishedFlag) {

				// ignition mode for one second
				if (ignitionCounter < maxIgnitionTime) { // try ignition
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET); // disable Status LED
					HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
							dac_IsenseMOS_ign); // disable current limit for init
					__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, dutyIgn); // high energy ignition
					TIM1->ARR = ignFrequency;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); // enable UV-LED
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // enable Status2 LED
					//if (adc_iSenseLamp > adc_iSenseLampIgnited && adc_uSenseLamp < adc_uSenseLampIgnited) {
					if (adc_iSenseLamp > ILAMP_IGNITED) {
						lampOnFlag = 1;
					}

					if (adc_iSenseLamp > ILAMP_IGNITED
							&& ignitionCounter > MIN_IGNITION_TIME) { // minimum ignition time 1000 ms
						//if ( ignitionCounter > 1000) { // minimum ignition time 1000 ms
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // disable UV-LED
						ignitionFinishedFlag = 1;
						//snprintf(msg, sizeof(msg), "RUN\r\n");
						//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_DELAY);
						state = RUN;
					}

				} else if (ignitionCounter > (maxIgnitionTime - 1)) { // ignition failed
					state = IGN_FAIL;
				}
			}
			if (!enableFlag) {
				state = INIT;
			}

			break;
		case IGN_FAIL:

			ignitionFinishedFlag = 0;
			//snprintf(msg, sizeof(msg), "IGNITION FAIL\r\n");
			//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_DELAY);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); // disable UV-LED
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // off
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_Delay(100);
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15);
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // disable Status2 LED
			HAL_Delay(delayFailedIgnition); // wait and restart
			failedIgnitionCounter++;
			state = INIT;
			break;

		case RUN:

			// RUN code
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET); // enable status LED
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // disable Status2 LED
			TIM1->ARR = operationFrequencyARR;
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, chargeTimeOperation); // chargingTime setpoint
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
					dac_IsenseMOS); // set current limiting value

			// exit conditions

			if (enableFlag == 0 || supplyOKFlag == 0) {
				state = INIT;
			}

			/*if (OT_flag || errorFlag || (adc_uSenseLamp>adc_uSenseLampOpenCircuit)) {
			 state = ERROR_state;
			 }*/
			if (OT_flag) {
				state = INIT;
			}

			break;
		case ERROR_state:

			// ERROR code
			// set DRV to zero
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			//snprintf(msg, sizeof(msg), "ERROR\r\n");
			//HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), UART_DELAY);

			// communicate error (blink LED, UART)
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_15); //  LED
			HAL_Delay(1000);
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
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
	RCC_OscInitStruct.PLL.PLLN = 8;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
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
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

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
	hadc1.Init.NbrOfConversion = 5;
	hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T6_TRGO;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = ADC_REGULAR_RANK_4;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = ADC_REGULAR_RANK_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
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
static void MX_COMP2_Init(void) {

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
	if (HAL_COMP_Init(&hcomp2) != HAL_OK) {
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
static void MX_DAC1_Init(void) {

	/* USER CODE BEGIN DAC1_Init 0 */

	/* USER CODE END DAC1_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC1_Init 1 */

	/* USER CODE END DAC1_Init 1 */

	/** DAC Initialization
	 */
	hdac1.Instance = DAC1;
	if (HAL_DAC_Init(&hdac1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC1_Init 2 */

	/* USER CODE END DAC1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.Timing = 0x00303D5B;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_ClearInputConfigTypeDef sClearInputConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 320;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sClearInputConfig.ClearInputState = ENABLE;
	sClearInputConfig.ClearInputSource = TIM_CLEARINPUTSOURCE_COMP2;
	if (HAL_TIM_ConfigOCrefClear(&htim1, &sClearInputConfig, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = 0;
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 0;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 0;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_SlaveConfigTypeDef sSlaveConfig = { 0 };
	TIM_IC_InitTypeDef sConfigIC = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 0;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_IC_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
	sSlaveConfig.InputTrigger = TIM_TS_TI2FP2;
	sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
	sSlaveConfig.TriggerFilter = 0;
	if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
	sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
	sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
	sConfigIC.ICFilter = 0;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
	if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM6_Init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 16;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 0;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 65535;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
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
static void MX_USART2_UART_Init(void) {

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
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LED2_Pin | LED_Pin | OCP_Reset_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5 | Status_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : MCU_Remote_Pin */
	GPIO_InitStruct.Pin = MCU_Remote_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(MCU_Remote_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : LED2_Pin LED_Pin OCP_Reset_Pin */
	GPIO_InitStruct.Pin = LED2_Pin | LED_Pin | OCP_Reset_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA4 OCP_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_4 | OCP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PA5 Status_Pin */
	GPIO_InitStruct.Pin = GPIO_PIN_5 | Status_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
