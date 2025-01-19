/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "utils.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

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
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t tx_buffer[TX_BUFFER_SIZE];
//uint16_t uspu_buttons[USPU_BUTTON_SIZE];
//uint16_t uspu_leds[USPU_LED_SIZE];
//uint16_t acps[ACPS_SIZE];

void prepare_tx_buffer();

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim4;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern UART_HandleTypeDef huart4;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream4 global interrupt.
  */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
	// 1us - delay 6
	// 2us - delay 15

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

//  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);
//  delay(15);
//  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);

//  tx_data[30] = 0xa;
//  tx_data[31] = 0xd;
//	delay(100);
//	HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);

//  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);

//   delay(30);
//  ou_exchange();
//  HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);
  prepare_tx_buffer();


//	if (uart_tx_rq) {
//		HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);
//		prepare_tx_buffer();
//
//		HAL_UART_Transmit(&huart4, tx_buffer, TX_BUFFER_SIZE, HAL_MAX_DELAY);
//		// HAL_UART_Transmit(&huart4, tx_buffer, TX_BUFFER_SIZE, 0x10);
//
//		uart_tx_rq = 0;
//		HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);
//	}

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
//  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
//  HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
  /* USER CODE END UART4_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	  prepare_tx_buffer();
	uart_tx_rq = 1;
//	HAL_UART_Transmit_DMA(&huart4, tx_buffer, TX_BUFFER_SIZE);
//	HAL_UART_Receive_DMA(&huart4, rx_buffer, RX_BUFFER_SIZE);
}


void prepare_tx_buffer() {
	int i;
	int buffer_index = 0;

	int t1[USPU_BUTTON_SIZE] =  {0xa001, 0xa002, 0xa003, 0xa004, 0xa005, 0xa006, 0xa007, 0xa008, 0xa009};

	for (i=0; i<USPU_BUTTON_SIZE; i++) {
		uspu_buttons[i] = t1[i];
	}

	int t2[ACPS_SIZE] =  {0xb1fe, 0xb1ff, 0xb200, 0xb201, 0xb000, 0xbfff};

	for (i=0; i<ACPS_SIZE; i++) {
		acps[i] = t2[i];
	}

	int t3[OP_DATA_SIZE] =  {0xc001, 0xc002, 0xc003, 0xc004, 0xc005, 0xc006, 0xc007, 0xc008, 0xc009, 0xc00a, 0xc00b, 0xc00c, 0xc00d, 0xc00e, 0xc00f, 0xc010, 0xc011, 0xc012};

	for (i=0; i<OP_DATA_SIZE; i++) {
		op_data[i] = t3[i];
	}

	tx_buffer[0] = 0xad;
	tx_buffer[1] = 0xde;
	tx_buffer[2] = 0xef;
	tx_buffer[3] = 0xbe;
	buffer_index = 4;

	for (i=0;i<USPU_BUTTON_SIZE;i++) {
		tx_buffer[buffer_index] = uspu_buttons[i] & 0xff;
		buffer_index++;
		tx_buffer[buffer_index] = (uspu_buttons[i] >> 8) & 0xff;
		buffer_index++;
	}

	for (i=0;i<ACPS_SIZE;i++) {
		tx_buffer[buffer_index] = acps[i] & 0xff;
		buffer_index++;
		tx_buffer[buffer_index] = (acps[i] >> 8) & 0xff;
		buffer_index++;
	}

	for (i=0;i<OP_DATA_SIZE;i++) {
		tx_buffer[buffer_index] = op_data[i] & 0xff;
		buffer_index++;
		tx_buffer[buffer_index] = (op_data[i] >> 8) & 0xff;
		buffer_index++;
	}

	tx_buffer[TX_BUFFER_SIZE-4] = 0xad;
	tx_buffer[TX_BUFFER_SIZE-3] = 0xde;
	tx_buffer[TX_BUFFER_SIZE-2] = 0xef;
	tx_buffer[TX_BUFFER_SIZE-1] = 0xbe;
}
/* USER CODE END 1 */
