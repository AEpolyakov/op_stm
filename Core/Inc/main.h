/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "my_gpio.h"
#define USPU_START_ADDRESS 0x0
#define ACPS_START_ADDRESS 0x40
#define OP_START_ADDRESS 0xC0

#define USPU_BUTTON_SIZE 9
#define USPU_LED_SIZE 5
#define ACPS_SIZE 6
#define OP_DATA_SIZE 18

#define TX_BUFFER_SIZE 74
#define RX_BUFFER_SIZE 14

#define US_1 6
#define US_2 15

extern uint16_t uspu_buttons[USPU_BUTTON_SIZE];
extern uint16_t uspu_leds[USPU_LED_SIZE];
extern uint16_t acps[ACPS_SIZE];
extern uint16_t op_data[OP_DATA_SIZE];
extern uint16_t op_command;
extern uint16_t oo_delay;

extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint8_t rx_buffer[RX_BUFFER_SIZE];

extern uint8_t upr_zap_count;
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED0_Pin GPIO_PIN_2
#define LED0_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_3
#define LED1_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOE
#define UPRZAP_Pin GPIO_PIN_4
#define UPRZAP_GPIO_Port GPIOA
#define OO_Pin GPIO_PIN_5
#define OO_GPIO_Port GPIOA
#define D0_Pin GPIO_PIN_7
#define D0_GPIO_Port GPIOA
#define D1_Pin GPIO_PIN_0
#define D1_GPIO_Port GPIOB
#define D2_Pin GPIO_PIN_1
#define D2_GPIO_Port GPIOB
#define D3_Pin GPIO_PIN_2
#define D3_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define D13_Pin GPIO_PIN_10
#define D13_GPIO_Port GPIOB
#define D14_Pin GPIO_PIN_11
#define D14_GPIO_Port GPIOB
#define D15_Pin GPIO_PIN_12
#define D15_GPIO_Port GPIOB
#define DATA_DIR_Pin GPIO_PIN_13
#define DATA_DIR_GPIO_Port GPIOB
#define L_TI_Pin GPIO_PIN_11
#define L_TI_GPIO_Port GPIOA
#define L_SIN_IN_Pin GPIO_PIN_12
#define L_SIN_IN_GPIO_Port GPIOA
#define L_RW_Pin GPIO_PIN_10
#define L_RW_GPIO_Port GPIOC
#define L_IN_Pin GPIO_PIN_11
#define L_IN_GPIO_Port GPIOC
#define L_OUT_Pin GPIO_PIN_12
#define L_OUT_GPIO_Port GPIOC
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOD
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOD
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOD
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOD
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOD
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOD
#define A6_Pin GPIO_PIN_6
#define A6_GPIO_Port GPIOD
#define A7_Pin GPIO_PIN_7
#define A7_GPIO_Port GPIOD
#define A8_Pin GPIO_PIN_3
#define A8_GPIO_Port GPIOB
#define A9_Pin GPIO_PIN_4
#define A9_GPIO_Port GPIOB
#define L_TIOUT_Pin GPIO_PIN_5
#define L_TIOUT_GPIO_Port GPIOB
#define GT0_Pin GPIO_PIN_6
#define GT0_GPIO_Port GPIOB
#define LT0_Pin GPIO_PIN_7
#define LT0_GPIO_Port GPIOB
#define LLPIA_Pin GPIO_PIN_8
#define LLPIA_GPIO_Port GPIOB
#define LLPIB_Pin GPIO_PIN_9
#define LLPIB_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
