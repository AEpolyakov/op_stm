#include "main.h"
#include "ou.h"
#include "stm32f4xx_hal.h"
#include "utils.h"
#include "my_gpio.h"

void write_address(uint16_t address){
	uint16_t temp = ~address;

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, temp & GPIO_PIN_0 ? 1 : 0);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, temp & GPIO_PIN_1 ? 1 : 0);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, temp & GPIO_PIN_2 ? 1 : 0);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, temp & GPIO_PIN_3 ? 1 : 0);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, temp & GPIO_PIN_4 ? 1 : 0);
	HAL_GPIO_WritePin(A5_GPIO_Port, A5_Pin, temp & GPIO_PIN_5 ? 1 : 0);
	HAL_GPIO_WritePin(A6_GPIO_Port, A6_Pin, temp & GPIO_PIN_6 ? 1 : 0);
	HAL_GPIO_WritePin(A7_GPIO_Port, A7_Pin, temp & GPIO_PIN_7 ? 1 : 0);
	HAL_GPIO_WritePin(A8_GPIO_Port, A8_Pin, temp & GPIO_PIN_8 ? 1 : 0);
	HAL_GPIO_WritePin(A9_GPIO_Port, A9_Pin, temp & GPIO_PIN_9 ? 1 : 0);
}

void write_data(uint16_t data){
	uint16_t temp = ~data;

	HAL_GPIO_WritePin(D0_GPIO_Port,  D0_Pin,  temp & GPIO_PIN_0 ? 1 : 0);
	HAL_GPIO_WritePin(D1_GPIO_Port,  D1_Pin,  temp & GPIO_PIN_1 ? 1 : 0);
	HAL_GPIO_WritePin(D2_GPIO_Port,  D2_Pin,  temp & GPIO_PIN_2 ? 1 : 0);
	HAL_GPIO_WritePin(D3_GPIO_Port,  D3_Pin,  temp & GPIO_PIN_3 ? 1 : 0);
	HAL_GPIO_WritePin(D4_GPIO_Port,  D4_Pin,  temp & GPIO_PIN_4 ? 1 : 0);
	HAL_GPIO_WritePin(D5_GPIO_Port,  D5_Pin,  temp & GPIO_PIN_5 ? 1 : 0);
	HAL_GPIO_WritePin(D6_GPIO_Port,  D6_Pin,  temp & GPIO_PIN_6 ? 1 : 0);
	HAL_GPIO_WritePin(D7_GPIO_Port,  D7_Pin,  temp & GPIO_PIN_7 ? 1 : 0);
	HAL_GPIO_WritePin(D8_GPIO_Port,  D8_Pin,  temp & GPIO_PIN_8 ? 1 : 0);
	HAL_GPIO_WritePin(D9_GPIO_Port,  D9_Pin,  temp & GPIO_PIN_9 ? 1 : 0);
	HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, temp & GPIO_PIN_10 ? 1 : 0);
	HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, temp & GPIO_PIN_11 ? 1 : 0);
	HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, temp & GPIO_PIN_12 ? 1 : 0);
	HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, temp & GPIO_PIN_13 ? 1 : 0);
	HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, temp & GPIO_PIN_14 ? 1 : 0);
	HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, temp & GPIO_PIN_15 ? 1 : 0);

	write_out();
	delay(100);
}

uint16_t gpio_data_read(){
	uint16_t t = 0;
	t |= HAL_GPIO_ReadPin(D0_GPIO_Port,  D0_Pin ) << 0;
	t |= HAL_GPIO_ReadPin(D1_GPIO_Port,  D1_Pin ) << 1;
	t |= HAL_GPIO_ReadPin(D2_GPIO_Port,  D2_Pin ) << 2;
	t |= HAL_GPIO_ReadPin(D3_GPIO_Port,  D3_Pin ) << 3;
	t |= HAL_GPIO_ReadPin(D4_GPIO_Port,  D4_Pin ) << 4;
	t |= HAL_GPIO_ReadPin(D5_GPIO_Port,  D5_Pin ) << 5;
	t |= HAL_GPIO_ReadPin(D6_GPIO_Port,  D6_Pin ) << 6;
	t |= HAL_GPIO_ReadPin(D7_GPIO_Port,  D7_Pin ) << 7;
	t |= HAL_GPIO_ReadPin(D8_GPIO_Port,  D8_Pin ) << 8;
	t |= HAL_GPIO_ReadPin(D9_GPIO_Port,  D9_Pin ) << 9;
	t |= HAL_GPIO_ReadPin(D10_GPIO_Port, D10_Pin) << 10;
	t |= HAL_GPIO_ReadPin(D11_GPIO_Port, D11_Pin) << 11;
	t |= HAL_GPIO_ReadPin(D12_GPIO_Port, D12_Pin) << 12;
	t |= HAL_GPIO_ReadPin(D13_GPIO_Port, D13_Pin) << 13;
	t |= HAL_GPIO_ReadPin(D14_GPIO_Port, D14_Pin) << 14;
	t |= HAL_GPIO_ReadPin(D15_GPIO_Port, D15_Pin) << 15;
	return t;
}

void write_in(){
	HAL_GPIO_WritePin(L_IN_GPIO_Port, L_IN_Pin, GPIO_PIN_RESET);
	delay(15);
	HAL_GPIO_WritePin(L_IN_GPIO_Port, L_IN_Pin, GPIO_PIN_SET);
}

void write_out(){
	HAL_GPIO_WritePin(L_OUT_GPIO_Port, L_OUT_Pin, GPIO_PIN_RESET);
	delay(15);
	HAL_GPIO_WritePin(L_OUT_GPIO_Port, L_OUT_Pin, GPIO_PIN_SET);
}

void buffer_direction_out(){
	HAL_GPIO_WritePin(L_RW_GPIO_Port, L_RW_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(DATA_DIR_GPIO_Port, DATA_DIR_Pin, GPIO_PIN_SET);
	change_gpio_direction(D0_GPIO_Port,  D0_Pin,  1);
	change_gpio_direction(D1_GPIO_Port,  D1_Pin,  1);
	change_gpio_direction(D2_GPIO_Port,  D2_Pin,  1);
	change_gpio_direction(D3_GPIO_Port,  D3_Pin,  1);
	change_gpio_direction(D4_GPIO_Port,  D4_Pin,  1);
	change_gpio_direction(D5_GPIO_Port,  D5_Pin,  1);
	change_gpio_direction(D6_GPIO_Port,  D6_Pin,  1);
	change_gpio_direction(D7_GPIO_Port,  D7_Pin,  1);
	change_gpio_direction(D8_GPIO_Port,  D8_Pin,  1);
	change_gpio_direction(D9_GPIO_Port,  D9_Pin,  1);
	change_gpio_direction(D10_GPIO_Port, D10_Pin, 1);
	change_gpio_direction(D11_GPIO_Port, D11_Pin, 1);
	change_gpio_direction(D12_GPIO_Port, D12_Pin, 1);
	change_gpio_direction(D13_GPIO_Port, D13_Pin, 1);
	change_gpio_direction(D14_GPIO_Port, D14_Pin, 1);
	change_gpio_direction(D15_GPIO_Port, D15_Pin, 1);
}

void buffer_direction_in(){
	change_gpio_direction(D0_GPIO_Port,  D0_Pin,  0);
	change_gpio_direction(D1_GPIO_Port,  D1_Pin,  0);
	change_gpio_direction(D2_GPIO_Port,  D2_Pin,  0);
	change_gpio_direction(D3_GPIO_Port,  D3_Pin,  0);
	change_gpio_direction(D4_GPIO_Port,  D4_Pin,  0);
	change_gpio_direction(D5_GPIO_Port,  D5_Pin,  0);
	change_gpio_direction(D6_GPIO_Port,  D6_Pin,  0);
	change_gpio_direction(D7_GPIO_Port,  D7_Pin,  0);
	change_gpio_direction(D8_GPIO_Port,  D8_Pin,  0);
	change_gpio_direction(D9_GPIO_Port,  D9_Pin,  0);
	change_gpio_direction(D10_GPIO_Port, D10_Pin, 0);
	change_gpio_direction(D11_GPIO_Port, D11_Pin, 0);
	change_gpio_direction(D12_GPIO_Port, D12_Pin, 0);
	change_gpio_direction(D13_GPIO_Port, D13_Pin, 0);
	change_gpio_direction(D14_GPIO_Port, D14_Pin, 0);
	change_gpio_direction(D15_GPIO_Port, D15_Pin, 0);
	HAL_GPIO_WritePin(DATA_DIR_GPIO_Port, DATA_DIR_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(L_RW_GPIO_Port, L_RW_Pin, GPIO_PIN_SET);
}

void make_ti(){
	HAL_GPIO_WritePin(L_TIOUT_GPIO_Port, L_TIOUT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
	delay(US_1);
	HAL_GPIO_WritePin(L_TIOUT_GPIO_Port, L_TIOUT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
}

uint8_t pin_to_number(uint16_t GPIO_Pin) {
	switch(GPIO_Pin) {
		case GPIO_PIN_0: return 0;
		case GPIO_PIN_1: return 1;
		case GPIO_PIN_2: return 2;
		case GPIO_PIN_3: return 3;
		case GPIO_PIN_4: return 4;
		case GPIO_PIN_5: return 5;
		case GPIO_PIN_6: return 6;
		case GPIO_PIN_7: return 7;
		case GPIO_PIN_8: return 8;
		case GPIO_PIN_9: return 9;
		case GPIO_PIN_10: return 10;
		case GPIO_PIN_11: return 11;
		case GPIO_PIN_12: return 12;
		case GPIO_PIN_13: return 13;
		case GPIO_PIN_14: return 14;
		case GPIO_PIN_15: return 15;
		default: return 0;
	}
}

void change_gpio_direction(GPIO_TypeDef* GPIO_Port, uint16_t GPIO_Pin, uint8_t Mode)
{

	int pin_number = pin_to_number(GPIO_Pin);

    // Modify MODER register to change pin direction
    if (Mode == 1) {
        GPIO_Port->MODER &= ~(3 << (pin_number * 2));
        GPIO_Port->MODER |=  (1 << (pin_number * 2)); // Set pin as output with fast speed

        GPIO_Port->OTYPER &= ~(1 << pin_number);
        GPIO_Port->OTYPER |=  (0 << pin_number); // Push-pull mode

        GPIO_Port->OSPEEDR &= ~(3 << (pin_number * 2));
        GPIO_Port->OSPEEDR |=  (3 << (pin_number * 2)); // Fastest speed

        GPIO_Port->PUPDR &= ~(3 << (pin_number * 2));
        GPIO_Port->PUPDR |=  (1 << (pin_number * 2));

    } else if (Mode == 0) {
        // Set pin as input
        GPIO_Port->MODER &= ~(3 << (pin_number * 2));
    }
}
