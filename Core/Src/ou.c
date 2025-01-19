#include "main.h"
#include "ou.h"
#include "stm32f4xx_hal.h"
#include "utils.h"

void ou_exchange() {
	uint8_t address;

	for(address = 0; address < USPU_BUTTON_SIZE; address++) {
		uspu_buttons[address] = read_word(address + 0x21);
	}

	for(address = 0; address < ACPS_SIZE; address++) {
		acps[address] = read_word(address + 0x41);
	}

	for(address = 0; address < OP_DATA_SIZE; address++) {
		op_data[address] = read_word(address + 0xc1);
	}

	for(address = 0; address < USPU_LED_SIZE; address++) {
		write_word(uspu_leds[address], address + 0x21);
	}

	for(address = 0; address < OP_COMMAND_SIZE; address++) {
		write_word(op_command[address], address + 0x01);
	}
}


void write_word(uint16_t data, uint16_t address) {
	write_address(address);
	write_data(data);
}

uint16_t read_word(uint16_t address) {
	uint16_t t;

	write_address(address);
	t = read_data();

	return t;
}

void write_address(uint16_t address){
	uint16_t temp = ~address;

	HAL_GPIO_WritePin(A0_GPIO_Port, A0_Pin, temp && GPIO_PIN_0);
	HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, temp && GPIO_PIN_1);
	HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, temp && GPIO_PIN_2);
	HAL_GPIO_WritePin(A3_GPIO_Port, A3_Pin, temp && GPIO_PIN_3);
	HAL_GPIO_WritePin(A4_GPIO_Port, A4_Pin, temp && GPIO_PIN_4);
	HAL_GPIO_WritePin(A5_GPIO_Port, A5_Pin, temp && GPIO_PIN_5);
	HAL_GPIO_WritePin(A6_GPIO_Port, A6_Pin, temp && GPIO_PIN_6);
	HAL_GPIO_WritePin(A7_GPIO_Port, A7_Pin, temp && GPIO_PIN_7);
	HAL_GPIO_WritePin(A8_GPIO_Port, A8_Pin, temp && GPIO_PIN_8);
	HAL_GPIO_WritePin(A9_GPIO_Port, A9_Pin, temp && GPIO_PIN_9);
}

void write_data(uint16_t data){
	uint16_t temp = ~data;

	buffer_direction_out();

	HAL_GPIO_WritePin(D0_GPIO_Port,  D0_Pin,  temp && GPIO_PIN_0 );
	HAL_GPIO_WritePin(D1_GPIO_Port,  D1_Pin,  temp && GPIO_PIN_1 );
	HAL_GPIO_WritePin(D2_GPIO_Port,  D2_Pin,  temp && GPIO_PIN_2 );
	HAL_GPIO_WritePin(D3_GPIO_Port,  D3_Pin,  temp && GPIO_PIN_3 );
	HAL_GPIO_WritePin(D4_GPIO_Port,  D4_Pin,  temp && GPIO_PIN_4 );
	HAL_GPIO_WritePin(D5_GPIO_Port,  D5_Pin,  temp && GPIO_PIN_5 );
	HAL_GPIO_WritePin(D6_GPIO_Port,  D6_Pin,  temp && GPIO_PIN_6 );
	HAL_GPIO_WritePin(D7_GPIO_Port,  D7_Pin,  temp && GPIO_PIN_7 );
	HAL_GPIO_WritePin(D8_GPIO_Port,  D8_Pin,  temp && GPIO_PIN_8 );
	HAL_GPIO_WritePin(D9_GPIO_Port,  D9_Pin,  temp && GPIO_PIN_9 );
	HAL_GPIO_WritePin(D10_GPIO_Port, D10_Pin, temp && GPIO_PIN_10);
	HAL_GPIO_WritePin(D11_GPIO_Port, D11_Pin, temp && GPIO_PIN_11);
	HAL_GPIO_WritePin(D12_GPIO_Port, D12_Pin, temp && GPIO_PIN_12);
	HAL_GPIO_WritePin(D13_GPIO_Port, D13_Pin, temp && GPIO_PIN_13);
	HAL_GPIO_WritePin(D14_GPIO_Port, D14_Pin, temp && GPIO_PIN_14);
	HAL_GPIO_WritePin(D15_GPIO_Port, D15_Pin, temp && GPIO_PIN_15);

	write_out();
	delay(500);
	buffer_direction_in();
}

/** Read byte from gpio  */
uint16_t read_data(){
	uint16_t t;

	buffer_direction_in();

	write_in();
	wait_sin();

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

	return ~t;
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
}

void buffer_direction_in(){
	HAL_GPIO_WritePin(L_RW_GPIO_Port, L_RW_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DATA_DIR_GPIO_Port, DATA_DIR_Pin, GPIO_PIN_RESET);
}

void wait_sin(){

	uint16_t t = 1;

	while(t == 1){
		t = HAL_GPIO_ReadPin(D0_GPIO_Port,  L_SIN_IN_Pin);
	}
}

