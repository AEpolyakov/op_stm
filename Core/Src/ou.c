#include "main.h"
#include "ou.h"
#include "stm32f4xx_hal.h"
#include "utils.h"
#include "my_gpio.h"

uint8_t no_sin_error;
uint8_t zero_sin_error;

void ou_exchange() {
	uint8_t address;

	buffer_direction_in();
	for(address = 0; address < USPU_BUTTON_SIZE; address++) {
		uspu_buttons[address] = read_word(address + 0x21);
	}

	for(address = 0; address < ACPS_SIZE; address++) {
		acps[address] = read_word(address + 0x41);
	}

	for(address = 0; address < OP_DATA_SIZE; address++) {
		op_data[address] = read_word(address + 0xc1);
	}

	buffer_direction_out();
	for(address = 0; address < USPU_LED_SIZE; address++) {
		write_word(uspu_leds[address], address + 0x21);
	}

	write_word(op_command, address + 0xc1);

    buffer_direction_in();
	write_address(0);
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


/** Read byte from gpio  */
uint16_t read_data(){
	uint16_t t = 0;
	uint8_t data_readed = 0;
	uint16_t i;

	no_sin_error = 1;
	zero_sin_error = HAL_GPIO_ReadPin(D0_GPIO_Port,  L_SIN_IN_Pin) ? 0 : 1;

	buffer_direction_in();
	write_in();

	for(i=0; i<80; i++) {
		if (data_readed == 0) {
			if (HAL_GPIO_ReadPin(D0_GPIO_Port,  L_SIN_IN_Pin) == 0) {
				no_sin_error = 0;
				data_readed = 1;
				t = gpio_data_read();
			}
		}
	}


	return ~t;
}

void parse_rx_buffer() {
	int i;

    op_command = (rx_buffer[1] << 8 | rx_buffer[0]);
    oo_delay = ((rx_buffer[3] << 8) | rx_buffer[2]);

	for(i=0;i<5;i++) {
		uspu_leds[i] = (rx_buffer[i*2+3] << 8 | rx_buffer[i*2+4]);
	}
}

void prepare_tx_buffer() {
	int i;
	int buffer_index = 0;

//	int t1[USPU_BUTTON_SIZE] =  {0xa001, 0xa002, 0xa003, 0xa004, 0xa005, 0xa006, 0xa007, 0xa008, 0xa009};
//
//	for (i=0; i<USPU_BUTTON_SIZE; i++) {
//		uspu_buttons[i] = t1[i];
//	}
//
//	int t2[ACPS_SIZE] =  {0xb1fe, 0xb1ff, 0xb200, 0xb201, 0xb000, 0xbfff};
//
//	for (i=0; i<ACPS_SIZE; i++) {
//		acps[i] = t2[i];
//	}
//
//	int t3[OP_DATA_SIZE] =  {0xc001, 0xc002, 0xc003, 0xc004, 0xc005, 0xc006, 0xc007, 0xc008, 0xc009, 0xc00a, 0xc00b, 0xc00c, 0xc00d, 0xc00e, 0xc00f, 0xc010, 0xc011, 0xc012};
//
//	for (i=0; i<OP_DATA_SIZE; i++) {
//		op_data[i] = t3[i];
//	}

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

void uz_oo_handle() {

	int period;
	int del = oo_delay + 148;

	period = (op_command & 0x0100) ? 1550 : 2700;

	if (upr_zap_count == 0 || upr_zap_count > 10){
	} else {
	  if (upr_zap_count % 2) {
		  HAL_GPIO_WritePin(UPRZAP_GPIO_Port, UPRZAP_Pin, GPIO_PIN_RESET);
		  delay(US_1);
		  HAL_GPIO_WritePin(UPRZAP_GPIO_Port, UPRZAP_Pin, GPIO_PIN_SET);
		  TIM9->ARR = del;
	  } else {
		  HAL_GPIO_WritePin(OO_GPIO_Port, GPIO_PIN_5, GPIO_PIN_SET);
		  delay(US_1);
		  HAL_GPIO_WritePin(OO_GPIO_Port, GPIO_PIN_5, GPIO_PIN_RESET);
		  TIM9->ARR = period - del;
	  }
	}
	upr_zap_count++;
}

void reset_uz_counter() {
	  TIM9->CNT = 0;
	  TIM9->ARR = 1200;
	  upr_zap_count = 0;
}
