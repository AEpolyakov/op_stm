/*
 * ou.h
 *
 *  Created on: Jan 8, 2025
 *      Author: Alexey
 */

#ifndef INC_OU_H_
#define INC_OU_H_

extern uint8_t no_sin_error;
extern uint8_t zero_sin_error;

void ou_exchange();
void make_ti();
void write_word(uint16_t data, uint16_t address);
uint16_t read_word(uint16_t address);
void write_address(uint16_t address);
void write_data(uint16_t data);
uint16_t read_data();
uint16_t gpio_data_read();

void buffer_direction_out();
void buffer_direction_in();
void write_in();
void write_out();

void prepare_tx_buffer();

void uz_oo_handle();
void reset_uz_counter();

#endif /* INC_OU_H_ */
