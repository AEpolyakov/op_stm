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
void write_word(uint16_t data, uint16_t address);
uint16_t read_data();
uint16_t read_word(uint16_t address);


void prepare_tx_buffer();
void parse_rx_buffer();

void uz_oo_handle();
void reset_uz_counter();

#endif /* INC_OU_H_ */
