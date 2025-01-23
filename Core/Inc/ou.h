/*
 * ou.h
 *
 *  Created on: Jan 8, 2025
 *      Author: Alexey
 */

#ifndef INC_OU_H_
#define INC_OU_H_


void ou_exchange();
void write_word(uint16_t data, uint16_t address);
uint16_t read_word(uint16_t address);
void write_address(uint16_t address);
void write_data(uint16_t data);
uint16_t read_data();


void buffer_direction_out();
void buffer_direction_in();
void write_in();
void write_out();
void wait_sin();


#endif /* INC_OU_H_ */
