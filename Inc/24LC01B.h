/*
 * 24LC01B.h
 *
 *  Created on: Dec 10, 2022
 *      Author: Sudow
 */

#ifndef INC_24LC01B_H_
#define INC_24LC01B_H_

// Address of EEPROM device
uint8_t address_24lc01b = 0x50;

void i2c_init();

void i2c_master_transmit(uint8_t slave_address, uint8_t *buffer, uint16_t size);

void master_transmit_24lc01b(uint8_t *buffer, uint16_t size);

void debug_diode_init();

void debug_diode_on();

#endif /* INC_24LC01B_H_ */
