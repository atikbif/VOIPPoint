/*
 * eeprom.h
 *
 *  Created on: 11 ���. 2019 �.
 *      Author: User
 */

#ifndef EEPROM_H_
#define EEPROM_H_

#include <stdio.h>
#include "main.h"

uint8_t  init_eeprom();
uint64_t read_var();
void write_var(uint64_t value);

#endif /* EEPROM_H_ */
