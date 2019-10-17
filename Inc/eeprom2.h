/*
 * eeprom.h
 *
 *  Created on: 11 окт. 2019 г.
 *      Author: User
 */

#ifndef EEPROM2_H_
#define EEPROM2_H_

#include <stdio.h>
#include "main.h"

uint8_t  init_eeprom2();
uint64_t read_var2();
void write_var2(uint64_t value);

#endif /* EEPROM2_H_ */
