/*
 * eeprom.c
 *
 *  Created on: 11 ���. 2019 �.
 *      Author: User
 */

#include "eeprom.h"
#include "flash_interface.h"

// ������/������ ������������ ���������� ���������

#define EEPROM2_START_ADDR   ((uint32_t)0x0807f000)
#define EEPROM2_PAGE_SIZE	2048

static const uint64_t ee_key = 0xBAAAAAAAAAAAAAAA;

static uint32_t PAGEError = 0;

static FLASH_EraseInitTypeDef EraseInitStruct;

// ����� ������ ���������� ������� ����������� �������� � ��������
static uint8_t find_last_written_var_addr(uint16_t* var_addr) {
	uint64_t word1, word2;
	uint8_t res = 0;
	*var_addr=0;
	uint16_t addr=16;

	while(addr+16<=EEPROM2_PAGE_SIZE) {
		word1 = *(__IO uint64_t *)(EEPROM2_START_ADDR+addr);
		word2 = *(__IO uint64_t *)(EEPROM2_START_ADDR+addr+8);
		if(word1==(~word2)) {
			res=1;
			*var_addr=addr;
			addr+=16;
		}else break;
	}
	return res;
}

// �������� ������������ ��������� ��������
static uint8_t check_eeprom() {
	// � ������ ������ ���� ������� �����
	uint64_t key_word = *(__IO uint64_t *)(EEPROM2_START_ADDR);
	if(key_word!=ee_key) return 0;
	// ������� ����� ���������� ��������
	uint16_t addr=16;
	find_last_written_var_addr(&addr);
	addr+=16;
	// ��������� ������ ������ ���� 0xFF
	uint64_t word1, word2;
	while(addr+16<=EEPROM2_PAGE_SIZE) {
		word1 = *(__IO uint64_t *)(EEPROM2_START_ADDR+addr);
		word2 = *(__IO uint64_t *)(EEPROM2_START_ADDR+addr+8);
		if((word1!=0xFFFFFFFFFFFFFFFF) || (word2!=0xFFFFFFFFFFFFFFFF)) return 0;
		addr+=16;
	}
	return 1;
}

uint8_t  init_eeprom2() {
	if(check_eeprom()==0) {
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = GetBank(EEPROM2_START_ADDR);
		EraseInitStruct.Page        = GetPage(EEPROM2_START_ADDR);
		EraseInitStruct.NbPages     = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)==HAL_OK) {
			// ������ �������� �����
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR, ee_key);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR+8, 0);
			return 1;
		}
		return 0;
	}
	return 1;
}

uint64_t read_var2(uint8_t *success_flag) {
	uint8_t res = 0;
	uint16_t addr=0;
	res=find_last_written_var_addr(&addr);
	*success_flag = res;
	if(res) {
		return *(__IO uint64_t *)(EEPROM2_START_ADDR+addr);
	}
	return 0;
}

void write_var2(uint64_t value) {
	uint16_t addr=0;
	HAL_FLASH_Unlock();
	if(find_last_written_var_addr(&addr)==0) addr=0;
	if(addr >= EEPROM2_PAGE_SIZE-16) { // �������� ���������
		// erase page
		EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
		EraseInitStruct.Banks       = GetBank(EEPROM2_START_ADDR);
		EraseInitStruct.Page        = GetPage(EEPROM2_START_ADDR);
		EraseInitStruct.NbPages     = 1;
		if(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError)==HAL_OK) {
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR, ee_key);
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR+8, 0);
			addr=0;
		}
	}
	addr+=16;
	if(addr<=EEPROM2_PAGE_SIZE-16) {
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR+addr, value);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, EEPROM2_START_ADDR+addr+8, ~value);
	}
}
