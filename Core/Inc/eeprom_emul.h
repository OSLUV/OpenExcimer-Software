/*
 * eeprom_emul.h
 *
 *  Created on: Apr 7, 2026
 *      Author: Leon Fauth
 */

#ifndef INC_EEPROM_EMUL_H_
#define INC_EEPROM_EMUL_H_

#include "stm32g0xx_hal.h"
#include <stdint.h>

/*
#define FLASH_BASE_ADDR   0x08000000
#define FLASH_STORAGE_SIZE	(128 * 1024)   // 128 kb flash
#define PAGE_SIZE         2048
#define DOUBLE_WORD 8 // 8 bytes / 64 bit
#define FLASH_STORAGE_ADDR 0x0801F800
#define FLASH_PAGE_NUMBER     63          // Page index
#define FLASH_EMPTY_VALUE     0xFFFFFFFFFFFFFFFFULL
*/

#define EEPROM_PAGE_ADDR  0x0800F800  // last page (example!)
#define PAGE_SIZE         2048

uint16_t EE_Read(void);
void EE_Write(uint16_t data);

uint16_t EEPROM_Read(void);
void EEPROM_Write(uint16_t value);


#endif /* INC_EEPROM_EMUL_H_ */
