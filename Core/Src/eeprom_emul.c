/*
 * eeprom_emul.c
 *
 *  Created on: Apr 7, 2026
 *      Author: Leon Fauth
 */


#include "eeprom_emul.h"
#include "stm32g0xx_hal.h"
#include <stdint.h>

/*
// reads the last value from the Flash page. Assumes uint16_t data.
uint16_t EE_Read(void) {
    uint16_t last_valid_val = 0;
    uint64_t *ptr = (uint64_t*)FLASH_STORAGE_ADDR;

    // Scan the 2KB page (256 slots of 8-bytes each)
    for (int i = 0; i < (PAGE_SIZE / DOUBLE_WORD); i++) {
        if (ptr[i] == FLASH_EMPTY_VALUE) {
            break; // Reached the end of written data
        }
        last_valid_val = (uint16_t)ptr[i];
    }
    return last_valid_val;
}

void EE_Write(uint16_t data) {
    uint16_t target_addr = 0;
    uint64_t *ptr = (uint64_t*)FLASH_STORAGE_ADDR;

    // 1. Find the first empty slot
    int slot_index = 0;
    for (slot_index = 0; slot_index < (PAGE_SIZE / DOUBLE_WORD); slot_index++) {
        if (ptr[slot_index] == FLASH_EMPTY_VALUE) {
            target_addr = FLASH_STORAGE_ADDR + (slot_index * 8);
            break;
        }
    }

    HAL_FLASH_Unlock();

    // 2. If page is full, erase it and start over at index 0
    if (target_addr == 0) {
        FLASH_EraseInitTypeDef eraseInit;
        uint32_t pageError;

        eraseInit.TypeErase = FLASH_TYPEERASE_PAGES;
        eraseInit.Banks     = FLASH_BANK_1;
        eraseInit.Page      = FLASH_PAGE_NUMBER;
        eraseInit.NbPages   = 1;

        if (HAL_FLASHEx_Erase(&eraseInit, &pageError) != HAL_OK) {
            HAL_FLASH_Lock();
            return; // Erase failed
        }
        target_addr = FLASH_STORAGE_ADDR;
    }

    // 3. Program the data (STM32G MUST use DoubleWord)
    // We cast to uint64_t to satisfy the 64-bit requirement
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, target_addr, (uint64_t)data);

    HAL_FLASH_Lock();
}

*/

uint16_t EEPROM_Read(void)
{
    uint32_t addr = EEPROM_PAGE_ADDR;
    uint16_t last = 0xFFFF;

    while (addr < EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        uint64_t data = *(uint64_t*)addr;

        if (data == 0xFFFFFFFFFFFFFFFFULL)
            break;

        last = (uint16_t)(data & 0xFFFF);
        addr += 8;
    }

    return last;
}

void EEPROM_Write(uint16_t value)
{
    HAL_FLASH_Unlock();

    uint32_t addr = EEPROM_PAGE_ADDR;

    // Find first empty slot
    while (addr < EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        if (*(uint64_t*)addr == 0xFFFFFFFFFFFFFFFFULL)
            break;

        addr += 8;
    }

    // If full → erase page
    if (addr >= EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t error;

        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Page = (EEPROM_PAGE_ADDR - FLASH_BASE) / PAGE_SIZE;
        erase.NbPages = 1;

        HAL_FLASHEx_Erase(&erase, &error);

        addr = EEPROM_PAGE_ADDR;
    }

    uint64_t data = (uint64_t)value;  // only lower 16 bits used

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data);

    HAL_FLASH_Lock();
}

