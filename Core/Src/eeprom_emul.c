/*
 * eeprom_emul.c
 *
 *  Created on: Apr 7, 2026
 *      Author: Leon Fauth
 */


#include "eeprom_emul.h"
#include "stm32g0xx_hal.h"
#include <stdint.h>


// read uint16_t value from flash
uint16_t EEPROM_Read(void)
{
    uint32_t addr = EEPROM_PAGE_ADDR;
    uint16_t last = 0xFFFF;

    while (addr < EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        uint64_t data = *(uint64_t*)addr;

        if (data == 0xFFFFFFFFFFFFFFFFULL)  // if empty, we found the last occupied address
            break;

        last = (uint16_t)(data & 0xFFFF); // only lowest 16bit
        addr += 8;
    }

    return last;
}

// store uint16_t value to flash
void EEPROM_Write(uint16_t value)
{
    HAL_FLASH_Unlock();

    uint32_t addr = EEPROM_PAGE_ADDR;


    while (addr < EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        if (*(uint64_t*)addr == 0xFFFFFFFFFFFFFFFFULL) // if empty, use this address
            break;

        addr += 8;
    }

    // If full, erase page
    if (addr >= EEPROM_PAGE_ADDR + PAGE_SIZE)
    {
        FLASH_EraseInitTypeDef erase = {0};
        uint32_t error;

        erase.TypeErase = FLASH_TYPEERASE_PAGES;
        erase.Page = (EEPROM_PAGE_ADDR - FLASH_BASE) / PAGE_SIZE;
        erase.NbPages = 1;

        HAL_FLASHEx_Erase(&erase, &error);

        addr = EEPROM_PAGE_ADDR; // and reset address to start point
    }

    uint64_t data = (uint64_t)value;  // stm32 uses double word

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, data);

    HAL_FLASH_Lock();
}

