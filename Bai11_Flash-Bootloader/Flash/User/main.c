#include "stm32f10x.h"                  // Device header
#include "stm32f10x_flash.h"            // Device:StdPeriph Drivers:Flash

void Flash_Erase1Page(uint32_t pageAddress){
	FLASH_Unlock();
	FLASH->CR |= FLASH_CR_PER;
	FLASH->AR = pageAddress;
	FLASH->CR |= FLASH_CR_STRT;
	while((FLASH->SR & FLASH_SR_BSY) == SET);
	FLASH->CR &= ~FLASH_CR_PER;
	FLASH_Lock();
}

void writeFlash(uint32_t *address, uint16_t data){
	FLASH_Unlock();
	FLASH->CR |= FLASH_CR_PG;
	*(address) = data;
	while((FLASH->SR & FLASH_SR_BSY) == SET);
	FLASH->CR &= ~FLASH_CR_PG;
	FLASH_Lock();
}

int main(){
	Flash_Erase1Page(0x08008810);
	writeFlash((uint32_t *)0x08008810, 0xDA26);
}
