#include <libopencm3/stm32/flash.h>
#include "eeprom.h"
#include "uart/uart.h"

static void flash_erase (unsigned int);
static void flash_write (unsigned char*, unsigned int, unsigned int);

// MUST be 32 cells as it takes full flash page (1Kb)
__attribute__ ((section(".eeprom"), used)) const arm_pid_gains_t pid_eeprom [32] =
{{160, 98304, 53084, 1835, 222822, 0, 0}, 	// Ordinary DC motor on 3-5A
 { 0, 65535, 40000, 800, 60000, 0, 0 },		// One direction speed control with half-bridge
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, 
 {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}, {0, 0, 0, 0, 0, 0, 0}
};

void flash_load_to_host (uint8_t cell)
{
	printf ("%ld, %ld, %ld, %ld, %ld, %ld\n", 
		pid_eeprom [cell].VelocityLimit, pid_eeprom [cell].Kf, 
		pid_eeprom [cell].Kp, pid_eeprom [cell].Ki, 
		pid_eeprom [cell].Kd, pid_eeprom [cell].Kg);
}

void flash_store_from_host (uint8_t cell, arm_pid_gains_t *pid)
{	
	arm_pid_gains_t epid [CELL_COUNT + 1];
	
	memcpy (epid, pid_eeprom, sizeof (arm_pid_gains_t) * CELL_COUNT);
	memcpy (&epid [cell], pid, sizeof (arm_pid_gains_t));
		
	flash_erase ((unsigned int) &pid_eeprom [0]);
	flash_write ((unsigned char *) &epid[0], (unsigned int) &pid_eeprom [0], sizeof (arm_pid_gains_t) * (CELL_COUNT + 1));
}

static void flash_erase (unsigned int pageAddress) 
{
	while (FLASH_SR & FLASH_SR_BSY);
	if (FLASH_SR & FLASH_SR_EOP) {
		FLASH_SR = FLASH_SR_EOP;
	}

	FLASH_CR |= FLASH_CR_PER;
	FLASH_AR = pageAddress;
	FLASH_CR |= FLASH_CR_STRT;
	while (!(FLASH_SR & FLASH_SR_EOP));
	FLASH_SR = FLASH_SR_EOP;
	FLASH_CR &= ~FLASH_CR_PER;
}

// data - pointer to data
// address - address in flash memory 
// count - write byte count; should be a multiple of 2
static void flash_write (unsigned char* data, unsigned int address, unsigned int count) 
{
	unsigned int i;

	while (FLASH_SR & FLASH_SR_BSY);
	if (FLASH_SR & FLASH_SR_EOP) 
	{
		FLASH_SR = FLASH_SR_EOP;
	}

	FLASH_CR |= FLASH_CR_PG;

	for (i = 0; i < count; i += 2) 
	{
		*(volatile unsigned short*) (address + i) = (((unsigned short) data[i + 1]) << 8) + data[i];
		while (!(FLASH_SR & FLASH_SR_EOP));
		FLASH_SR = FLASH_SR_EOP;
	}

	FLASH_CR &= ~(FLASH_CR_PG);
}
