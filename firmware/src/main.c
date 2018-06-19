#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/timer.h>

#include "eeprom.h"
#include "pid_def.h"
#include "pid/pid.h"

#include "main.h"

/* Feedback memory allocation size */
#define BUF_SIZE	1024	// ~1024ms storing size

/* Array of PID struct */
static arm_pid_t pid [2];

/* Array to strore the actual motor position */
static int32_t feedback_buf [BUF_SIZE];

/* Current feedback array index */
static int16_t feedback_buf_index = BUF_SIZE;

/* Data source to store into the buffer: velocity || position */
static int32_t *feedback_src = NULL;

/* Velocity compare counter */
static uint8_t resampler;

/* Global time variable */
volatile uint32_t time = 0;


void delay_ms (uint32_t delay) {
	register uint32_t time_cur = time;
	while ((time - time_cur) < delay);
}

#ifdef STEP_DIR
void exti15_10_isr(void)
{
	register uint8_t dir;
	
	/* Step 0 */ 
	if (EXTI_PR & STEP_0_PIN)
	{
		/* Clear interrupt pending bit */
		EXTI_PR |= STEP_0_PIN;
		/* Check direction */		
		dir = (GPIOB_IDR & DIR_0_PIN) == 0;
		/* Calc position according to direction */
		pid [0].setpoint += dir - !dir;		
	}

	/* Step 1 */	
	if (EXTI_PR & STEP_1_PIN)
	{
		EXTI_PR |= STEP_0_PIN;
		dir = (GPIOB_IDR & DIR_1_PIN) == 0;
		pid [1].setpoint += dir - !dir;		
	}
}
#endif

void sys_tick_handler (void)
{	
	/* Storing feedback for transmitting to PC */
	if (feedback_buf_index < BUF_SIZE)
		feedback_buf [feedback_buf_index++] = *feedback_src; // Store either velocity or position
	
	/* Calculate PID */
	pid_update (&pid [0], resampler == 0);
/*	pid_update (&pid [1], resampler == 0);*/
	
	/* Velocity sampling signal occurs every nth tick */
	resampler = (resampler + 1) & 15;
	
	time++;
}

void message_handler ()
{
	register uint8_t mode;
	char cmd [64];
	
	if (read (0, cmd, 64) > 0) {		
		switch (cmd [0])
		{		
		/* Load/Store from/to flash memory */
		case 'S':
			if (cmd [1] == 's')
				flash_store_from_host (atoi (&cmd [2]), &pid [0].gains);
			else
				flash_load_to_host (atoi (&cmd [2]));
            break;
			
		/* Return current motor position */
		case 'L':								
			printf ("%ld\n", pid [0].feedback);
			break;
		
		/* Ask if feedback buffer is full */
		case 'F':								
			printf ("%d\n", feedback_buf_index);			
			break;
		
		/* Transmit feedback buffer */
		case 'G':		
			for (uint32_t i = 0; i < BUF_SIZE; i++) {
				printf ("%ld\n", feedback_buf [i]);
				#ifndef USB_COMM
					for (volatile uint32_t j = 0; j < 5000; j++); /* this hack is due to uart doesn't keep up with CPU */
				#endif
			}
			break;
			
		/* Set new servo mode */
		case 'M':
			mode = atoi (&cmd [1]);
			feedback_src = mode == POSITION ? &pid [0].feedback : &pid [0].velocity;
			pid_set_mode (pid, mode);
			break;
		
		/* Set sub-command (PWM, TORQUE, VELOCITY) */
		case 'C':
			pid_set_sub_cmd (pid, atoi (&cmd [1]));
			feedback_buf_index = 0;
			break;
		
		/* Jump to position */
		case 'J':
			feedback_buf_index = 0;				// Start filling feedback buffer
			pid_set_point (&pid [0], atoi (&cmd [1]));
			break;
		
		/* Set PID parameters */
		case 'P':
			pid_set_parameters (&pid [0], atoi (&cmd [2]), cmd [1] - 0x30);
			break;
			
		/* Reset PID */
		case 'R':			
			pid_reset (&pid [0]);			
			break;
		
		/* Check communication integrity :) */		
		case 'c':
			printf ("ACK\n");
			break;
		}		
	}
}

void main (void)
{
	/* Periferies initialization */
	clock_setup ();
	gpio_setup ();
#ifdef USB_COMM	
	usb_setup ();
#else
	uart_setup ();
#endif
	encoders_setup ();
	pwm_setup ();
	
	pid_setup (&pid [0], &pid_eeprom [0], POSITION, PWM_MAX_VAL, (uint32_t *) &PWM1_P, (uint32_t *) &PWM1_N, (uint32_t *) &ENCODER_0);
	pid_setup (&pid [1], &pid_eeprom [0], POSITION, PWM_MAX_VAL, (uint32_t *) &PWM2_P, (uint32_t *) &PWM2_N, (uint32_t *) &ENCODER_1);
	
	/* Start servo cycle */
	systick_setup ();

	/* Unblocking flash memory for write by software */
	FLASH_KEYR = 0x45670123;
	FLASH_KEYR = 0xCDEF89AB;
	
	while (1) {		
		message_handler ();
	}
}
