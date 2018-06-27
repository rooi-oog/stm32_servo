#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/exti.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>

#include "stm32_usb/usb.h"
#include "config.h"

void clock_setup (void)
{
#if (HSE_VALUE == 8000000)
	rcc_clock_setup_in_hse_8mhz_out_72mhz ();
#else
	rcc_clock_setup_in_hsi_out_64mhz ();
#endif
	rcc_periph_clock_enable (RCC_GPIOC);
	rcc_periph_clock_enable (RCC_GPIOA);
	rcc_periph_clock_enable (RCC_GPIOB);
	rcc_periph_clock_enable (RCC_AFIO);
}

void usb_setup (void)
{
	rcc_periph_clock_enable (RCC_OTGFS);
	usb_cdcacm_init ();
}

void uart_setup ()
{	
	rcc_periph_clock_enable (RCC_USART2);
	
	nvic_enable_irq (NVIC_USART2_IRQ);						// USART2 interrupt enable	
	
	/* Setup UART parameters. */
	usart_set_baudrate (USART2, 115200);
	usart_set_databits (USART2, 8);
	usart_set_stopbits (USART2, USART_STOPBITS_1);
	usart_set_parity (USART2, USART_PARITY_NONE);
	usart_set_flow_control (USART2, USART_FLOWCONTROL_NONE);
	usart_set_mode (USART2, USART_MODE_TX_RX);

	/* Enable USART2 Receive interrupt. */
	USART_CR1(USART2) |= USART_CR1_RXNEIE | USART_CR1_TCIE;

	/* Finally enable the USART. */
	usart_enable (USART2);
}

void gpio_setup (void)
{
#ifdef UART_COMM
	/* USART2 pins */
	gpio_set_mode (GPIOA, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO2);
	gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO3);	
#endif

	/* Velocity sensor input */
	gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0 | GPIO1);
		
	/* Encoder input */
	gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO6 | GPIO7);
		
	/* PWM output */
	gpio_set_mode (GPIOB, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, 
			GPIO6 | GPIO7 | GPIO8 | GPIO9);

#ifdef STEP_DIR /* Controlling DC motor position by step/dir signal */
	gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, 
			STEP_0_PIN | DIR_0_PIN | STEP_1_PIN | DIR_1_PIN);
	/* 
	 * Unless I'll realize how to occupy last remaining timer
	 * for step/dir counting purpose I have to use external interrupt 
	 */	
	exti_enable_request (STEP_0_PIN | STEP_1_PIN);
    exti_set_trigger (STEP_0_PIN | STEP_1_PIN, EXTI_TRIGGER_RISING);
    exti_select_source (STEP_0_PIN | STEP_1_PIN, GPIOB);
	nvic_enable_irq (NVIC_EXTI15_10_IRQ);
		
#else /* Speed adjusting buttons */	
	gpio_set_mode (GPIOA, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, ALL_SPEED_STOP);
	gpio_set_mode (GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_PULL_UPDOWN, 
			MAIN_SPEED_UP | MAIN_SPEED_DOWN | SCROLL_SPEED_UP | SCROLL_SPEED_DOWN);	
			
	/* Pulling up the above pins */
	gpio_set (GPIOA, ALL_SPEED_STOP);
	gpio_set (GPIOB, MAIN_SPEED_UP | MAIN_SPEED_DOWN | SCROLL_SPEED_UP | SCROLL_SPEED_DOWN);	
#endif	

	/* Single LED onboard :( */
	gpio_set_mode (GPIOC, GPIO_MODE_OUTPUT_2_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO13);
	
	/* LED off */
	gpio_set (GPIOC, GPIO13);
}

void encoders_setup ()
{
	rcc_periph_clock_enable (RCC_TIM2);
	rcc_periph_clock_enable (RCC_TIM3);
	
	/* Counting on both edges of both channels */
	TIM2_SMCR	|= TIM_SMCR_SMS_EM3;
	TIM2_CCMR1	|= TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_CC2S_IN_TI2;
	TIM2_ARR	 = 0xffff;
	TIM2_CR1	|= TIM_CR1_CEN;
	
	TIM3_SMCR	|= TIM_SMCR_SMS_EM3;
	TIM3_CCMR1	|= TIM_CCMR1_CC1S_IN_TI1 | TIM_CCMR1_CC2S_IN_TI2;
	TIM3_ARR	 = 0xffff;
	TIM3_CR1	|= TIM_CR1_CEN;	
}
	
void pwm_setup () 
{
	rcc_periph_clock_enable (RCC_TIM4);
	
/* ----------------------------------------------------------------------------------------------------- */
/* -- TIM4 - configuration (PWM) ----------------------------------------------------------------------- */
/* ----------------------------------------------------------------------------------------------------- */

	TIM4_CCMR1	|= (TIM_CCMR1_OC1PE  | TIM_CCMR1_OC2PE  |		// Preload enable for both CH1 and CH2
					TIM_CCMR1_OC1M_PWM1 |						// PWM mode 1 - CH1
	                TIM_CCMR1_OC2M_PWM1 );						// PWM mode 1 - CH2
	TIM4_CCMR2	|= (TIM_CCMR2_OC3PE  | TIM_CCMR2_OC4PE  |		// Preload enable for both CH1 and CH2
					TIM_CCMR2_OC3M_PWM1 |						// PWM mode 1 - CH1
	               	TIM_CCMR2_OC4M_PWM1 );						// PWM mode 1 - CH2
	TIM4_CCER	|= (TIM_CCER_CC1E | TIM_CCER_CC2E |
	                TIM_CCER_CC3E | TIM_CCER_CC4E );			// TIM4 CH1-4 output enable

	TIM4_ARR 	 = PWM_MAX_VAL;
	TIM4_PSC 	 = (TIM4_CLK / (PWM_FREQ * PWM_MAX_VAL)) * 2;	// Set PWM frequency

	TIM4_CR1	|= TIM_CR1_CEN | TIM_CR1_ARPE;	
}

void systick_setup (void)
{
	/* 72MHz / 8 => 9000000 counts per second */
	systick_set_clocksource (STK_CSR_CLKSOURCE_AHB_DIV8);

	/* 1000 overflows per second - every 1ms one interrupt */
	/* SysTick interrupt every N clock pulses: set reload to N-1 */
	systick_set_reload (rcc_ahb_frequency / 8 / 1000 - 1);

	systick_interrupt_enable ();

	/* Start counting. */
	systick_counter_enable ();
	nvic_enable_irq (NVIC_SYSTICK_IRQ);
}
