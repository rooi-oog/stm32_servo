#include <stdio.h>
#include <errno.h>
#include <libopencm3/stm32/usart.h>
#include "config.h"
#include "uart.h"

static char rx_buf [UART_RINGBUFFER_SIZE_RX];
static volatile unsigned int rx_produce;
static volatile unsigned int rx_consume;

static char tx_buf [UART_RINGBUFFER_SIZE_TX];
static unsigned int tx_produce;
static unsigned int tx_consume;
static volatile int tx_cts;

void usart2_isr (void)
{
	if (USART_SR (USART2) & USART_SR_TC) {                                     // Check interrupt source
		USART_SR (USART2) &= ~USART_SR_TC;                                     // Clear pending bit
		if (tx_produce != tx_consume) {
			USART_DR (USART2) = (uint8_t) tx_buf [tx_consume];                 // Send byte through UART
			tx_consume = (tx_consume + 1) & UART_RINGBUFFER_MASK_TX;
		} else
			tx_cts = 1;
	}
	
	if (USART_SR (USART2) & USART_SR_RXNE) {
		USART_SR (USART2) &= ~USART_SR_RXNE;
		rx_buf[rx_produce] = (uint8_t) USART_DR (USART2) & 0xff;               // Receive byte from UART
		rx_produce = (rx_produce + 1) & UART_RINGBUFFER_MASK_RX;
	}
}

/* Do not use in interrupt handlers! */
char uart_read (void)
{
	char c;

	while (rx_consume == rx_produce);
	c = rx_buf [rx_consume];
	rx_consume = (rx_consume + 1) & UART_RINGBUFFER_MASK_RX;
	return c;
}

int uart_read_nonblock (void)
{
	return (rx_consume != rx_produce);
}

void uart_write (char c)
{
	if (tx_cts) {
		tx_cts = 0;
		USART_DR (USART2) = (uint8_t) c;
	}
	else {		
		tx_buf [tx_produce] = c;
		tx_produce = (tx_produce + 1) & UART_RINGBUFFER_MASK_TX;
	}
}

void uart_print (char *msg)
{
	while (*msg != 0) uart_write (*msg++);
}

#ifdef UART_COMM
int _write(int file, char *ptr, int len)
{
	if (file == 1) {
		uart_print (ptr);
		return len;
	}

	errno = EIO;
	return -1;
}

int _read (int file, char *ptr, int len)
{
	int l = 0;	
	
	if (file == 0) {
		if (!uart_read_nonblock ())
			return 0;
			
		while (l < len) {
			ptr [l] = uart_read ();
			if (ptr [l++] == '\n')
				break;		
		}
		
		return l;
	}
	
	errno = EIO;
	return -1;	
}
#endif
