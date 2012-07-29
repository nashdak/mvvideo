
#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "ring_buffer.h"

#include "cmd.h"
#include "uart.h"

#define BAUD 9600
#include <util/setbaud.h>

#define LC_INCLUDE "lc-addrlabels.h"
#include "pt.h"


stat_t stat;

int uart_putchar(char c)
{
	loop_until_bit_is_set(UCSR0A, UDRE0);
	UDR0 = c;
	return 0;
}

static struct ring_buffer tx_ring_buffer;
static unsigned char tx_buffer[255];

unsigned char rx_buffer[255];
unsigned char rx_buffer_size = 0;

int uart_tx_str(char *s)
{
    while (*s)
    {
    	ring_buffer_put(&tx_ring_buffer, *s);
        s++;
    }
	ring_buffer_put(&tx_ring_buffer, 0);
    return 0;
}

int uart_tx_buffer(unsigned char *s, int len)
{
	while (len--)
	{
    	ring_buffer_put(&tx_ring_buffer, *s);
		s++;
	}
	return 0;
}

int uart_tx_putbuf(unsigned char *s, int len)
{
	while (len--)
	{
		uart_putchar(*s);
		s++;
	}
	return 0;
}

ISR(USART_RX_vect)
{
	unsigned char b = UDR0;
	rx_buffer[rx_buffer_size] = b;
	rx_buffer_size++;
	stat.uart_rx_irq++;
}

static inline void uart_init(void)
{
	UBRR0H = UBRRH_VALUE;
	UBRR0L = UBRRL_VALUE;
#if USE_2X
	UCSR0A |= (1 << U2X0);
#else
	UCSR0A &= ~(1 << U2X0);
#endif

	// enable receiver & transmitter, set IRG on Rx complete
	UCSR0B |= (1 << RXCIE0) | (1 << RXEN0) | (1 << TXEN0);
	loop_until_bit_is_set(UCSR0A, UDRE0);

	// 8 buts, 1 stop bit, no parity
	UCSR0C |= (1 << UCSZ01) | (1 << UCSZ00);


}

static inline void board_init(void)
{
	wdt_reset();
	wdt_disable();

	// Sleep mode - idle
	MCUCR &= ~ ((1 << SM0) | (1 << SM1) | (1 << SM2));
}

static int pt_uart_rx(struct pt *pt)
{
	PT_BEGIN(pt);

	while (1)
	{
		PT_YIELD_UNTIL(pt, rx_buffer_size != 0);
		process_command();
	}

	PT_END(pt);
}

static int pt_uart_tx(struct pt *pt)
{
	unsigned char c;
	PT_BEGIN(pt);

	while (1)
	{
		PT_YIELD_UNTIL(pt, !ring_buffer_is_empty(&tx_ring_buffer));
		c = ring_buffer_get(&tx_ring_buffer);
		uart_putchar(c);
	}

	PT_END(pt);
}

static struct pt pt_uart_rx_state;
static struct pt pt_uart_tx_state;

/**
 * Implements main loop
 */
int main(void)
{
	cli();
	board_init();
	uart_init();

	tx_ring_buffer = ring_buffer_init(tx_buffer, sizeof(tx_buffer));
	PT_INIT(&pt_uart_rx_state);
	PT_INIT(&pt_uart_tx_state);
	sei();

	while (1)
	{
		pt_uart_tx(&pt_uart_tx_state);
		pt_uart_rx(&pt_uart_rx_state);
		//uart_putchar(0x13);
	}

	cli();
	// this quits the simulator, since interrupts are off
	// this is a "feature" that allows running tests cases and exit
	sleep_cpu();

	return 0;
}
