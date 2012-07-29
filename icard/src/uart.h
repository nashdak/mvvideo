

#ifndef UART_H_INCLUDED
#define UART_H_INCLUDED


/**
 * Send string to UART
 * This is non-blocking call
 */
unsigned char uart_tx_buffer(unsigned char *s, unsigned char len);

/**
 * Send zero terminated string to UART
 * This is non-blocking call
 */
unsigned char uart_tx_str(char *s);


/**
 * Send a char out
 * This a blocking call
 */
unsigned char uart_putchar(char c);

extern unsigned char rx_buffer[255];
extern unsigned char rx_buffer_size;

unsigned char uart_tx_putbuf(unsigned char *s, unsigned char len);

#endif

