#ifndef CMD_H_INCLUDED
#define CMD_H_INCLUDED


/**
 * Complete FSM which handles incoming commands
 * Returns non-zero (size of the command) if a
 * complete command was handled in this call
 */
unsigned char process_command(void);


typedef struct
{
	int uart_rx_irq;
	int uart_rx_irq_add;
	int uart_rx_of;
	int process_command;
	int process_command_fs_1;
	int process_command_fs_2;
	int process_command_fs_3;
	int process_command_bad_id;
	int process_command_bad_size;
	int process_command_bad_cs;
	int process_command_handler;
	int process_command_handler_done;
	int process_command_am_read;
	int process_command_am_write;
} stat_t;

extern stat_t stat;


typedef struct
{
	char temperature[3];
} system_status_t;

extern system_status_t system_status;
extern char watchdog_enabled;

// Timer1 or Timer3
// This macro requires #include <avr/io.h>
#define SYS_TIMER_COUNTER (TCNT1)
#define SYS_TIMER_PRESCALER (TCCR1B)

#endif
