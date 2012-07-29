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
	int process_command;
	int process_command_fs_1;
	int process_command_fs_2;
	int process_command_fs_3;
	int process_command_bad_id;
	int process_command_bad_size;
	int process_command_bad_cs;
	int process_command_handler;
	int process_command_handler_done;
} stat_t;

extern stat_t stat;

#endif
