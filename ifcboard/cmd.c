#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>

#include "uart.h"
#include "cmd.h"


#define PRINT_TRACE 0


static const unsigned char VERSION[] = {0, 0, 1};

#define PAYLOAD_OFFSET 4
#define PAYLOAD_SIZE_OFFSET 3
#define COMMAND_ID_OFFSET 2


typedef struct
{
	unsigned char size;
	int (*handler)(int size);
	int calls;
} cmd_t;

static int calculate_checksum(int size)
{
	unsigned char sum = 0;
	int i;
	unsigned char *s = rx_buffer;

	size = size - 1;
	for (i = 0;i < size;i++,s++)
		sum += *s;
	sum = ~sum + 1;

	return sum;
}

static int checksum(int size)
{
	int sum;
	unsigned char cs;

	sum = calculate_checksum(size);
	cs = rx_buffer[size - 1];

	return (sum == cs);
}

static int send_command(int size)
{
	int cs;

	cs = calculate_checksum(size);
	rx_buffer[size - 1] = cs;
	uart_tx_buffer(rx_buffer, size);

#if (PRINT_TRACE != 0)
	uart_putchar('0');
	uart_tx_putbuf(rx_buffer, size);
	uart_putchar('1');
#endif

	return 0;
}

static int cmd_error(int size)
{
	return -1;
}

static int cmd_ping(int size)
{
	send_command(size);

	return 0;
}

static int cmd_get_firmware_version(int size)
{
	memcpy(&rx_buffer[PAYLOAD_OFFSET], VERSION, sizeof(VERSION));
	size = size + sizeof(VERSION);
	send_command(size);

	return 0;
}

static const cmd_t commands[] = {
		{ 0, 		cmd_error									}, // 0x00
		{ 0, 		cmd_error									}, // 0x01
		{ 0, 		cmd_error									}, // 0x02
		{ 5, 		cmd_ping									}, // 0x03
		{ 0, 		cmd_error									}, // 0x04
		{ 0, 		cmd_error									}, // 0x05
		{ 0, 		cmd_error									}, // 0x06
		{ 0, 		cmd_error									}, // 0x07
		{ 0, 		cmd_error									}, // 0x08
		{ 0, 		cmd_error									}, // 0x09
		{ 0, 		cmd_error									}, // 0x0A
		{ 0, 		cmd_error									}, // 0x0B
		{ 0, 		cmd_error									}, // 0x0C
		{ 0, 		cmd_error									}, // 0x0D
		{ 0, 		cmd_error									}, // 0x0E
		{ 0, 		cmd_error									}, // 0x0F
		{ 0, 		cmd_error									}, // 0x10
		{ 5, 		cmd_get_firmware_version					}, // 0x11
		{ 0, 		cmd_error									}, // 0x12
		{ 0, 		cmd_error									}, // 0x13
		{ 0, 		cmd_error									},
};

#define COMMANDS_TOTAL (sizeof(commands)/sizeof(commands[0]))

/**
 * size of this array should be at least commands
 */
static unsigned char commands_stat[0x20];

/**
 * I am going to reset this variable after I get first command
 */
static char first_command = 1;

static int shift_rx_buffer(void)
{
	cli();
	rx_buffer_size--;
	memcpy(&rx_buffer[0], &rx_buffer[1], rx_buffer_size);
	sei();

	return 0;
}

static int find_synchronization(void)
{
	do
	{
		if ((rx_buffer_size >= 2) && (rx_buffer[0] == 0x7F) && (rx_buffer[1] == 0xEF))
		{
			stat.process_command_fs_1++;
			return 1;
		}
		else if (rx_buffer_size <= 2)
		{
			stat.process_command_fs_2++;
			return 0;
		}

		stat.process_command_fs_3++;
		// shift all bytes left
		shift_rx_buffer();
	}
	while (1);
}


int process_command()
{
	int ret = 0;
	int res;
	const cmd_t *cmd;
	int expected_size, command_size;
	int command_id;

	stat.process_command++;

	while (1)
	{
		// check that the command is well formed
		if (rx_buffer_size < 5) // I did not get all bytes yet
			break;

#if (PRINT_TRACE != 0)
		uart_putchar('2');
		uart_tx_putbuf(rx_buffer, rx_buffer_size);
		uart_putchar('3');
#endif

		res = find_synchronization();
		if (!res)
			break;

		command_id = rx_buffer[COMMAND_ID_OFFSET];
		if (command_id >= COMMANDS_TOTAL)
		{
			stat.process_command_bad_id++;
			shift_rx_buffer();
			continue;
		}

		cmd = &commands[command_id];
		expected_size = cmd->size;
		command_size = rx_buffer[PAYLOAD_SIZE_OFFSET] + PAYLOAD_OFFSET; // payload size + 4 bytes of header
		if (expected_size != command_size)
		{
			stat.process_command_bad_size++;
			shift_rx_buffer();
			continue;
		}

		res = checksum(expected_size);
		if (!res)
		{
			stat.process_command_bad_cs++;
			shift_rx_buffer();
			continue;
		}

		if (!first_command)      // in all responses but very first
		{                        // MSB is set
			command_id |= 0x80;
			rx_buffer[COMMAND_ID_OFFSET] = command_id;
		}

		stat.process_command_handler++;
		commands_stat[command_id]++;
		// This is a legal command
		cmd->handler(expected_size);
		stat.process_command_handler_done++;

		rx_buffer_size = 0;
		first_command = 0;
		ret = 1;

		break;
	}

	return ret;
}
