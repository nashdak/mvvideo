#include <string.h>

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "uart.h"
#include "cmd.h"


#define PRINT_TRACE 0

system_status_t system_status;

static const unsigned char VERSION[] = {0, 0, 1};

#define PAYLOAD_OFFSET 4
#define PAYLOAD_SIZE_OFFSET 3
#define COMMAND_ID_OFFSET 2

#define READ_ACCESS_8 0
#define WRITE_ACCESS_8 1
#define READ_ACCESS_16 2
#define WRITE_ACCESS_16 3

#define IS_READ_ACCESS(type) (((type) & 0x01) == 0)
#define IS_ACCESS_8(type) (((type) & 0x02) == 0)

typedef struct
{
	unsigned char size;
	unsigned char (*handler)(unsigned char size);
	int calls;
} cmd_t;

static unsigned char calculate_checksum(unsigned char size)
{
	unsigned char sum = 0;
	unsigned char i;
	unsigned char *s = rx_buffer;

	size = size - 1;
	for (i = 0;i < size;i++,s++)
		sum += *s;
	sum = ~sum + 1;

	return sum;
}

static unsigned char checksum(unsigned char size)
{
	unsigned char sum;
	unsigned char cs;

	sum = calculate_checksum(size);
	cs = rx_buffer[size - 1];

	return (sum == cs);
}

static unsigned char send_command(unsigned char size)
{
	unsigned char cs;

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

static unsigned char cmd_error(unsigned char size)
{
	return -1;
}

static unsigned char cmd_ping(unsigned char size)
{
	send_command(size);

	return 0;
}

static unsigned char cmd_get_firmware_version(unsigned char size)
{
	memcpy(&rx_buffer[PAYLOAD_OFFSET], VERSION, sizeof(VERSION));
	size = size + sizeof(VERSION);
	send_command(size);

	return 0;
}

static unsigned char cmd_get_statistics(unsigned char size)
{
	unsigned char stat_size = sizeof(stat);

	memcpy(&rx_buffer[PAYLOAD_OFFSET], &stat, stat_size);
	size = size + stat_size;
	send_command(size);

	return 0;
}

/**
 * Command always contains 5 bytes in the payload
 * Response contains 5 bytes
 * 1 byte access type: 0-read8,1-write8,2-read16,3-write16
 * 2 bytes of address
 * 2 bytes of value
 */
static unsigned char cmd_access_memory(unsigned char size)
{
	unsigned char *cmd = &rx_buffer[PAYLOAD_OFFSET];
	char type = cmd[0];
	unsigned int *address = (unsigned int*)&cmd[1];
	unsigned int value;

	switch (type)
	{
	case READ_ACCESS_8:
		stat.process_command_am_read++;
		value = *(unsigned char*)address;
		*(unsigned int*)&cmd[3] = (unsigned int)(value & 0xFF);
		break;
	case READ_ACCESS_16:
		stat.process_command_am_read++;
		value = *(unsigned int*)address;
		*(unsigned int*)&cmd[3] = (unsigned int)value;
		break;
	case WRITE_ACCESS_8:
		stat.process_command_am_write++;
		value = *(unsigned int*)&cmd[3];
		*(unsigned char*)address = (unsigned char)(value & 0xFF);
		break;
	case WRITE_ACCESS_16:
		stat.process_command_am_write++;
		value = *(unsigned int*)&cmd[3];
		*(unsigned int*)address = (unsigned int)value;
		break;
	}

	send_command(size);

	return 0;
}

static unsigned char cmd_get_uptime(unsigned char size)
{
	unsigned int uptime;

	uptime = SYS_TIMER_COUNTER;
	*(unsigned int *)(&rx_buffer[PAYLOAD_OFFSET]) = uptime;

	// uptime is 16 bits of seconds
	size = size + 2;
	send_command(size);

	return 0;
}

static unsigned char cmd_software_reset(unsigned char size)
{
	size = size + sizeof(VERSION);
	send_command(size);

	// Do not reset WD in the main loop
	watchdog_enabled = 0;
	// Enable the WD, let some time to send the bytes
	// Possible values for timeout WDTO_15MS, WDTO_30MS,WDTO_60MS,
	// WDTO_120MS,WDTO_250MS,WDTO_500MS,WDTO_1S,WDTO_2S
	wdt_enable(WDTO_30MS);
	wdt_reset();

	return 0;
}

static unsigned char cmd_get_status(unsigned char size)
{
	unsigned char system_status_size = sizeof(system_status);

	memcpy(&rx_buffer[PAYLOAD_OFFSET], &system_status, system_status_size);
	size = size + system_status_size;
	send_command(size);

	return 0;
}

static const cmd_t commands[] = {
		{ 0, 		cmd_error									}, // 0x00 - Reserved
		{ 0, 		cmd_error									}, // 0x01 - Set debug output state
		{ 0, 		cmd_error									}, // 0x02 - Set serial link rate
		{ 5, 		cmd_ping									}, // 0x03 - Ping
		{ 5, 		cmd_get_status								}, // 0x04 - Get status
		{ 0, 		cmd_error									}, // 0x05 - Set fan
		{ 0, 		cmd_error									}, // 0x06 - 48V set
		{ 0, 		cmd_error									}, // 0x07 - GPIO set
		{ 0, 		cmd_error									}, // 0x08 - GPIO read
		{ 0, 		cmd_error									}, // 0x09 - SPI write
		{ 0, 		cmd_error									}, // 0x0A - SPI read
		{ 0, 		cmd_error									}, // 0x0B - MDIO read/write
		{ 0, 		cmd_error									}, // 0x0C - Reserved
		{ 0, 		cmd_error									}, // 0x0D - IIC read/write
		{ 0, 		cmd_error									}, // 0x0E - Reserved
		{ 0, 		cmd_error									}, // 0x0F - SFP read
		{ 0, 		cmd_error									}, // 0x10 - Set configuration parameters
		{ 0, 		cmd_error									}, // 0x11 - Write configuration parameters to EEPROM
		{ 0, 		cmd_error									}, // 0x12 - Get configuration parameters
		{ 5, 		cmd_get_firmware_version					}, // 0x13 - Get firmware version
		{ 5, 		cmd_get_statistics                          }, // 0x14 - Get statistics
		{ 9, 		cmd_access_memory                           }, // 0x15 - Access memory
		{ 0, 		cmd_error									}, // 0x16 - Invalidate configuration in the EEPOM
		{ 5, 		cmd_get_uptime                              }, // 0x17 - Get uptime
		{ 5, 		cmd_software_reset                          }, // 0x18 - Software reset
		{ 0, 		cmd_error									}, // 0x19 - Debug message
		{ 0, 		cmd_error									}  // 0x1A - GPIO setup
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

static unsigned char shift_rx_buffer(void)
{
	cli();
	rx_buffer_size--;
	memcpy(&rx_buffer[0], &rx_buffer[1], rx_buffer_size);
	sei();

	return 0;
}

static unsigned char find_synchronization(void)
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


unsigned char process_command()
{
	unsigned char ret = 0;
	unsigned char res;
	const cmd_t *cmd;
	unsigned char expected_size, command_size;
	unsigned char command_id;

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
