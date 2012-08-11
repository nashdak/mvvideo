
#include <string.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/sfr_defs.h>


#include "i2c.h"

#define I2C_DDR DDRD
#define I2C_PIN PIND
#define I2C_PORT PORTD

#define I2C_CLK 0
#define I2C_DAT 1

#define I2C_DATA_HI() I2C_DDR &= ~(1 << I2C_DAT);
#define I2C_DATA_LO() I2C_DDR |= (1 << I2C_DAT);

#define I2C_CLOCK_HI() I2C_DDR &= ~(1 << I2C_CLK);
#define I2C_CLOCK_LO() I2C_DDR |= (1 << I2C_CLK);

void i2c_sw_writebit(unsigned char c)
{
	if (c > 0)
	{
		I2C_DATA_HI();
	}
	else
	{
		I2C_DATA_LO();
	}

	I2C_CLOCK_HI();
	_delay_ms(1);

	I2C_CLOCK_LO();
	_delay_ms(1);

	if (c > 0)
	{
		I2C_DATA_LO();
	}

	_delay_ms(1);
}

unsigned char i2c_sw_readbit(void)
{
	I2C_DATA_HI();

	I2C_CLOCK_HI();
	_delay_ms(1);

	unsigned char c = I2C_PIN;

	I2C_CLOCK_LO();
	_delay_ms(1);

	return (c >> I2C_DAT) & 1;
}

void i2c_sw_init(void)
{
	I2C_PORT &= ~((1 << I2C_DAT) | (1 << I2C_CLK));

	I2C_CLOCK_HI();
	I2C_DATA_HI();

	_delay_ms(1);
}

void i2c_sw_start(void)
{
	// set both to high at the same time
	I2C_DDR &= ~((1 << I2C_DAT) | (1 << I2C_CLK));
	_delay_ms(1);

	I2C_DATA_LO();
	_delay_ms(1);

	I2C_CLOCK_LO();
	_delay_ms(1);
}

void i2c_sw_stop(void)
{
	I2C_CLOCK_HI();
	_delay_ms(1);

	I2C_DATA_HI();
	_delay_ms(1);
}

unsigned char i2c_sw_write(unsigned char c)
{
	for (char i=0;i<8;i++)
	{
		i2c_sw_writebit(c & 128);

		c<<=1;
	}

	return i2c_sw_readbit();
}


unsigned char i2c_sw_read(unsigned char ack)
{
	unsigned char res = 0;

	for (char i=0;i<8;i++)
	{
		res <<= 1;
		res |= i2c_sw_readbit();
	}

	if (ack > 0)
	{
		i2c_sw_writebit(0);
	}
	else
	{
		i2c_sw_writebit(1);
	}

	_delay_ms(1);

	return res;
}

#define TW_START					0x08
#define TW_REP_START				0x10
// Master Transmitter
#define TW_MT_SLA_ACK				0x18
#define TW_MT_SLA_NACK				0x20
#define TW_MT_DATA_ACK				0x28
#define TW_MT_DATA_NACK				0x30
#define TW_MT_ARB_LOST				0x38
// Master Receiver
#define TW_MR_ARB_LOST				0x38
#define TW_MR_SLA_ACK				0x40
#define TW_MR_SLA_NACK				0x48
#define TW_MR_DATA_ACK				0x50
#define TW_MR_DATA_NACK				0x58
#define TW_NO_INFO					0xF8
#define TW_BUS_ERROR				0x00

// defines and constants
#define TWCR_CMD_MASK		0x0F
#define TWSR_STATUS_MASK	0xF8

// return values
#define I2C_OK				0x00
#define I2C_ERROR_NODEV		0x01

#define sbi(port, bit) (port) |= (1 << (bit))
#define cbi(port, bit) (port) &= ~(1 << (bit))
#define inb(port) (port)
#define outb(port, val) (port) = (val)

void i2c_hw_setbitrate(unsigned short bitrate_khz);

void i2c_hw_setbitrate(unsigned short bitrate_khz)
{
	unsigned char bitrate_div;
    // set i2c bitrate
    // SCL freq = F_CPU/(16+2*TWBR))
    #ifdef TWPS0
            // for processors with additional bitrate division (mega128)
            // SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
            // set TWPS to zero
            cbi(TWSR, TWPS0);
            cbi(TWSR, TWPS1);
    #endif
    // calculate bitrate division
    bitrate_div = ((F_CPU/1000l)/bitrate_khz);
    if(bitrate_div >= 16)
            bitrate_div = (bitrate_div-16)/2;
    outb(TWBR, bitrate_div);
}

void i2c_hw_init(void)
{
	// set pull-up resistors on I2C bus pins
    sbi(PORTD, 0);  // i2c SCL on ATmega128,64
    sbi(PORTD, 1);  // i2c SDA on ATmega128,64

    // set i2c bit rate to 100KHz
    i2c_hw_setbitrate(100);

    // enable TWI (two-wire interface)
    sbi(TWCR, TWEN);

    // enable TWI interrupt and slave address ACK
    sbi(TWCR, TWIE);
    sbi(TWCR, TWEA);

    //outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT)|_BV(TWEA));
    // enable interrupts
    sei();
}

inline void i2c_hw_waitforcomplete(void);
inline void i2c_hw_waitforcomplete(void)
{
	// wait for i2c interface to complete operation
	while( !(inb(TWCR) & _BV(TWINT)) );
}

void i2c_hw_start(void)
{
	outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT)|_BV(TWSTA));
}

void i2c_hw_stop(void)
{
        // transmit stop condition
        // leave with TWEA on for slave receiving
        outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT)|_BV(TWEA)|_BV(TWSTO));
}

unsigned char i2c_hw_write(unsigned char data)
{
	unsigned char ack;
	// save data to the TWDR
    outb(TWDR, data);
    // begin send
    outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT));

    i2c_hw_waitforcomplete();

	ack = (inb(TWSR) == TW_MT_SLA_ACK);
	return ack;
}


unsigned char i2c_hw_read(unsigned char ack)
{
	unsigned char b;

	if (ack)
	{
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT)|_BV(TWEA));
	}
	else
	{
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|_BV(TWINT));
	}
    i2c_hw_waitforcomplete();

    // retrieve received data byte from i2c TWDR
    b = inb(TWDR);

    return b;
}




