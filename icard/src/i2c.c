#include "i2c.h"

#include <string.h>
#include <avr/io.h>
#include <util/delay.h>

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
