
#ifndef I2C_H
#define I2C_H

extern void i2c_writebit(unsigned char c);
extern unsigned char i2c_readbit(void);
extern void i2c_init(void);
extern void i2c_start(void);
extern void i2c_stop(void);
extern unsigned char i2c_write(unsigned char c);
extern unsigned char i2c_read(unsigned char ack);

#endif // I2C_H
