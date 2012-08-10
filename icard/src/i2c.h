
#ifndef I2C_H
#define I2C_H

extern void i2c_sw_init(void);
extern void i2c_hw_init(void);

extern void i2c_sw_writebit(unsigned char c);
extern unsigned char i2c_sw_readbit(void);
extern void i2c_sw_start(void);
extern void i2c_sw_stop(void);
extern unsigned char i2c_sw_write(unsigned char c);
extern unsigned char i2c_sw_read(unsigned char ack);

extern void i2c_hw_writebit(unsigned char c);
extern unsigned char i2c_hw_readbit(void);
extern void i2c_hw_start(void);
extern void i2c_hw_stop(void);
extern unsigned char i2c_hw_write(unsigned char c);
extern unsigned char i2c_hw_read(unsigned char ack);

#endif // I2C_H
