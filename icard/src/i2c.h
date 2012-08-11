
#ifndef I2C_H
#define I2C_H

// Generic API
typedef struct
{
	void (*init)(void);
	void (*start)(void);
	void (*stop)(void);
	unsigned char (*write)(unsigned char c);
	unsigned char (*read)(unsigned char ack);
} i2c_device_t;


#define I2C_IFC_SPF1 0
#define I2C_IFC_SPF1 1
#define I2C_IFC_FAN 2

/**
 * Use I2C_IFC_X for index in the array
 */
extern const i2c_device_t i2c_devices[3];

/**
 * Set of functions which call the correct
 * I2C API
 */

inline void i2c_init(const i2c_device_t *device);
inline void i2c_start(const i2c_device_t *device);
inline void i2c_stop(const i2c_device_t *device);
inline unsigned char i2c_write(const i2c_device_t *device, unsigned char c);
inline unsigned char i2c_read(const i2c_device_t *device, unsigned char ack);

inline void i2c_init(const i2c_device_t *device)
{
	device->init();
}

inline void i2c_start(const i2c_device_t *device)
{
	device->start();
}

inline void i2c_stop(const i2c_device_t *device)
{
	device->stop();
}

inline unsigned char i2c_write(const i2c_device_t *device, unsigned char c)
{
	device->write(c);
}

inline unsigned char i2c_read(const i2c_device_t *device, unsigned char ack)
{
	device->read(ack);
}

// Bitbang API
extern void i2c_sw_init(void);
extern void i2c_sw_start(void);
extern void i2c_sw_stop(void);
extern unsigned char i2c_sw_write(unsigned char c);
extern unsigned char i2c_sw_read(unsigned char ack);

// Two wire interface (TWI) driver API
extern void i2c_hw_init(void);
extern void i2c_hw_start(void);
extern void i2c_hw_stop(void);
extern unsigned char i2c_hw_write(unsigned char c);
extern unsigned char i2c_hw_read(unsigned char ack);

#endif // I2C_H
