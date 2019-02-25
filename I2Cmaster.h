/*
 * I2Cmaster.h
 *
 * Created: 11/27/2014 2:31:02 PM
 *  Author: niraj
 */


#ifndef I2CMASTER_H_
#define I2CMASTER_H_

#include <util/twi.h>

#define F_CPU 16000000UL
#define SCL_CLOCK  100000L
#define I2C_WRITE   0
#define I2C_READ    1

void i2c_init(void);
void i2c_stop(void);
unsigned char i2c_start(unsigned char addr);
unsigned char i2c_rep_start(unsigned char addr);
unsigned char i2c_write(unsigned char data);
unsigned char i2c_readrep(void);
unsigned char i2c_read(void);

#endif /* I2CMASTER_H_ */



