/*
 * I2Cmaster.c
 *
 * Created: 11/27/2014 2:29:47 PM
 *  Author: niraj
 */

#include "I2Cmaster.h"


void i2c_init(void)
{
	//TWBR=0x01; // Bit rate
	TWBR = ((F_CPU/SCL_CLOCK)-16)/2;  // Bit rates
	TWSR=(0<<TWPS1)|(0<<TWPS0); // Setting prescalar bits

}
void i2c_stop(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);       // send stop condition
	while(TWCR & (1<<TWSTO));   // wait until stop condition is executed and bus released
}
unsigned char i2c_start(unsigned char address)
{
	unsigned char  twst;
	TWCR = (1<<TWINT) | (1<<TWSTA) | (1<<TWEN);  // send START condition
	while(!(TWCR & (1<<TWINT)));  // wait until start transmission is completed
	twst =TWSR & 0xF8;
	if ( (twst != TW_START) && (twst != TW_REP_START)) return 1; //wait for the acknowledgement


	TWDR = address;  // send device address plus read or write instruction
	TWCR = (1<<TWINT) | (1<<TWEN);    // Clear i2c interrupt flag, enable i2c
	while(!(TWCR & (1<<TWINT))); // wail until transmission completed and ACK/NACK has been received
	twst = TWSR & 0xF8;
	if ( (twst != TW_MT_SLA_ACK)&& (twst != TW_MR_SLA_ACK)) return 1;   //wait for the acknowledgement

	return 0;

}
unsigned char i2c_rep_start(unsigned char addr)
{
	return i2c_start( addr );
}

unsigned char i2c_write(unsigned char data)
{
	unsigned char twst;


	TWDR = data;  // send data to the previously addressed device
	TWCR = (1<<TWINT) | (1<<TWEN);
	while(!(TWCR & (1<<TWINT))); // wait until transmission completed

	twst = TWSR & 0xF8;
	if( twst != TW_MT_DATA_ACK) return 1;   //wait for the acknowledgement

	return 0;
}
unsigned char i2c_read(void)
{
	TWCR=(1<<TWINT)|(1<<TWEN);
	while (!(TWCR & (1<<TWINT)));   //wait until data is read

	return TWDR;
}
unsigned char i2c_readrep(void)
{
	TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWEA);
	while(!(TWCR & (1<<TWINT)));

	return TWDR;
}
