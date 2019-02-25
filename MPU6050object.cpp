/*
 * MPU6050object.cpp
 *
 * Created: 2/11/2015 3:42:33 PM
 *  Author: Niraj
 */ 


#define F_CPU 16000000UL
#include <avr/io.h>
#include "lcd.h"
#include "I2Cmaster.h"
#include "IMU_MPU6050.h"
#include <util/delay.h>
#include "math.h"
#include "Complementary_filter.h"

void timer1setup() ;

int main(void)
{
	float pitch,temperature,pitchrate;
	int zacc,xacc;
	timer1setup();
	MPU6050 accelgyro;
	accelgyro.initialize();
	accelgyro.SetAccelRange(MPU6050_ACCEL_FS_2);
	accelgyro.SetGyroRange(MPU6050_GYRO_FS_500);
	lcd_init();
	
	sei();
    while(1)
    {
		accelgyro.Read_MPU6050();
		temperature=accelgyro.readTemperature();
		
		zacc=accelgyro.accel_getZaxis();
		xacc=accelgyro.accel_getXaxis();
		Printf("Z=%dX=%d",zacc,xacc);
		//pitch=atan2(-(double)xacc,(double)zacc)*180*7/22.0;
		pitch=accelgyro.get_pitch();
		pitchrate=accelgyro.get_pitchrate();
       lcd_clear();
		Printf("\nOCR1A=%d",OCR1A);
		//Printf("\nTemp=%f",temperature);
		_delay_ms(200);
    }
}

void timer1setup()   //10 ms timer1
{
	DDRD|=(1<<PIND4)|(1<<PIND5);
	TCCR1A|=(1<<COM1A1)|(1<<COM1B1)|(1<<WGM11);
	TCCR1B|=(1<<WGM12)|(1<<WGM13)|(1<<CS10);     //1 PRESCALER
	ICR1=2000; 
	OCR1A=1500;
	OCR1B=1000;
}