#include "IMU_MPU6050.h"

struct accel_data
{
    int XAxis;
	int YAxis;
	int ZAxis;
}raw_accel;

struct gyro_data
{
	int XAxis;
	int YAxis;
	int ZAxis;
}raw_gyro;

void MPU6050::initialize()
{
    raw_accel.XAxis=raw_accel.YAxis=raw_accel.ZAxis=0;
    raw_gyro.XAxis=raw_gyro.YAxis=raw_gyro.ZAxis=0;
    accel_roll=0;
	accel_pitch=0;
    gyro_rateX=0;
	gyro_rateY=0;
	gyro_rateZ=0;
    temperature=0;
	gyro_sensitivity=131;
	
    i2c_init();

    setSampleRateDivider(0x07);

    i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_INT_PIN_CFG);   //set pointer to interrupt enable register
	i2c_write(0x00);                  //set  interrupt enable
	i2c_stop();

	i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_INT_ENABLE);   //set pointer to interrupt sources enable register
	i2c_write(0x00);                  //set data ready interrupt
	i2c_stop();

	i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_PWR_MGMT_1);   //set pointer to power management register 1
	i2c_write(MPU6050_CLOCK_PLL_XGYRO);                 //set clock to gyro clock reference
	i2c_stop();

}
unsigned char  MPU6050::MPU6050_readRegister(unsigned char register_addr)
 {
	 char tempdata;
     i2c_start(MPU6050_WRITE);
     i2c_write(register_addr);
     i2c_stop();
     i2c_rep_start(MPU6050_READ);
     tempdata=i2c_read();
     i2c_stop();
	 return tempdata;
}

void  MPU6050::MPU6050_writeRegister(unsigned char register_addr,unsigned char data)
{
    i2c_start(MPU6050_WRITE);
    i2c_write(register_addr);
    i2c_write(data);
    i2c_stop();
}

void MPU6050::SetAccelRange(unsigned char acrange)
{
    i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_ACCEL_CONFIG);   //set pointer to accelerometer configuration register
	i2c_write(acrange);                  //set accelerometer scale to 2g
	i2c_stop();
}

void  MPU6050::SetGyroRange(unsigned char gyrange)
{
    i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_GYRO_CONFIG);   //set pointer to gyro configuration register
	i2c_write(gyrange);                  //set gyro scale to 500 degree/second
	i2c_stop();
	
	switch (gyrange)
	{
		case 0x00: gyro_sensitivity=131;
		           break;
	    case 0x01: gyro_sensitivity=65.5;
		           break;
	    case 0x02: gyro_sensitivity=32.8;
		           break;
	    case 0x03: gyro_sensitivity=16.4;
		           break;
	}

}

 void MPU6050::setSampleRateDivider(unsigned char rate)
 {
    i2c_start(MPU6050_WRITE);
	i2c_write(MPU6050_SMPLRT_DIV);     // set pointer to sample rate divider
	i2c_write(rate);                   //e.g.when divider=7, sample rate becomes 8000/(1+7)=1000HZ
	i2c_stop();
 }

void MPU6050::Calibrate_accel()
{

}

void MPU6050::Calibrate_gyro()
{

}

void MPU6050::Read_raw_accel_angle()
{
raw_accel.XAxis=(MPU6050_readRegister(MPU6050_ACCEL_XOUT_H))<<8;
raw_accel.XAxis |=(MPU6050_readRegister(MPU6050_ACCEL_XOUT_L));

raw_accel.YAxis=(MPU6050_readRegister(MPU6050_ACCEL_YOUT_H))<<8;
raw_accel.YAxis |=(MPU6050_readRegister(MPU6050_ACCEL_YOUT_L));

raw_accel.ZAxis=(MPU6050_readRegister(MPU6050_ACCEL_ZOUT_H))<<8;
raw_accel.ZAxis |=(MPU6050_readRegister(MPU6050_ACCEL_ZOUT_L));
}

void MPU6050::Read_raw_gyro_rate()
{
raw_gyro.XAxis = (MPU6050_readRegister(MPU6050_GYRO_XOUT_H))<<8;
raw_gyro.XAxis |=(MPU6050_readRegister(MPU6050_GYRO_XOUT_L));

raw_gyro.YAxis = (MPU6050_readRegister(MPU6050_GYRO_YOUT_H))<<8;
raw_gyro.YAxis |= (MPU6050_readRegister(MPU6050_GYRO_YOUT_L));

raw_gyro.ZAxis = (MPU6050_readRegister(MPU6050_GYRO_ZOUT_H))<<8;
raw_gyro.ZAxis |= (MPU6050_readRegister(MPU6050_GYRO_ZOUT_L));
}

float MPU6050::readTemperature()
{
float temp;
temperature   = (MPU6050_readRegister(MPU6050_TEMP_OUT_H))<<8;
temperature |= (MPU6050_readRegister(MPU6050_TEMP_OUT_L));
temp=(float)temperature/340+36.53;
return temp;

}
void MPU6050::setDLPF_bandwidth()
{

}

 void MPU6050::Read_MPU6050()
 {
	 Read_raw_accel_angle();
	 Read_raw_gyro_rate();
	 
	// accel_roll= 57.295*atan((float)raw_accel.YAxis/ sqrt(pow((float)raw_accel.ZAxis,2)+pow((float)raw_accel.XAxis,2)));  //180/PI =57.295
	// accel_pitch = 57.295*atan((float)-raw_accel.XAxis/ sqrt(pow((float)raw_accel.ZAxis,2)+pow((float)raw_accel.YAxis,2)));
	  accel_roll= 57.295*atan2((float)raw_accel.YAxis,(float)raw_accel.ZAxis);  //180/PI =57.295
	  accel_pitch = 57.295*atan2((float)-raw_accel.XAxis,(float)raw_accel.ZAxis);
	 
	 gyro_rateX = (float)raw_gyro.XAxis/gyro_sensitivity;
	 gyro_rateY = (float)raw_gyro.YAxis/gyro_sensitivity;
	 gyro_rateZ = (float)raw_gyro.ZAxis/gyro_sensitivity;
	 
 }
 float MPU6050::get_pitch()
 {
	 return accel_pitch;	 
 }
 float  MPU6050::get_pitchrate()
 {
	 return gyro_rateY;
 }
 void MPU6050::get_roll_pitch(float *roll,float *pitch)
 {
	 *roll=accel_roll;
	 *pitch=accel_pitch;
 }
 void MPU6050::get_gyrorateXYZ(float *rx,float *ry,float *rz)
 {
	 *rx=gyro_rateX;
	 *ry=gyro_rateY;
	 *rz=gyro_rateZ;
 }
  int MPU6050::accel_getZaxis()
  {
	  return raw_accel.ZAxis;
  }
  int MPU6050::accel_getXaxis()
  {
	  return raw_accel.XAxis;
  }