#ifndef IMU_MPU6050_H
#define IMU_MPU6050_H

#include <avr/interrupt.h>
#include "math.h"
#include "I2Cmaster.h"

#define MPU6050_ADDRESS_AD0_LOW  0x68     //Pull address pin AD0  to logic low
#define MPU6050_ADDRESS_AD0_HIGH 0x69     //Pull address pin AD0  to logic high
#define MPU6050_DEFAULT_ADDRESS  MPU6050_ADDRESS_AD0_LOW
#define MPU6050_READ   0xD1
#define MPU6050_WRITE  0xD0

#define MPU6050_SMPLRT_DIV   0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_TEMP_OUT_H   0x41
#define MPU6050_TEMP_OUT_L   0x42
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_GYRO_XOUT_L  0x44
#define MPU6050_GYRO_YOUT_H  0x45
#define MPU6050_GYRO_YOUT_L  0x46
#define MPU6050_GYRO_ZOUT_H  0x47
#define MPU6050_GYRO_ZOUT_L  0x48

#define MPU6050_INT_PIN_CFG  0x37
#define MPU6050_INT_ENABLE   0x38

#define MPU6050_GYRO_FS_250         0x00
#define MPU6050_GYRO_FS_500         0x01
#define MPU6050_GYRO_FS_1000        0x02
#define MPU6050_GYRO_FS_2000        0x03

#define MPU6050_ACCEL_FS_2          0x00
#define MPU6050_ACCEL_FS_4          0x01
#define MPU6050_ACCEL_FS_8          0x02
#define MPU6050_ACCEL_FS_16         0x03

#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06

#define MPU6050_USER_CTRL        0x6A
#define MPU6050_PWR_MGMT_1       0x6B
#define MPU6050_PWR_MGMT_2       0x6C

#define MPU6050_CLOCK_INTERNAL          0x00
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_CLOCK_PLL_YGYRO         0x02
#define MPU6050_CLOCK_PLL_ZGYRO         0x03

#define MPU6050_INTMODE_ACTIVEHIGH  0x00
#define MPU6050_INTMODE_ACTIVELOW   0x01

#define MPU6050_INTDRV_PUSHPULL     0x00
#define MPU6050_INTDRV_OPENDRAIN    0x01

#define MPU6050_FIFO_COUNTH      0x72
#define MPU6050_FIFO_COUNTL      0x73
#define MPU6050_FIFO_R_W         0x74

#define MPU6050_WHO_AM_I     0x75

class MPU6050
{
private:
    float accel_roll;
	float accel_pitch;
    float gyro_rateX;
    float gyro_rateY;
    float gyro_rateZ;
	float gyro_sensitivity;
	int temperature;

public:
    void initialize();
    unsigned char MPU6050_readRegister(unsigned char register_addr);
    void MPU6050_writeRegister(unsigned char register_addr,unsigned char data);
    void SetAccelRange(unsigned char acrange);
    void SetGyroRange(unsigned char gyrange);
    void Calibrate_accel();
    void Calibrate_gyro();
    void Read_raw_accel_angle();
    void Read_raw_gyro_rate();
    void setDLPF_bandwidth();
	void Read_MPU6050();

    void setSampleRateDivider(uint8_t rate);  // SMPLRT_DIV register
    void setDLPFMode(uint8_t bandwidth);
    void setDHPFMode(uint8_t mode);

    float readTemperature();  // TEMP_OUT register reading
    
	void get_roll_pitch(float *roll,float *pitch);
	void get_gyrorateXYZ(float *rx,float *ry,float *rz);
	float get_pitch();
	float get_pitchrate();
	int accel_getXaxis();
	int accel_getZaxis();


};







#endif
