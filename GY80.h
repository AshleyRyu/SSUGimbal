/*
 * GY80.h    Created on: 2017. 11. 7.
 *
 * GY80 readout Vector float x[9]
 * x[0] : pitch axis angle ( gravity dir from accelerometer )
 * x[1] : pitch axis angular velocity (gyro)
 * x[2] : pitch axis angular acceleration
 *
 * x[3] : yaw axis angle ( gravity dir from accelerometer )
 * x[4] : yaw axis angular velocity (gyro)
 * x[5] : yaw axis angular acceleration
 *
 * x[6] : roll axis angle ( gravity dir from accelerometer )
 * x[7] : roll axis angular velocity (gyro)
 * x[8] : roll axis angular acceleration
  */

#ifndef MYSRC_GY80_H_
#define MYSRC_GY80_H_

#endif /* MYSRC_GY80_H_ */
#define PD0 0
#define PD1 1
#define PD2 2
#define PE1 3
#define PE2 4
#define PE3 5
#define HIGH 0x01
#define LOW  0x0
#define ACCEL_SLAVE_ADDR 0x53
#define ACCEL_X1_DATA 0x32
#define GYRO_SLAVE_ADDR 0x69
#define GYRO_X1_DATA 0x28
#define COMP_SLAVE_ADDR 0x1E
#define COMP_X1_DATA 0x03
#define declination_angle -25.905

void GetGY80(float *x);


void InitGY80();

void SetAccelPowerMode(void);
void SetGyroPowerMode(void);
int16_t getAccl_raw(uint8_t);
int16_t getGyro_raw(uint8_t);
void SetCompassPowerMode();
int16_t getComp_raw(uint8_t);
