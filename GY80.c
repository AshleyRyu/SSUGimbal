#pragma once
/*
 * GY80.c
 *
 *  Created on: 2017. 11. 7.
 *      Author: WCSYS_01
 */

#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"

//---- Internal heads -------------------
#include "GY80.h"
#include "wcsysIO.h"
//----------------------------------------------//
const float pi_h = 3.141592/2.;
float ma_x = 0.;
float ma_y = 0.;
float ma_z = 0.;

/*
 * GY80.h    Created on: 2017. 11. 7.
 *
 * GY80 readout Vector float x[9]
 * x[0] : roll axis angle ( gravity dir from accelerometer )
 * x[1] : roll axis angular velocity (gyro)
 * x[2] : roll axis angular acceleration
 *
 * x[3] : pitch axis angle ( gravity dir from accelerometer )
 * x[4] : pitch axis angular velocity (gyro)
 * x[5] : pitch axis angular acceleration
 *
 * x[6] : yaw axis angle ( gravity dir from accelerometer )
 * x[7] : yaw axis angular velocity (gyro)
 * x[8] : yaw axis angular acceleration
  */

void GetGY80(float *x)
{
    int i;
    // Slight Averging
    const float Navg = 4.;
    const float ma = 1./Navg;
    const float mb = 1.- ma;

    float ax = (float) getAccl_raw(0);
    float ay = (float) getAccl_raw(1);
    float az = (float) getAccl_raw(2);

    float theta_x = atan2(ax,az)/pi_h;
    float theta_y = atan2(-ay,az)/pi_h;
    float theta_z = atan2(ay,ax)/pi_h;

    ma_x = mb*ma_x + ma*theta_x;
    ma_y = mb*ma_y + ma*theta_y;
    ma_z = mb*ma_z + ma*theta_z;

    // angle
    x[0] = ma_x;
    x[3] = ma_y;
    x[6] = ma_z;

    // angluar acceleration
    x[2] = theta_x -ma_x;
    x[5] = theta_y -ma_y;
    x[8] = theta_z -ma_z;


    // anglar  velocity (gyro)
 //   x[1] = (float)getGyro_raw(0);
 //   x[4] = (float)getGyro_raw(1);
    x[1] = (float)getGyro_raw(0)/6000.;
    x[4] = (float)getGyro_raw(1)/6000.;
    x[7] = (float)getGyro_raw(2)/6000.;

    // compass
//    x[2] = (float)getComp_raw(0);
//    x[5] = (float)getComp_raw(1);
//    x[8] = (float)getComp_raw(2);

    x[2] = 0.;
    x[5] = 0.;
    x[8] = 0.;

}


int64_t rawAX, rawAY, rawAZ;
int32_t rawGX, rawGY, rawGZ;
int32_t rawCX, rawCY, rawCZ;
signed long Comp_X = 0;
signed long Comp_Y = 0;
signed long Comp_Z = 0;

void InitGY80()
{
    SetAccelPowerMode();
    SetGyroPowerMode();
    SetCompassPowerMode();

}

//--------------- ACC ------------------------------------------------------//
void SetAccelPowerMode()
{
    writeI2C0(ACCEL_SLAVE_ADDR, 0x2C, 0x0C); //set 400Hz output data rate  -> 2.5 ms
    writeI2C0(ACCEL_SLAVE_ADDR, 0x2D, 0x08); //set power register as measurement mode
    writeI2C0(ACCEL_SLAVE_ADDR, 0x31, 0x09); //set 4g range   // set data format register as full resolution

}

int16_t getAccl_raw(uint8_t coord) // 0:X, 1:Y, 2:Z
{

    int16_t accelData1, accelData2;
    //accelrawAX = readI2C0(ACCEL_SLAVE_ADDR, ACCEL_X1_DATA);
    accelData1 =(uint8_t) readI2C0(ACCEL_SLAVE_ADDR, ACCEL_X1_DATA+2*coord);
    accelData2 =(uint8_t) readI2C0(ACCEL_SLAVE_ADDR, ACCEL_X1_DATA+2*coord+1);
    rawAX = (accelData2&0xff)<<8;
    rawAX = rawAX | (accelData1&0xff);
    if( (rawAX & 0x400)!= 0x0) rawAX = rawAX|0xf800;
    return rawAX;

}
//------------------ GRYO ----------------------------------------------------//

void SetGyroPowerMode()
{

    writeI2C0(GYRO_SLAVE_ADDR, 0x20, 0x8F); //set ODR as 400Hz -> 2.5ms
    writeI2C0(GYRO_SLAVE_ADDR, 0x21, 0x00);
    writeI2C0(GYRO_SLAVE_ADDR, 0x22, 0x08);
    writeI2C0(GYRO_SLAVE_ADDR, 0x23, 0x10);
    writeI2C0(GYRO_SLAVE_ADDR, 0x24, 0x10);

}

int16_t getGyro_raw(uint8_t coord) // 0:X, 1:Y, 2:Z
{
    int16_t gyroData1, gyroData2;
    //accelrawAX = readI2C0(ACCEL_SLAVE_ADDR, ACCEL_X1_DATA);
    gyroData1 =(uint8_t) readI2C0(GYRO_SLAVE_ADDR, GYRO_X1_DATA+2*coord);
    gyroData2 =(uint8_t) readI2C0(GYRO_SLAVE_ADDR, GYRO_X1_DATA+2*coord+1);
    rawGX = (gyroData2&0xff)<<8;
    rawGX = rawGX | (gyroData1&0xff);
    //if( (rawGX & 0x400)!= 0x0) rawGX = rawGX|0xf800;
    return rawGX;
}

//------------------- COMPASS --------------------------------------------//
void SetCompassPowerMode()
{
    writeI2C0(COMP_SLAVE_ADDR, 0x00, 0x1C); //select number of samples averaged (1 to 8) per measurement output.
                                            //00:default, 01:2, 10:4, 11:8
                                            //set ODR as 75Hz(maximum) -> 13.3 ms
    writeI2C0(COMP_SLAVE_ADDR, 0x01, 0x00); //clear configration
    writeI2C0(COMP_SLAVE_ADDR, 0x02, 0x00); //set mode register as continuous measurement mode
    //writeI2C0(COMP_SLAVE_ADDR, 0x21, 0x00);
}
int16_t getComp_raw(uint8_t coord) // 0:X, 1:Y, 2:Z
{

    int16_t compData1, compData2;
    //accelrawAX = readI2C0(ACCEL_SLAVE_ADDR, ACCEL_X1_DATA);
    compData1 =(uint8_t) readI2C0(COMP_SLAVE_ADDR, COMP_X1_DATA+2*coord);
    compData2 =(uint8_t) readI2C0(COMP_SLAVE_ADDR, COMP_X1_DATA+2*coord+1);
    rawCX = (compData2&0xff)<<8;
    rawCX = rawCX | (compData1&0xff);
//        if( (rawGX & 0x400)!= 0x0) rawGX = rawGX|0xf800;
        return rawCX;

}




