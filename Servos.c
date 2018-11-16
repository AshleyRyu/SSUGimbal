/*
 * Servos.c
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
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

//---- Internal heads -------------------
#include "Servos.h"
#include "GY80.h"
#include "wcsysIO.h"
//--------------------------------------


const float ServoConv[3] =
{
 (float)(SERVO_ROLL_MAX - SERVO_ROLL_OFS), //pitch
 (float)(SERVO_PTCH_MAX  - SERVO_PTCH_OFS), //raw
 (float)(SERVO_YAW_MAX  - SERVO_YAW_OFS) //roll
};
const int ServoOFS[3] ={ SERVO_ROLL_OFS, SERVO_PTCH_OFS, SERVO_YAW_OFS};

int ServoVal[3] ={ SERVO_ROLL_OFS, SERVO_PTCH_OFS, SERVO_YAW_OFS};


void SetServoPos(float *pos)
{
//    pos[0]: pitch => roll
//    pos[1]: yaw  => pitch
//    pos[2]: roll
    int i;
    for( i = 0; i <3; i++ )
    {
        // Protection
        if(pos[i] <-1.) pos[i] = -1.;
        if(pos[i] > 1.) pos[i] = 1.;

        // Servo update
         ServoVal[i] = ServoOFS[i] + (int) ServoConv[i] *pos[i];
         Servo_write( i, ServoVal[i]);
    }

}
void Servo_write(int SERVO_N, int DUTY_CYCLE)
{
    //0~180 --> duty cycle : 3%~12% (in arduino)
    DUTY_CYCLE = 150+(DUTY_CYCLE*2.5); //convert integer to degree
    switch (SERVO_N)
    {

        case 0: //PC4
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, DUTY_CYCLE); //PC4 M0PWM6
            break;
        case 1: //PC5
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, DUTY_CYCLE); //PC5 M0PWM6
            break;
        case 2: //PB7
            PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, DUTY_CYCLE); //PB7 M0PWM1
            break;
        default:
            break;

    }

}
