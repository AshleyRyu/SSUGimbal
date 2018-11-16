/*
 * Servos.h
 *
 *  Created on: 2017. 11. 7.
 *      Author: WCSYS_01
 */

#ifndef MYSRC_SERVOS_H_
#define MYSRC_SERVOS_H_
#endif /* MYSRC_SERVOS_H_ */

#define SERVO0 0  //PC4
#define SERVO1 1  //PC5
#define SERVO2 2  //PB7

// Servo Parameters
#define SERVO_ROLL_MAX 185
#define SERVO_ROLL_OFS 119//119
#define SERVO_ROLL_MIN 48
#define SERVO_PTCH_MAX 146
#define SERVO_PTCH_OFS 77
#define SERVO_PTCH_MIN 9
#define SERVO_YAW_MAX  146
#define SERVO_YAW_OFS  78//78

void SetServoPos(float *pos);
//
// pos[0] : roll  position [-1,1]
// pos[1] : yaw   position [-1,1]
// pos[2] : pitch position [-1,1]
//
void Servo_write(int , int);
