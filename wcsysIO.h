/*
 * wcsysIO.h
 *
 *  Created on: 2017. 11. 7.
 *      Author: WCSYS_01
 */

#ifndef MYSRC_WCSYSIO_H_
#define MYSRC_WCSYSIO_H_

#endif /* MYSRC_WCSYSIO_H_ */
void clk_set();
void i2c_init();
void gpio_init();
void adc_init();
void pwm_init();
void ConfigureUART();

void InitWCSYS();
void writeI2C0(uint8_t device_address, uint8_t device_register, uint8_t device_data);
int8_t readI2C0(uint16_t device_address, uint16_t device_register);
