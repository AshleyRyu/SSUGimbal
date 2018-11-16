#pragma once
/*
 * wcsysIO.c
 *
 *  Created on: 2017. 11. 7.
 *      Author: WCSYS_01
 */
/*---------------------------------------------
  GPIO_Pin     Pin_Function     Device
    PF4            GPIO          SW1
    PF0            GPIO          SW2
    PF1            GPIO      RGB LED(Red)
    PF2            GPIO      RGB LED(Blue)
    PF3            GPIO      RGB LED(Green)
    ------------------------------------------
    PC4            PWM         SERVO0
    PC5            PWM         SERVO1 @
    PE5            PWM         SERVO2 @ //같이 물려있음
    PD0            DOUT
    PD1            DOUT
    PD2            DOUT
    PE1            DIN
    PB2            I2C
    PB3            I2C
    PE2            ADC
    PE3            ADC         *not use
    PA0            UART
    PA1            UART

-----------------------------------------------*/
#include <stdint.h>
#include <math.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/i2c.h"
#include "driverlib/adc.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/rom.h"


#include "wcsysIO.h"

// ------- Main Initialization -------------------------------
void InitWCSYS()
{
    clk_set();
    i2c_init();
    gpio_init();
    adc_init();
    pwm_init();
    ConfigureUART();
}
//------------------------------------------------------------
void clk_set()
{
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
}
void gpio_init()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD));
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOE));
    GPIOPinTypeGPIOOutput(GPIO_PORTD_BASE, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2);
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1);
}
void pwm_init()
{
    //Configure PWM Clock to match system
   SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
    //Enable the PWM0 peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    //Wait for the PWM0 module to be ready.
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_PWM0));
    //HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    //HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    GPIOPinConfigure(GPIO_PE5_M0PWM5);
    GPIOPinConfigure(GPIO_PC4_M0PWM6);
    GPIOPinConfigure(GPIO_PC5_M0PWM7);
    GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_5);
    GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5);
    //GPIOPinTypePWM(GPIO_PORTC_BASE, GPIO_PIN_5);
    //Configure the PWM generator for count down mode with immediate
    //updates to the parameters.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    PWMGenConfigure(PWM0_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN|PWM_GEN_MODE_NO_SYNC);
    //These settings are specifically designed to run servo motors
    //which expect 20mS period with between 1ms and 2ms high time
    //System clock is 16MHz with PWM divider of 64
    // 16000000/64 = 250000/50 = 5000  ### 1S/50 = 20mS thats where divisor comes from
    // Set high time to 2mS
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, 5000);
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_3, 5000);
    // Set the pulse width of PWM1 for a 10% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_5, 500); //PB7 M0PWM1
    // Set the pulse width of PWM6 for a 10% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_6, 500); //PC4 M0PWM6
    // Set the pulse width of PWM7 for a 20% duty cycle.
    PWMPulseWidthSet(PWM0_BASE, PWM_OUT_7, 1000); //PC5 M0PWM7
    // Start the timers in generator 3.
    PWMGenEnable(PWM0_BASE, PWM_GEN_2);
    PWMGenEnable(PWM0_BASE, PWM_GEN_3);
    // Enable the outputs.
    PWMOutputState(PWM0_BASE, (PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT), true);
    //DataSheet
    //Table 20-1. PWM Signals (64LQFP) --> pwm module , generator , pin number
}
void adc_init()
{
    //
    // The ADC0 peripheral must be enabled for use.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
    //
    // For this example ADC0 is used with AIN0/1 on port E7/E6.
    // The actual port and pins used may be different on your part, consult
    // the data sheet for more information.  GPIO port E needs to be enabled
    // so these pins can be used.
    // TODO: change this to whichever GPIO port you are using.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    //
    // Select the analog ADC function for these pins.
    // Consult the data sheet to see which functions are allocated per pin.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    //
    // Enable sample sequence 3 with a processor signal trigger.  Sequence 3
    // will do a single sample when the processor sends a signal to start the
    // conversion.  Each ADC module has 4 programmable sequences, sequence 0
    // to sequence 3.  This example is arbitrarily using sequence 3.
    //
    ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
    //
    // Configure step 0 on sequence 3.  Sample channel 0 (ADC_CTL_CH0) in
    // differential mode (ADC_CTL_D) and configure the interrupt flag
    // (ADC_CTL_IE) to be set when the sample is done.  Tell the ADC logic
    // that this is the last conversion on sequence 3 (ADC_CTL_END).  Sequence
    // 3 has only one programmable step.  Sequence 1 and 2 have 4 steps, and
    // sequence 0 has 8 programmable steps.  Since we are only doing a single
    // conversion using sequence 3 we will only configure step 0.   For more
    // information on the ADC sequences and steps, refer to the datasheet.
    //

    ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH1 | ADC_CTL_END | ADC_CTL_IE);
    //
    // Since sample sequence 3 is now configured, it must be enabled.
    //
    ADCSequenceEnable(ADC0_BASE, 0);
    //
    // Clear the interrupt status flag.  This is done to make sure the
    // interrupt flag is cleared before we sample.
    //
    ADCIntClear(ADC0_BASE, 0);

}

void i2c_init()
{

     // Enable the I2C0 peripheral
     SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

     // Wait for the I2C0 module to be ready.
     while(!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0));

     //reset module
     SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
     //GPIO port B needs to be enabled so these pins can be used.
     SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

     // Configure the pin muxing for I2C0 functions on port B2 and B3.
     GPIOPinConfigure(GPIO_PB2_I2C0SCL);
     GPIOPinConfigure(GPIO_PB3_I2C0SDA);

     // Configures pin for use as SDA by the I2C peripheral.GPIO_PB3_I2C0SDA
     GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
     // Configures pin for use as SCL by the I2C peripheral.GPIO_PB2_I2C0SCL
     GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

     //  Don't know HOW TO USE
     I2CMasterEnable(I2C0_BASE);

     // Initialize Master and Slave
     // it will sets the bus speed and enables the master module.
     I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), true); // set 100kbps

}

void ConfigureUART(void)
{

    // Enable the GPIO Peripheral used by the UART.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);


    // Enable UART0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);


    // Configure GPIO Pins for UART mode.
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    // Use the internal 16MHz oscillator as the UART clock source.
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    // Initialize the UART for console I/O.
    UARTStdioConfig(0, 9600, 16000000);

}

void writeI2C0(uint8_t device_address, uint8_t device_register, uint8_t device_data)
{

  //specify that we want to communicate to device address with an intended write to bus
  I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);

  //register to be read
  I2CMasterDataPut(I2C0_BASE, device_register);

  //send control byte and register address byte to slave device
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

  //wait for MCU to finish transaction
  while ( !(I2CMasterIntStatusEx(I2C0_BASE, false) & I2C_MASTER_INT_DATA) );
  while(I2CMasterBusy(I2C0_BASE));

  I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);

  //specify data to be written to the above mentioned device_register
  I2CMasterDataPut(I2C0_BASE, device_data);

  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);

  while(I2CMasterBusy(I2C0_BASE));

  //wait while checking for MCU to complete the transaction
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

  //wait for MCU & device to complete transaction
  while(I2CMasterBusy(I2C0_BASE));

}

//single byte read from slave
int8_t readI2C0(uint16_t device_address, uint16_t device_register)
{

  //specify that we want to communicate to device address with an intended write to bus
  I2CMasterSlaveAddrSet(I2C0_BASE, device_address, false);

  //the register to be read
  I2CMasterDataPut(I2C0_BASE, device_register);

  //send control byte and register address byte to slave device
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_SEND);

  //wait for MCU to complete send transaction
  while(I2CMasterBusy(I2C0_BASE));

  //read from the specified slave device
  I2CMasterSlaveAddrSet(I2C0_BASE, device_address, true);

  //send control byte and read from the register from the MCU
  I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

  //wait while checking for MCU to complete the transaction
  while(I2CMasterBusy(I2C0_BASE));

  //Get the data from the MCU register and return to caller
  return( I2CMasterDataGet(I2C0_BASE));

}
