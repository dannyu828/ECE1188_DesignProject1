// Motor.c
// Runs on MSP432
// Provide mid-level functions that initialize ports and
// set motor speeds to move the robot. Lab 13 solution
// Daniel Valvano
// July 11, 2019

/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyright (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/

Simplified BSD License (FreeBSD License)
Copyright (c) 2019, Jonathan Valvano, All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are
those of the authors and should not be interpreted as representing official
policies, either expressed or implied, of the FreeBSD Project.
*/

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include <stdint.h>
#include "msp.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"

void SysTick_Wait1us(uint32_t delay){
    SysTick->LOAD = (delay*48 - 1);// count down to zero
    SysTick->VAL = 0;          // any write to CVR clears it and COUNTFLAG in CSR
    while(( SysTick->CTRL&0x00010000) == 0){};
}

void Motor_Init(void){
// Initializes the 6 GPIO lines and puts driver to sleep
// Returns right away
// initialize P5.4 and P5.5 and make them outputs

    // Set P2.6 and P2.7 as outputs for EN ports
    P2->SEL0 &= ~0xC0;
    P2->SEL1 &= ~0xC0;
    P2->DIR |= 0xC0;
    P2->REN &= ~0xC0;

    // Set P3.6 and P3.7 as outputs for nSLEEP ports
    P3->SEL0 &= ~0xC0;
    P3->SEL1 &= ~0xC0;
    P3->DIR |= 0xC0;
    P3->REN &= ~0xC0;

    // Set P5.4 and P5.5 as outputs for PH ports
    P5->SEL0 &= ~0x30;
    P5->SEL1 &= ~0x30;
    P5->DIR  |= 0x30;
    P5->REN &= ~0x30;

    // Put motors to sleep to prevent any movement during initialization
    P3->OUT &= ~0xC0;

    // Initialize PWM
    PWM_Init34(15000, 2000, 2000);
}

void Motor_Stop(void){
// Stops both motors, puts driver to sleep
// Returns right away

    P2->OUT &= ~0xC0;
    P3->OUT &= ~0xC0;
    P5->OUT &= ~0x30;
}

void Motor_Forward(uint16_t dutyLeft, uint16_t dutyRight){
// Drives both motors forward at duty (100 to 9900)

    // Unsleep motors and set direction forward
    P3->OUT |= 0xC0;
    P5->OUT &= ~0x30;

    // Set PWM duty cycles
    PWM_Duty3(dutyRight);
    PWM_Duty4(dutyLeft);
}
void Motor_Backward(uint16_t dutyLeft, uint16_t dutyRight){
// Drives both motors backward at duty (100 to 9900)

    // Unsleep motors and set direction forward
    P3->OUT |= 0xC0;
    P5->OUT |= 0x30;

    // Set PWM duty cycles
    PWM_Duty3(dutyRight);
    PWM_Duty4(dutyLeft);
}

void Motor_Left(uint16_t dutyLeft, uint16_t dutyRight){
// Drives just the left motor forward at duty (100 to 9900)
// Right motor is stopped (sleeping)

    // Sleep right, unsleep left, set left forward
    P3->OUT &= ~0x40;
    P3->OUT |= 0x80;
    P5->OUT &= ~0x10;

    // Set PWM duty cycles
    PWM_Duty3(dutyRight);
    PWM_Duty4(dutyLeft);
}
void Motor_Right(uint16_t dutyLeft, uint16_t dutyRight){
// Drives just the right motor forward at duty (100 to 9900)
// Left motor is stopped (sleeping)

    // Sleep left, unsleep right, set left forward
    P3->OUT &= ~0x80;
    P3->OUT |= 0x40;
    P5->OUT &= ~0x20;

    // Set PWM duty cycles
    PWM_Duty3(dutyRight);
    PWM_Duty4(dutyLeft);
}
