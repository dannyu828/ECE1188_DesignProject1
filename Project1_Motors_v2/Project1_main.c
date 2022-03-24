// Lab13_Timersmain.c
// Runs on MSP432
// Student starter code for Timers lab
// Daniel and Jonathan Valvano
// July 11, 2019
// PWM output to motor
// Second Periodic interrupt

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
// Negative logic bump sensors
// P4.7 Bump5, left side of robot
// P4.6 Bump4
// P4.5 Bump3
// P4.3 Bump2
// P4.2 Bump1
// P4.0 Bump0, right side of robot

// Left motor direction connected to P5.4 (J3.29)
// Left motor PWM connected to P2.7/TA0CCP4 (J4.40)
// Left motor enable connected to P3.7 (J4.31)
// Right motor direction connected to P5.5 (J3.30)
// Right motor PWM connected to P2.6/TA0CCP3 (J4.39)
// Right motor enable connected to P3.6 (J2.11)

#include "msp.h"
#include "..\inc\Clock.h"
#include "..\inc\SysTick.h"
#include "..\inc\CortexM.h"
#include "..\inc\LaunchPad.h"
#include "..\inc\Motor.h"
#include "..\inc\TimerA1.h"
#include "..\inc\TExaS.h"


struct State {
  uint8_t out;                // 2-bit output
  uint16_t d1;
  uint16_t d2;
  uint16_t delay;              // time to delay in 1ms
  const struct State *next[8]; // Next if 2-bit input is 0-3
};
typedef const struct State State_t;

#define Center    &fsm[0]
#define Left1      &fsm[1]
#define Left2      &fsm[2]
#define Right1     &fsm[3]
#define Right2     &fsm[4]
#define HT1        &fsm[5]
#define HT2        &fsm[6]
#define Straight   &fsm[7]
#define Stop       &fsm[8]

State_t fsm[9]={
  {1, 1000, 1000, 50, {Center, Left1, Left1, Left1, Center, Right1, Right1, Right1}},  // Center
  {3, 1000, 0, 50, {HT1, Left2, Left2, Left2, Center, Right1, Right1, Right1}},  // Left1
  {3, 2000, 0, 50, {HT1, Left1, Left1, Left1, Center, Right1, Right1, Right1}},   // Left2
  {4, 0, 1000, 50, {HT2, Left1, Left1, Left1, Center, Right2, Right2, Right2}}, //Right1
  {4, 0, 2000, 50, {HT2, Left1, Left1, Left1, Center, Right1, Right1, Right1}}, //Right2
  {3, 3000, 0, 50, {Straight, Left1, Left1, Left1, Center, Right1, Right1, Right1}}, //HT1
  {4, 0, 3000, 50, {Straight, Left1, Left1, Left1, Center, Right1, Right1, Right1}}, //HT2
  {1, 1000, 1000, 50, {Stop, Left1, Left1, Left1, Center, Right1, Right1, Right1 }}, //Straight
  {0, 0, 0, 50, {Stop, Left1, Left1, Left1, Center, Right1, Right1, Right1}} //Stop
};

State_t *Spt;  // pointer to the current state
uint32_t Input;
uint32_t Output;

// Driver test
void TimedPause(uint32_t time){
  Clock_Delay1ms(time);          // run for a while and stop
  Motor_Stop();
  while(LaunchPad_Input()==0);  // wait for touch
  while(LaunchPad_Input());     // wait for release
}
int Program13_1(void){
  Clock_Init48MHz();
  LaunchPad_Init(); // built-in switches and LEDs
  Motor_Init();     // your function
  while(1){
    TimedPause(4000);
    Motor_Forward(7500,7500);  // your function
    TimedPause(2000);
    Motor_Backward(7500,7500); // your function
    TimedPause(3000);
    Motor_Left(5000,5000);     // your function
    TimedPause(3000);
    Motor_Right(5000,5000);    // your function
  }
}

// Test of Periodic interrupt
#define REDLED (*((volatile uint8_t *)(0x42098060)))
#define BLUELED (*((volatile uint8_t *)(0x42098068)))
uint32_t Time;
void Task(void){
  REDLED ^= 0x01;       // toggle P2.0
  REDLED ^= 0x01;       // toggle P2.0
  Time = Time + 1;
  REDLED ^= 0x01;       // toggle P2.0
}
int Program13_2(void){
  Clock_Init48MHz();
  LaunchPad_Init();  // built-in switches and LEDs
  TimerA1_Init(&Task,500);  // 1000 Hz
  EnableInterrupts();
  while(1){
    BLUELED ^= 0x01; // toggle P2.1
  }
}

int main(void){
    // write a main program that uses PWM to move the robot
    // like Program13_1, but uses TimerA1 to periodically
    // check the bump switches, stopping the robot on a collision
 
    Clock_Init48MHz();
        LaunchPad_Init();
        Reflectance_Init();
        Motor_Init();
        Spt = Center;
        while(1){
                Motor_Drive(Spt->out, Spt->d1, Spt->d2);     // do output to two motors
                Clock_Delay1ms(Spt->delay);   // wait
                Input = Reflectance_Read(1000);
                Inuput = (Input & 0x3C) >> 2;
                Spt = Spt->next[Input];       // next depends on input and state    }
  while(1){
    int x = Program13_1();
  }
}

        void (*BumpTask)(uint8_t);

        // Initialize Bump sensors
        // Make six Port 4 pins inputs
        // Activate interface pullup
        // pins 7,6,5,3,2,0
        // Interrupt on falling edge (on touch)
        void BumpInt_Init(void(*task)(uint8_t)){
            // write this as part of Lab 14
            P4->SEL0 &= ~0xED;
            P4->SEL1 &= ~0xED;    // configure sensors as GPIO
            P4->DIR &= ~0xED;     // make sensors in
            P4->REN |= 0xED;      // enable internal pull resistors for sensors
            P4->OUT |= 0xED;      // make sensors pull-up
            P4->IES |= 0xED;      // sensors are falling edge events
            P4->IFG &= ~0xED;     // initially clear flags
            P4->IE |= 0xED;       // arm interrupts
            NVIC->IP[9] = (NVIC->IP[9]&0xFF00FFFF)|0x00400000; // priority 2
            NVIC->ISER[1] = 0x00000040;        // enable interrupt 38 in NVIC
            BumpTask = task;
        }
        // Read current state of 6 switches
        // Returns a 6-bit positive logic result (0 to 63)
        // bit 5 Bump5
        // bit 4 Bump4
        // bit 3 Bump3
        // bit 2 Bump2
        // bit 1 Bump1
        // bit 0 Bump0
        uint8_t Bump_Read(void){
            // write this as part of Lab 14
            uint8_t result;
            result = (~P4->IN)&0x01;       // sets bit 0 with positive logic;
            result |= ((~P4->IN)&0x0C)>>1; // sets bits 1-2 with positive logic;
            result |= ((~P4->IN)&0xE0)>>2; // sets bits 1-2 with positive logic;

            return result;
        }
        // we do not care about critical section/race conditions
        // triggered on touch, falling edge
        void PORT4_IRQHandler(void){
            // write this as part of Lab 14
            uint8_t data;
            data = Bump_Read();
            (*BumpTask)(data);
        }


