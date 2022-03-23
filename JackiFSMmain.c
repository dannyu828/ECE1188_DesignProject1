//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
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

#include <stdint.h>
#include "msp.h"
#include "../inc/Clock.h"
#include "../inc/CortexM.h"
#include "../inc/PWM.h"
#include "../inc/LaunchPad.h"
#include "../inc/TExaS.h"
#include "../inc/AP.h"
#include "../inc/UART0.h"
#include "../inc/Bump.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"

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


void main(void){
    Clock_Init48MHz();
    LaunchPad_Init();
    Reflectance_Init();
    Motor_Init();
    Spt = Center;
    while(1){
            Motor_Drive(Spt->out, Spt->d1, Spt->d2);     // do output to two motors
            Clock_Delay1ms(Spt->delay);   // wait
            Input = Reflectance_Read(1000);
            Spt = Spt->next[Input];       // next depends on input and state    }

}
}
