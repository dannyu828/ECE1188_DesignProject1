//*****************************************************************************
//
// Jacki FSM test main
// MSP432 with Jacki
// Daniel and Jonathan Valvano
// July 11, 2019
/* This example accompanies the book
   "Embedded Systems: Introduction to Robotics,
   Jonathan W. Valvano, ISBN: 9781074544300, copyRight (c) 2019
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
Simplified BSD License (FreeBSD License)
CopyRight (c) 2019, Jonathan Valvano, All Rights reserved.
Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
1. Redistributions of source code must retain the above copyRight notice,
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyRight notice,
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
#include "../inc/SysTickInt.h"
#include "../inc/UART0.h"
#include "../inc/BumpInt.h"
#include "../inc/Reflectance.h"
#include "../inc/Motor.h"
#include "../inc/SysTickInts.h"

uint8_t CollisionData, CollisionFlag;  // mailbox
void HandleCollision(uint8_t bumpSensor){
   Motor_Stop();
   CollisionData = bumpSensor;
   CollisionFlag = 1;
}

struct State {
  uint8_t color;                   // 2-bit output
  uint16_t left;
  uint16_t right;
  void (*func)(uint16_t,uint16_t); // calls output function 
  const struct State *next[10];    // Next staes
};
typedef const struct State State_t;

#define Straight   &fsm[0]
#define Left       &fsm[1]
#define HardLeft   &fsm[2]
#define Right      &fsm[3]
#define HardRight  &fsm[4]
#define Search     &fsm[5]
#define Stop       &fsm[6]

#define fullSpeed 10000
#define softSpeed (FSpeed - 100)
#define midSpeed (FSpeed - 500)
#define turnSpeed (FSpeed - 4000)
#define hardTurnSpeed (FSpeed - 2000)

#define DARK      0x00
#define RED       0x01
#define GREEN     0x02
#define YELLOW    0x03
#define BLUE      0x04
#define PINK      0x05
#define SKYBLUE   0x06
#define WHITE     0x07

void Motor_HardLeft(uint16_t dutyLeft, uint16_t dutyRight) {
  // Right forward, left backward
}

void Motor_HardRight(uint16_t dutyLeft, uint16_t dutyRight) {
  // Left forward, right backward
}

void Motor_Search(uint16_t dutyLeft, uint16_t dutyRight) {
  // Go left and then right to search for line
}

void Motor_Stop_Func(uint16_t dutyLeft, uint16_t dutyRight) {
  Motor_Stop();
}

State_t fsm[9]={
  {GREEN, 2000, 2000, &Motor_Forward, {Straight, Left, HardLeft, Right, HardRight, Search, Search, Search, Straight, Straight }}, // Straight
  {PINK, 2500, &Motor_Left, {Straight, Left, HardLeft, Right, Right, Search, Search, Search, Left, Left }},                    // Left
  {YELLOW, 0, 1500, &Motor_HardLeft, { Straight, Left, HardLeft, Right, Right, Search, Search, Search, HardLeft, HardLeft }},      // HardLeft
  {PINK, 4500, 0, &Motor_Right, { Straight, Left, Left, Right, HardRight, Search, Search, Search, Right, Right }},              // Right
  {YELLOW, 0, 4500, &Motor_HardRight { Straight, Left, Left, Right, HardRight, Search, Search, Search, HardRight, HardRight }},  // HardRight
  {WHITE, 1500, 1500, &Motor_Search, { Straight, Left, HardLeft, Right, HardRight, Search, HardLeft, HardRight, Search, Stop }}, // Search
  {RED, 0, 0, &Motor_Stop_Func, { Search, Search, Search, Search, Search, Search, Search, Search, Search, Stop }}                   // Stop
};

State_t *Spt;     // pointer to the current state
State_t *lastSpt; // pointer to previous state
uint8_t input;    // line sensor data
uint8_t time;     // time elapsed
uint8_t idx;      // state index
bool lost;        // lost flag

void inputToState() {
  uint8_t tempIdx;
  if (input == 0x18 || input == 0x3C)
    tempIdx = 0;
  else if (input == 0xF0 || input == 0xE0)
    tempIdx = 1;
  else if (input == 0xC0 || input == 0x80)
    tempIdx = 2;
  else if (input == 0x0F || input == 0x07)
    tempIdx = 3;
  else if (input == 0x03 || input == 0x01)
    tempIdx = 4;
  else if (input == 0x00 || input == 0xFF) {
    if (lost == 1)
      tempIdx = 9;
    else
      tempIdx = 5;
  }
  else if (input == 0x7F)
    tempIdx = 6;
  else if (input == 0xFE)
    tempIdx = 7;
  else
    tempIdx = 8;

  if (tempIdx == 9)
    lost = 0;

  return tempIdx;
}

void SysTick_Handler(void){ // every 1ms
    if (time % 5 == 0){
        Reflectance_Start();
    }
    else if (time % 5 == 1){
        input = Reflectance_End();
        idx = lineToState(input);
        Spt=Spt->next[idx];
        (*Spt->func)(Spt->left,Spt->right);
        //Clock_Delay1ms(Spt->delay)
    }
    time++;
}

void main(void){
    Clock_Init48MHz();
    LaunchPad_Init();
    Reflectance_Init();
    Motor_Init();
    CollisionFlag = time = idx = lost = 0;
    SysTick_Init(48000, 0);
    BumpInt_Init(&HandleCollision);
    Spt = Straight;
    EnableInterrupts();
    while(1){
        WaitForInterrupt();
    }
}
