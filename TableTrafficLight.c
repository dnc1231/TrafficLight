// TableTrafficLight.c solution to edX lab 10, EE319KLab 5
// Runs on LM4F120 or TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  
// Daniel Valvano, Jonathan Valvano
// November 7, 2013

/* solution, do not post

 Copyright 2014 by Jonathan W. Valvano, valvano@mail.utexas.edu
    You may use, edit, run or distribute this file
    as long as the above copyright notice remains
 THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 VALVANO SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL,
 OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.
 For more information about my classes, my research, and my books, see
 http://users.ece.utexas.edu/~valvano/
 */

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)
#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "SysTick.h"
#include "TExaS.h"

#define RoadLIGHT                   (*((volatile unsigned long *)0x400043F0)) 	//port A bits 0-5
#define SENSOR                  		(*((volatile unsigned long *)0x4002401C)) 	//port E bits 0-2
#define PedLIGHT										(*((volatile unsigned long *)0x40025028))		//port F bits 1 & 3
// Declare your FSM linked structure here

void DisableInterrupts(void);
void EnableInterrupts(void);  // Enable interrupts

struct State {
	unsigned long RoadOutput;
	unsigned long PedOutput;
	unsigned long Time;
  unsigned long Next[8];}; 

typedef const struct State STyp;
#define goS          0
#define waitS        1
#define waitSP       2
#define goW          3
#define waitW        4
#define waitWP       5
#define PedWalk      6
#define PedHurryOn1  7
#define PedHurryOff1 8
#define PedHurryOn2  9
#define PedHurryOff2 10
#define RoadClosed   11
STyp FSM[12]={
//{SignalLEDs, WalkLED, Delay, NEXT}/////
 {0x84,0x02,150,{goS,waitS,goS,goS,waitSP,waitSP,waitSP,waitS}},                                                          //goS 
 {0x88,0x02,150,{RoadClosed,goW,goS,goS,PedWalk,PedWalk,PedWalk,goW}},                                           					//waitS
 {0x88,0x02,150,{PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk}},                                        //waitSP
 {0x30,0x02,150,{goW,goW,waitW,goW,waitWP,waitWP,waitWP,waitW}},                                                            //goW
 {0x50,0x02,150,{RoadClosed,RoadClosed,goS,goS,PedWalk,PedWalk,PedWalk,PedWalk}},                                           //waitW
 {0x50,0x02,150,{PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk,PedWalk}},                                         //waitWP
 {0x90,0x08,200,{PedHurryOn1,PedHurryOn1,PedHurryOn1,PedHurryOn1,PedWalk,PedWalk,PedWalk,PedHurryOn1}},         //PedWalk
 {0x90,0x02,100,{PedHurryOff1,PedHurryOff1,PedHurryOff1,PedHurryOff1,PedHurryOff1,PedHurryOff1,PedHurryOff1,PedHurryOff1}}, //PedHurryOn1
 {0x90,0x00,100,{PedHurryOn2,PedHurryOn2,PedHurryOn2,PedHurryOn2,PedHurryOn2,PedHurryOn2,PedHurryOn2,PedHurryOn2}},         //PedHurryOff1
 {0x90,0x02,100,{PedHurryOff2,PedHurryOff2,PedHurryOff2,PedHurryOff2,PedHurryOff2,PedHurryOff2,PedHurryOff2,PedHurryOff2}}, //PedHurryOn2
 {0x90,0x00,100,{RoadClosed,RoadClosed,RoadClosed,RoadClosed,RoadClosed,RoadClosed,RoadClosed,goS}},                 //PedHurryOff2
 {0x90,0x02,50,{RoadClosed,goW,goS,goS,PedWalk,PedWalk,PedWalk,goS}}                                                   //RoadClosed
};

unsigned long current;  // index to the current state 
unsigned long delay;
unsigned long Input;

int main(void){ volatile unsigned long delay;
  TExaS_Init(SW_PIN_PE210, LED_PIN_PB543210); // activate traffic simulation and set system clock to 80 MHz
  
	SYSCTL_RCGC2_R |= 0x31;      // 1) A  E F
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
	
	//input bits
	GPIO_PORTE_AMSEL_R &= ~0x07; // 3) disable analog function on PE2-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x00;   // 5) inputs on PE2-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE2-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE2-0
	
  //crosslight output bits
	GPIO_PORTA_AMSEL_R &= ~0xFC; // 3) disable analog function on PA2-7
  GPIO_PORTA_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTA_DIR_R |= 0xFC;    // 5) outputs on PA2-7
  GPIO_PORTA_AFSEL_R &= ~0xFC; // 6) regular function on PA2-7
  GPIO_PORTA_DEN_R |= 0xFC;    // 7) enable digital on PA2-5
	
	//pedlight output bits
	GPIO_PORTF_AMSEL_R &= ~0x0A; // 3) disable analog function on PF1 and PF3
  GPIO_PORTF_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTF_DIR_R |= 0x0A;    // 5) outputs on PF1 and PF3
  GPIO_PORTF_AFSEL_R &= ~0x0A; // 6) regular function on PF1 and PF3
  GPIO_PORTF_DEN_R |= 0x0A;    // 7) enable digital on PF1 and PF3
	
	SysTick_Init();     
  EnableInterrupts();
	
	
	current = RoadClosed;
	
	
  //FSM Engine
  while(1){
		RoadLIGHT = FSM[current].RoadOutput;  // set S&W lights
		PedLIGHT = FSM[current].PedOutput;		// set ped cross light
    SysTick_Wait10ms(FSM[current].Time);	// delay
    Input = SENSOR;    									 // read sensors
    current = FSM[current].Next[Input];  	// go to next state
  }
}

