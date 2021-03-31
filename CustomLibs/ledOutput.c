/*
 * ledOutput.c
 *
 *  Created on: Mar 8, 2021
 *      Author: Dmitriy Filin
 */
#include "ledOutput.h"

void delay2(int d){
	for(int z=0;z<d;z++){
		for(int t=0;t<d;t++){

		}
	}
}

void shortBlinkRed(void){
	TIM1->CCR2 = 0;
	delay2(100);
	TIM1->CCR2 = 2000;
	delay2(100);
	TIM1->CCR2 = 0;
}

void shortBlinkGreen(void){
	TIM1->CCR1 = 0;
	delay2(100);
	TIM1->CCR1 = 2000;
	delay2(100);
	TIM1->CCR1 = 0;
}
void shortBlinkBlue(void){
	TIM1->CCR3 = 0;
	delay2(100);
	TIM1->CCR3 = 2000;
	delay2(100);
	TIM1->CCR3 = 0;
}
void shortBlinkYellow(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	delay2(100);
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 3000;
	delay2(100);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
}
void shortBlinkPurple(void){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	delay2(100);
	TIM1->CCR3 = 2000;
	TIM1->CCR2 = 3000;
	delay2(100);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
}
void shortBlinkCayan(void){
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
	delay2(100);
	TIM1->CCR3 = 2000;
	TIM1->CCR1 = 2000;
	delay2(100);
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
}

void longBlinkRed(void){
	TIM1->CCR2 = 0;
	delay2(400);
	TIM1->CCR2 = 2000;
	delay2(400);
	TIM1->CCR2 = 0;
}

void longBlinkGreen(void){
	TIM1->CCR1 = 0;
	delay2(400);
	TIM1->CCR1 = 2000;
	delay2(400);
	TIM1->CCR1 = 0;
}
void longBlinkBlue(void){
	TIM1->CCR3 = 0;
	delay2(400);
	TIM1->CCR3 = 2000;
	delay2(400);
	TIM1->CCR3 = 0;
}
void longBlinkYellow(void){
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
	delay2(400);
	TIM1->CCR1 = 2000;
	TIM1->CCR2 = 3000;
	delay2(400);
	TIM1->CCR1 = 0;
	TIM1->CCR2 = 0;
}
void longBlinkPurple(void){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	delay2(400);
	TIM1->CCR3 = 2000;
	TIM1->CCR2 = 3000;
	delay2(400);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
}
void longBlinkCayan(void){
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
	delay2(400);
	TIM1->CCR3 = 2000;
	TIM1->CCR1 = 2000;
	delay2(400);
	TIM1->CCR3 = 0;
	TIM1->CCR1 = 0;
}

void customBlinkRGB(uint8_t r, uint8_t g, uint8_t b, int tim){
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
	delay2(tim);
	TIM1->CCR3 = 2000*b/255;
	TIM1->CCR2 = 3000*r/255;
	TIM1->CCR1 = 2000*g/255;
	delay2(tim);
	TIM1->CCR3 = 0;
	TIM1->CCR2 = 0;
	TIM1->CCR1 = 0;
}

