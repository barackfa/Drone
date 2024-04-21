/*
 * ESC.c
 *
 *  Created on: 16 Oct 2023
 *      Author: Andris
 */

#include "main.h"

#define P_pitch 5
#define D_pitch 0
#define P_roll 5
#define D_roll 0
#define P_yaw 5
#define D_yaw 0



void set_duty_Oneshot42(TIM_HandleTypeDef* const pwmHandle1, uint16_t ref_1, uint16_t ref_2, uint16_t ref_3, uint16_t ref_4){
	// Multishot42 12 kHz PSC 7-1, ARR 1000-1 -> 1000 = 100%, 500 = stop, ?0 = -100%?
	pwmHandle1 -> Instance -> CCR1 = ref_1;
	pwmHandle1 -> Instance -> CCR2 = ref_2;
	pwmHandle1 -> Instance -> CCR3 = ref_3;
	pwmHandle1 -> Instance -> CCR4 = ref_4;
}

float CRSFtoDuty(uint16_t CRSF_val){
	float Duty;
//	Duty = 550+((float)(CRSF_val-172))/4.1; // minimum duty 55% max duty 95%
	Duty = 550+((float)(CRSF_val-172))/4.1*0.4; //for safety reasons minimum duty 55% max duty 71%
	return Duty;
}

float CRSFtoPitch(uint16_t CRSF_val){
	float pitch;
	pitch = ((float)(CRSF_val-992))/820.0;
	return pitch;
}

float CRSFtoRoll(uint16_t CRSF_val){
	float roll;
	roll = ((float)(CRSF_val-992))/820.0;
	return roll;
}

float CRSFtoYaw(uint16_t CRSF_val){
	float yaw;
	yaw = ((float)(CRSF_val-992))/820.0;
	return yaw;
}

void motor_speed_calc(uint16_t *ref1, uint16_t *ref2, uint16_t *ref3, uint16_t *ref4, float pitch, float roll, float throttle){
	uint16_t calc1, calc2, calc3, calc4;
	calc1 = (uint16_t)(throttle * (1 + pitch/5));
	calc2 = (uint16_t)(throttle * (1 + pitch/5));
	calc3 = (uint16_t)(throttle * (1 - pitch/5));
	calc4 = (uint16_t)(throttle * (1 - pitch/5));
	if(calc1<550) *ref1 = 550;
	else *ref1 = calc1;
	if(calc2<550) *ref2 = 550;
	else *ref2 = calc2;
	if(calc3<550) *ref3 = 550;
	else *ref3 = calc3;
	if(calc4<550) *ref4 = 550;
	else *ref4 = calc4;
}
