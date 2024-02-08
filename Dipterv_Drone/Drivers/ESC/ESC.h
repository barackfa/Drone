/*
 * ESC.h
 *
 *  Created on: 16 Oct 2023
 *      Author: Andris
 */

#ifndef ESC_ESC_H_
#define ESC_ESC_H_

void set_duty_Oneshot42(TIM_HandleTypeDef* const pwmHandle1, uint16_t ref_1, uint16_t ref_2, uint16_t ref_3, uint16_t ref_4);

float CRSFtoDuty(uint16_t CRSF_val);

float CRSFtoPitch(uint16_t CRSF_val);

float CRSFtoRoll(uint16_t CRSF_val);

float CRSFtoYaw(uint16_t CRSF_val);

void motor_speed_calc(float *ref1, float *ref2, float *ref3, float *ref4, float pitch, float roll, float throttle);


#endif /* ESC_ESC_H_ */
