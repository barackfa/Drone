/*
 * Orifilter.h
 *
 *  Created on: 24 Oct 2023
 *      Author: Andris
 */

#ifndef ORIENTATIONFILTER_ORIFILTER_H_
#define ORIENTATIONFILTER_ORIFILTER_H_



typedef struct{
	float SEq_1;
	float SEq_2;
	float SEq_3;
	float SEq_4;
} quaternion;

typedef struct{
	float b_x;
	float b_z;
} flux;

typedef struct{
	float w_bx;
	float w_by;
	float w_bz;
} w_err;

void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, quaternion *q, flux *f, w_err *w);

void eulerAngles(quaternion q, float* roll, float* pitch, float* yaw);
/*typedef struct{
	float w_bx;
	float w_by;
	float w_bz;
} gyro_err;*/

#endif /* ORIENTATIONFILTER_ORIFILTER_H_ */
