/*
 * Orifilter.c
 *
 *  Created on: 24 Oct 2023
 *      Author: Andris
 */

#include "main.h"
#include "math.h"
#include "Orifilter.h"


// System constants
#define deltat 0.005f // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift // compute zeta
#define PI 3.14159265358979f


// Global system variables
//float a_x, a_y, a_z; // accelerometer measurements
//float w_x, w_y, w_z; // gyroscope measurements in rad/s
//float m_x, m_y, m_z; // magnetometer measurements
//float q->SEq_1 = 1, q->SEq_2 = 0, q->SEq_3 = 0, q->SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
//float f->b_x = 1, f->b_z = 0; // reference direction of flux in earth frame
//float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error


// Function to compute one filter iteration


void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z, quaternion *q, flux *f, w_err *w)
{
	// local system variables
	float norm; // vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4; // quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6; // objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33, // objective function Jacobian elements
	J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4; // estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z; // estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z; // computed flux in the earth frame
	//float w_bx = 0, w_by = 0, w_bz = 0; // estimate gyroscope biases error

	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * q->SEq_1;
	float halfSEq_2 = 0.5f * q->SEq_2;
	float halfSEq_3 = 0.5f * q->SEq_3;
	float halfSEq_4 = 0.5f * q->SEq_4;
	float twoSEq_1 = 2.0f * q->SEq_1;
	float twoSEq_2 = 2.0f * q->SEq_2;
	float twoSEq_3 = 2.0f * q->SEq_3;
	float twoSEq_4 = 2.0f * q->SEq_4;
	float twob_x = 2.0f * f->b_x;
	float twob_z = 2.0f * f->b_z;
	float twob_xSEq_1 = 2.0f * f->b_x * q->SEq_1;
	float twob_xSEq_2 = 2.0f * f->b_x * q->SEq_2;
	float twob_xSEq_3 = 2.0f * f->b_x * q->SEq_3;
	float twob_xSEq_4 = 2.0f * f->b_x * q->SEq_4;
	float twob_zSEq_1 = 2.0f * f->b_z * q->SEq_1;
	float twob_zSEq_2 = 2.0f * f->b_z * q->SEq_2;
	float twob_zSEq_3 = 2.0f * f->b_z * q->SEq_3;
	float twob_zSEq_4 = 2.0f * f->b_z * q->SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = q->SEq_1 * q->SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = q->SEq_2 * q->SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;

	// normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;

	// normalise the magnetometer measurement
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;

	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * q->SEq_4 - twoSEq_1 * q->SEq_3 - a_x; //checked
	f_2 = twoSEq_1 * q->SEq_2 + twoSEq_3 * q->SEq_4 - a_y; //checked
	f_3 = 1.0f - twoSEq_2 * q->SEq_2 - twoSEq_3 * q->SEq_3 - a_z; //checked

	f_4 = twob_x * (0.5f - q->SEq_3 * q->SEq_3 - q->SEq_4 * q->SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x; //checked
	f_5 = twob_x * (q->SEq_2 * q->SEq_3 - q->SEq_1 * q->SEq_4) + twob_z * (q->SEq_1 * q->SEq_2 + q->SEq_3 * q->SEq_4) - m_y; //checked
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - q->SEq_2 * q->SEq_2 - q->SEq_3 * q->SEq_3) - m_z; //checked

	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication //checked
	J_12or23 = 2.0f * q->SEq_4; //checked
	J_13or22 = twoSEq_1; // J_13 negated in matrix multiplication //checked
	J_14or21 = twoSEq_2; //checked
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication //checked
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication //checked

	J_41 = twob_zSEq_3; // negated in matrix multiplication //checked
	J_42 = twob_zSEq_4; //checked
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication //checked
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication //checked
	J_51 = twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication //checked
	J_52 = twob_xSEq_3 + twob_zSEq_1; //checked
	J_53 = twob_xSEq_2 + twob_zSEq_4; //checked
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication //checked
	J_61 = twob_xSEq_3; //checked
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2; //checked
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3; //checked
	J_64 = twob_xSEq_2; //checked

	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6; //checked
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6; //checked
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6; //checked
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6; //checked

	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;

	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;



	// compute and remove the gyroscope baises //checked
	w->w_bx += w_err_x * deltat * zeta;
	w->w_by += w_err_y * deltat * zeta;
	w->w_bz += w_err_z * deltat * zeta;
	w_x -= w->w_bx;
	w_y -= w->w_by;
	w_z -= w->w_bz;

	// compute the quaternion rate measured by gyroscopes //checked
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;

	// compute then integrate the estimated quaternion rate
	q->SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	q->SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	q->SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	q->SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;

	// normalise quaternion
	norm = sqrt(q->SEq_1 * q->SEq_1 + q->SEq_2 * q->SEq_2 + q->SEq_3 * q->SEq_3 + q->SEq_4 * q->SEq_4);
	q->SEq_1 /= norm;
	q->SEq_2 /= norm;
	q->SEq_3 /= norm;
	q->SEq_4 /= norm;

	// compute flux in the earth frame
	SEq_1SEq_2 = q->SEq_1 * q->SEq_2; // recompute axulirary variables
	SEq_1SEq_3 = q->SEq_1 * q->SEq_3;
	SEq_1SEq_4 = q->SEq_1 * q->SEq_4;
	SEq_3SEq_4 = q->SEq_3 * q->SEq_4;
	SEq_2SEq_3 = q->SEq_2 * q->SEq_3;
	SEq_2SEq_4 = q->SEq_2 * q->SEq_4;
	h_x = twom_x * (0.5f*q->SEq_1 * q->SEq_1 - q->SEq_3 * q->SEq_3 - q->SEq_4 * q->SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3); // lemaradt mx*q1^2 valszeg ennek van a 0.5f
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f*q->SEq_1 * q->SEq_1 - q->SEq_2 * q->SEq_2 - q->SEq_4 * q->SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2); // lemaradt my*q1^2 valszeg ennek van a 0.5f
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f*q->SEq_1 * q->SEq_1 - q->SEq_2 * q->SEq_2 - q->SEq_3 * q->SEq_3); // lemaradt mz*q1^2 valszeg ennek van a 0.5f

	// normalise the flux vector to have only components in the x and z
	f->b_x = sqrt((h_x * h_x) + (h_y * h_y));
	f->b_z = h_z;
}


void eulerAngles(quaternion q, float* roll, float* pitch, float* yaw){
    *yaw = atan2((2*q.SEq_2*q.SEq_3 - 2*q.SEq_1*q.SEq_4), (2*q.SEq_1*q.SEq_1 + 2*q.SEq_2*q.SEq_2 -1));  // equation (7)
    *pitch = -asin(2*q.SEq_2*q.SEq_4 + 2*q.SEq_1*q.SEq_3);                                  // equatino (8)
    *roll  = atan2((2*q.SEq_3*q.SEq_4 - 2*q.SEq_1*q.SEq_2), (2*q.SEq_1*q.SEq_1 + 2*q.SEq_4*q.SEq_4 -1));

    *yaw *= (180.0f / PI);
    *pitch *= (180.0f / PI);
    *roll *= (180.0f / PI);

}

