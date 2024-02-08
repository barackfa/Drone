/*
 * Altitude_filter.h
 *
 *  Created on: 5 Nov 2023
 *      Author: Andris
 */

#ifndef ALTITUDE_FILTER_ALTITUDE_FILTER_H_
#define ALTITUDE_FILTER_ALTITUDE_FILTER_H_

#include "main.h"

typedef struct{
	float a11;
	float a21;
	float a31;
} KF_Matrix31;

typedef struct{
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
	float a31;
	float a32;
	float a33;
}KF_Matrix33;

typedef struct{
	float a11;
	float a12;
	float a13;
	float a21;
	float a22;
	float a23;
}KF_Matrix23;

typedef struct{
	float a11;
	float a12;
	float a21;
	float a22;
	float a31;
	float a32;
}KF_Matrix32;

typedef struct{
	float a11;
	float a12;
	float a21;
	float a22;
}KF_Matrix22;
typedef struct{
	float a11;
	float a21;
}KF_Matrix21;

void altitudeKF(KF_Matrix31 prev_state, KF_Matrix31 *current_state, KF_Matrix33 P_prev, KF_Matrix33 *P, KF_Matrix21 meas);

KF_Matrix33 KF_matrix_multiply33_33(KF_Matrix33 left, KF_Matrix33 right);

KF_Matrix23 KF_matrix_multiply23_33(KF_Matrix23 left, KF_Matrix33 right);

KF_Matrix22 KF_matrix_multiply23_32(KF_Matrix23 left, KF_Matrix32 right);

KF_Matrix21 KF_matrix_multiply23_31(KF_Matrix23 left, KF_Matrix31 right);

KF_Matrix32 KF_matrix_multiply33_32(KF_Matrix33 left, KF_Matrix32 right);

KF_Matrix32 KF_matrix_multiply32_22(KF_Matrix32 left, KF_Matrix22 right);

KF_Matrix33 KF_matrix_multiply32_23(KF_Matrix32 left, KF_Matrix23 right);

KF_Matrix31 KF_matrix_multiply32_21(KF_Matrix32 left, KF_Matrix21 right);

KF_Matrix31 KF_matrix_multiply33_31(KF_Matrix33 left, KF_Matrix31 right);

KF_Matrix33 KF_matrix_transpose33(KF_Matrix33 matrix);

KF_Matrix32 KF_matrix_transpose23(KF_Matrix23 matrix);

KF_Matrix33 KF_matrix_add33(KF_Matrix33 mx1, KF_Matrix33 mx2);

KF_Matrix22 KF_matrix_add22(KF_Matrix22 mx1, KF_Matrix22 mx2);

KF_Matrix31 KF_matrix_add31(KF_Matrix31 mx1, KF_Matrix31 mx2);

KF_Matrix21 KF_matrix_sub21(KF_Matrix21 mx1, KF_Matrix21 mx2);

KF_Matrix33 KF_matrix_sub33(KF_Matrix33 mx1, KF_Matrix33 mx2);

KF_Matrix22 KF_matrix_inverse22(KF_Matrix22 mx);


#endif /* ALTITUDE_FILTER_ALTITUDE_FILTER_H_ */
