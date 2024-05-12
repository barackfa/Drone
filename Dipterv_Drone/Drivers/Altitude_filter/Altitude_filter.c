/*
 * Altitude_filter.c
 *
 *  Created on: 5 Nov 2023
 *      Author: Andris
 */

#include "Altitude_filter.h"
#include "main.h"


#define SAMPLE_PERIOD (0.005f)

#ifndef ALTITUDE_FILTER_ALTITUDE_3D_FILTER_H_

KF_Matrix33 A = {1, SAMPLE_PERIOD*SAMPLE_PERIOD/2, SAMPLE_PERIOD, 0,1,0,0,SAMPLE_PERIOD,1};
KF_Matrix23 C = {1,0,0,0,1,0};
KF_Matrix32 CT = {1,0,0,1,0,0};
KF_Matrix33 Q = {1	,0,0,0,1,0,0,0,0};
KF_Matrix22 R = {0.0484,0,0,0.00000000193007};
KF_Matrix33 eye = {1,0,0,0,1,0,0,0,1};

KF_Matrix33 KF_matrix_multiply33_33(KF_Matrix33 left, KF_Matrix33 right){
	KF_Matrix33 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a12=left.a11*right.a12 + left.a12*right.a22 + left.a13*right.a32;
	results.a13=left.a11*right.a13 + left.a12*right.a23 + left.a13*right.a33;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	results.a22=left.a21*right.a12 + left.a22*right.a22 + left.a23*right.a32;
	results.a23=left.a21*right.a13 + left.a22*right.a23 + left.a23*right.a33;
	results.a31=left.a31*right.a11 + left.a32*right.a21 + left.a33*right.a31;
	results.a32=left.a31*right.a12 + left.a32*right.a22 + left.a33*right.a32;
	results.a33=left.a31*right.a13 + left.a32*right.a23 + left.a33*right.a33;
	return results;
}

KF_Matrix23 KF_matrix_multiply23_33(KF_Matrix23 left, KF_Matrix33 right){
	KF_Matrix23 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a12=left.a11*right.a12 + left.a12*right.a22 + left.a13*right.a32;
	results.a13=left.a11*right.a13 + left.a12*right.a23 + left.a13*right.a33;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	results.a22=left.a21*right.a12 + left.a22*right.a22 + left.a23*right.a32;
	results.a23=left.a21*right.a13 + left.a22*right.a23 + left.a23*right.a33;
	return results;
}

KF_Matrix22 KF_matrix_multiply23_32(KF_Matrix23 left, KF_Matrix32 right){
	KF_Matrix22 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a12=left.a11*right.a12 + left.a12*right.a22 + left.a13*right.a32;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	results.a22=left.a21*right.a12 + left.a22*right.a22 + left.a23*right.a32;
	return results;
}

KF_Matrix32 KF_matrix_multiply33_32(KF_Matrix33 left, KF_Matrix32 right){
	KF_Matrix32 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a12=left.a11*right.a12 + left.a12*right.a22 + left.a13*right.a32;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	results.a22=left.a21*right.a12 + left.a22*right.a22 + left.a23*right.a32;
	results.a31=left.a31*right.a11 + left.a32*right.a21 + left.a33*right.a31;
	results.a32=left.a31*right.a12 + left.a32*right.a22 + left.a33*right.a32;
	return results;
}

KF_Matrix21 KF_matrix_multiply23_31(KF_Matrix23 left, KF_Matrix31 right){
	KF_Matrix21 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	return results;
}

KF_Matrix32 KF_matrix_multiply32_22(KF_Matrix32 left, KF_Matrix22 right){
	KF_Matrix32 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21;
	results.a12=left.a11*right.a12 + left.a12*right.a22;
	results.a21=left.a21*right.a11 + left.a22*right.a21;
	results.a22=left.a21*right.a12 + left.a22*right.a22;
	results.a31=left.a31*right.a11 + left.a32*right.a21;
	results.a32=left.a31*right.a12 + left.a32*right.a22;
	return results;
}

KF_Matrix33 KF_matrix_multiply32_23(KF_Matrix32 left, KF_Matrix23 right){
	KF_Matrix33 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21;
	results.a12=left.a11*right.a12 + left.a12*right.a22;
	results.a13=left.a11*right.a13 + left.a12*right.a23;
	results.a21=left.a21*right.a11 + left.a22*right.a21;
	results.a22=left.a21*right.a12 + left.a22*right.a22;
	results.a23=left.a21*right.a13 + left.a22*right.a23;
	results.a31=left.a31*right.a11 + left.a32*right.a21;
	results.a32=left.a31*right.a12 + left.a32*right.a22;
	results.a33=left.a31*right.a13 + left.a32*right.a23;
	return results;
}

KF_Matrix31 KF_matrix_multiply32_21(KF_Matrix32 left, KF_Matrix21 right){
	KF_Matrix31 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21;
	results.a21=left.a21*right.a11 + left.a22*right.a21;
	results.a31=left.a31*right.a11 + left.a32*right.a21;
	return results;
}

KF_Matrix31 KF_matrix_multiply33_31(KF_Matrix33 left, KF_Matrix31 right){
	KF_Matrix31 results;
	results.a11=left.a11*right.a11 + left.a12*right.a21 + left.a13*right.a31;
	results.a21=left.a21*right.a11 + left.a22*right.a21 + left.a23*right.a31;
	results.a31=left.a31*right.a11 + left.a32*right.a21 + left.a33*right.a31;
	return results;
}

KF_Matrix33 KF_matrix_transpose33(KF_Matrix33 matrix){
	KF_Matrix33 results;
	results.a11=matrix.a11;
	results.a12=matrix.a21;
	results.a13=matrix.a31;
	results.a21=matrix.a12;
	results.a22=matrix.a22;
	results.a23=matrix.a32;
	results.a31=matrix.a13;
	results.a32=matrix.a23;
	results.a33=matrix.a33;
	return results;
}

KF_Matrix32 KF_matrix_transpose23(KF_Matrix23 matrix){
	KF_Matrix32 results;
	results.a11=matrix.a11;
	results.a12=matrix.a21;
	results.a21=matrix.a12;
	results.a22=matrix.a22;
	results.a31=matrix.a13;
	results.a32=matrix.a23;
	return results;
}

KF_Matrix33 KF_matrix_add33(KF_Matrix33 mx1, KF_Matrix33 mx2){
	KF_Matrix33 results;
	results.a11=mx1.a11+mx2.a11;
	results.a12=mx1.a12+mx2.a12;
	results.a13=mx1.a13+mx2.a13;
	results.a21=mx1.a21+mx2.a21;
	results.a22=mx1.a22+mx2.a22;
	results.a23=mx1.a23+mx2.a23;
	results.a31=mx1.a31+mx2.a31;
	results.a32=mx1.a32+mx2.a32;
	results.a33=mx1.a33+mx2.a33;
	return results;
}

KF_Matrix22 KF_matrix_add22(KF_Matrix22 mx1, KF_Matrix22 mx2){
	KF_Matrix22 results;
	results.a11=mx1.a11+mx2.a11;
	results.a12=mx1.a12+mx2.a12;
	results.a21=mx1.a21+mx2.a21;
	results.a22=mx1.a22+mx2.a22;
	return results;
}

KF_Matrix31 KF_matrix_add31(KF_Matrix31 mx1, KF_Matrix31 mx2){
	KF_Matrix31 results;
	results.a11=mx1.a11+mx2.a11;
	results.a21=mx1.a21+mx2.a21;
	results.a31=mx1.a31+mx2.a31;
	return results;
}

KF_Matrix21 KF_matrix_sub21(KF_Matrix21 mx1, KF_Matrix21 mx2){
	KF_Matrix21 result;
	result.a11=mx1.a11-mx2.a11;
	result.a21=mx1.a21-mx2.a21;
	return result;
}

KF_Matrix33 KF_matrix_sub33(KF_Matrix33 mx1, KF_Matrix33 mx2){
	KF_Matrix33 result;
	result.a11=mx1.a11-mx2.a11;
	result.a12=mx1.a12-mx2.a12;
	result.a13=mx1.a13-mx2.a13;
	result.a21=mx1.a21-mx2.a21;
	result.a22=mx1.a22-mx2.a22;
	result.a23=mx1.a23-mx2.a23;
	result.a31=mx1.a31-mx2.a31;
	result.a32=mx1.a32-mx2.a32;
	result.a33=mx1.a33-mx2.a33;
	return result;
}

KF_Matrix22 KF_matrix_inverse22(KF_Matrix22 mx){
	KF_Matrix22 result;
	float sum=0;
	sum = mx.a11*mx.a22 - mx.a12*mx.a21;
	result.a11 = mx.a22/sum;
	result.a22 = mx.a11/sum;
	result.a12 = -mx.a12/sum;
	result.a21 = -mx.a21/sum;
	return result;
}


void altitudeKF(KF_Matrix31 prev_state, KF_Matrix31 *current_state, KF_Matrix33 P_prev, KF_Matrix33 *P, KF_Matrix21 meas){
	KF_Matrix31 xhatkk;
	KF_Matrix31 xhatkk_1;
	KF_Matrix33 Pkk_1;
	KF_Matrix33 Pcalc;
	KF_Matrix32 Kk;
	KF_Matrix22 forinv;
	KF_Matrix22 inv;
	KF_Matrix21 xhatcor;


	// prediction
	xhatkk_1 = KF_matrix_multiply33_31(A, prev_state);
	Pkk_1 = KF_matrix_add33(KF_matrix_multiply33_33(KF_matrix_multiply33_33( A,  P_prev), KF_matrix_transpose33(A)),Q);

	//correction
	Kk = KF_matrix_multiply33_32(Pkk_1, CT);
	forinv = KF_matrix_add22(KF_matrix_multiply23_32(KF_matrix_multiply23_33(C, Pkk_1),CT),R);
	inv = KF_matrix_inverse22(forinv);
	Kk = KF_matrix_multiply32_22(Kk, inv);
	xhatcor = KF_matrix_sub21(meas, KF_matrix_multiply23_31(C, xhatkk_1));
	xhatkk = KF_matrix_multiply32_21(Kk, xhatcor);
	xhatkk = KF_matrix_add31(xhatkk_1, xhatkk);
	*current_state = xhatkk;
	Pcalc = KF_matrix_sub33(eye, KF_matrix_multiply32_23(Kk, C));
	*P = KF_matrix_multiply33_33(Pcalc, Pkk_1);
}

#endif /* ALTITUDE_FILTER_ALTITUDE_3D_FILTER_H_ */



#ifndef ALTITUDE_FILTER_ALTITUDE_2D_FILTER_H_

KF_Matrix22 F = {1, SAMPLE_PERIOD, 0, 1};
KF_Matrix22 FT = {1, 0, SAMPLE_PERIOD, 1};
KF_Matrix21 G = {0.5*SAMPLE_PERIOD,SAMPLE_PERIOD};
KF_Matrix12 H = {1,0};
KF_Matrix21 HT = {1,0};
KF_Matrix22 Q = {0.25*SAMPLE_PERIOD*SAMPLE_PERIOD*SAMPLE_PERIOD*SAMPLE_PERIOD,0.5*SAMPLE_PERIOD*SAMPLE_PERIOD*SAMPLE_PERIOD,0.5*SAMPLE_PERIOD*SAMPLE_PERIOD*SAMPLE_PERIOD,SAMPLE_PERIOD*SAMPLE_PERIOD};
float R = 0.3*0.3;
float Lk = 0;
KF_Matrix22 eye = {1,0,0,1};

void altitudeKF(KF_Matrix21 prev_state, KF_Matrix21 *current_state, KF_Matrix22 P_prev, KF_Matrix22 *P, KF_Matrix21 meas){
	KF_Matrix21 Sk;
	KF_Matrix22 Pk;
	KF_Matrix22 Pcalc;
	KF_Matrix21 Kk;
	// meas.a21 accz
	// meas.a11 h


	// prediction
	Sk = KF_matrix_add21(KF_matrix_multiply22_21(F, prev_state), KF_matrix_scalar_multi_21(G, meas.a21));
	Pk = KF_matrix_add22(KF_matrix_multiply22_22(KF_matrix_multiply22_22(F, P_prev),FT),KF_matrix_scalar_multi_22(Q,0.01));

	//correction
	Lk = Pk.a11 + R;
	Kk = KF_matrix_scalar_multi_21(KF_matrix_multiply22_21(Pk, HT),1/Lk);

	Sk = KF_matrix_add21(Sk,KF_matrix_scalar_multi_21(Kk,(meas.a11-Sk.a11)));
	*current_state = Sk;
	Pcalc = KF_matrix_sub22(eye, KF_matrix_multiply21_12(Kk, H));
	*P = KF_matrix_multiply22_22(Pcalc, Pk);
}


//matrix multiply by matrix
KF_Matrix22 KF_matrix_multiply22_22(KF_Matrix22 left, KF_Matrix22 right){
    KF_Matrix22 results;
    results.a11 = left.a11 * right.a11 + left.a12 * right.a21;
    results.a12 = left.a11 * right.a12 + left.a12 * right.a22;
    results.a21 = left.a21 * right.a11 + left.a22 * right.a21;
    results.a22 = left.a21 * right.a12 + left.a22 * right.a22;
    return results;
}

KF_Matrix21 KF_matrix_multiply22_21(KF_Matrix22 left, KF_Matrix21 right) {
    KF_Matrix21 result;
    result.a11 = left.a11 * right.a11 + left.a12 * right.a21;
    result.a21 = left.a21 * right.a11 + left.a22 * right.a21;
    return result;
}

KF_Matrix12 KF_matrix_multiply12_22(KF_Matrix12 left, KF_Matrix22 right) {
    KF_Matrix12 result;
    result.a11 = left.a11 * right.a11 + left.a12 * right.a21;
    result.a12 = left.a11 * right.a12 + left.a12 * right.a22;
    return result;
}

float KF_matrix_multiply12_21(KF_Matrix12 left, KF_Matrix21 right){
    float result;
    result = left.a11 * right.a11 + left.a12 * right.a21;
    return result;
}

KF_Matrix22 KF_matrix_multiply21_12(KF_Matrix21 left, KF_Matrix12 right) {
    KF_Matrix22 result;
    result.a11 = left.a11 * right.a11;
    result.a12 = left.a21 * right.a11;
    result.a21 = left.a11 * right.a12;
    result.a22 = left.a21 * right.a12;
    return result;
}


//transpose
KF_Matrix22 KF_matrix_transpose22(KF_Matrix22 matrix){
	KF_Matrix22 results;
	results.a11 = matrix.a11;
	results.a12 = matrix.a21;
	results.a21 = matrix.a12;
	results.a22 = matrix.a22;
	return results;
}

KF_Matrix21 KF_matrix_transpose12(KF_Matrix12 matrix){
    KF_Matrix21 results;
    results.a11 = matrix.a11;
    results.a21 = matrix.a12;
    return results;
}

KF_Matrix12 KF_matrix_transpose21(KF_Matrix21 matrix){
    KF_Matrix12 results;
    results.a11 = matrix.a11;
    results.a12 = matrix.a21;
    return results;
}

//mx add
KF_Matrix22 KF_matrix_add22(KF_Matrix22 mx1, KF_Matrix22 mx2){
    KF_Matrix22 results;
    results.a11 = mx1.a11 + mx2.a11;
    results.a12 = mx1.a12 + mx2.a12;
    results.a21 = mx1.a21 + mx2.a21;
    results.a22 = mx1.a22 + mx2.a22;
    return results;
}

KF_Matrix21 KF_matrix_add21(KF_Matrix21 mx1, KF_Matrix21 mx2){
    KF_Matrix21 results;
    results.a11 = mx1.a11 + mx2.a11;
    results.a21 = mx1.a21 + mx2.a21;
    return results;
}

//mx sub
KF_Matrix22 KF_matrix_sub22(KF_Matrix22 mx1, KF_Matrix22 mx2){
    KF_Matrix22 results;
    results.a11 = mx1.a11 - mx2.a11;
    results.a12 = mx1.a12 - mx2.a12;
    results.a21 = mx1.a21 - mx2.a21;
    results.a22 = mx1.a22 - mx2.a22;
    return results;
}

KF_Matrix21 KF_matrix_sub21(KF_Matrix21 mx1, KF_Matrix21 mx2){
    KF_Matrix21 results;
    results.a11 = mx1.a11 - mx2.a11;
    results.a21 = mx1.a21 - mx2.a21;
    return results;
}

//mx multiply by scalar
KF_Matrix22 KF_matrix_scalar_multi_22(KF_Matrix22 mx, float scalar){
    KF_Matrix22 results;
    results.a11 = mx.a11 * scalar;
    results.a12 = mx.a12 * scalar;
    results.a21 = mx.a21 * scalar;
    results.a22 = mx.a22 * scalar;
    return results;
}

KF_Matrix21 KF_matrix_scalar_multi_21(KF_Matrix21 mx, float scalar){
    KF_Matrix21 results;
    results.a11 = mx.a11 * scalar;
    results.a21 = mx.a21 * scalar;
    return results;
}



#endif /* ALTITUDE_FILTER_ALTITUDE_3D_FILTER_H_ */
