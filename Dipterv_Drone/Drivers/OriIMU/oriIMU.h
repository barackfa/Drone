/*
 * oriIMU.h
 *
 *  Created on: Oct 25, 2023
 *      Author: Andris
 */

#ifndef ORIIMU_ORIIMU_H_
#define ORIIMU_ORIIMU_H_


void filterUpdateIMU(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, quaternion *q);
#endif /* ORIIMU_ORIIMU_H_ */
