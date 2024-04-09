/* Includes ------------------------------------------------------------------*/
#include "BMM150.h"
#include "math.h"

/* ----- PRIVATE FUNCTIONS PROTOTYPES ----- */
HAL_StatusTypeDef	BMM150_SoftReset(BMM150 *bmm);
HAL_StatusTypeDef	BMM150_ReadByte(BMM150 *bmm, uint8_t regAddr, uint8_t *data);
HAL_StatusTypeDef	BMM150_WriteByte(BMM150 *bmm, uint8_t regAddr, uint8_t data);
HAL_StatusTypeDef	BMM150_ReadMultiBytes(BMM150 *bmm, uint8_t regAddr, uint8_t *data, uint8_t len);

HAL_StatusTypeDef    BMM150_Init(BMM150 *bmm){
	HAL_StatusTypeDef rslt;
	uint8_t chip_id;
	uint8_t pwr_ctrl_on = 0b0000001;
	uint8_t opmode = 0x00;
	uint8_t debug;
	uint8_t debug_array[4];
//	uint16_t field_x;
//	uint16_t field_y;
	int16_t field_zp;
	int16_t field_zn;
	uint16_t Rhall;
	float magz_p;
	float magz_n;
	BMM150_trim_data trim_debug;

	//bring up from suspend mode to sleep mode
	HAL_I2C_Mem_Write(bmm->hi2c_handle, BMM150_ADDR << 1, BMM_PWR_CTRL_ADDR, I2C_MEMADD_SIZE_8BIT, &pwr_ctrl_on, 1, 100);
	HAL_Delay(10);
	// Read CHIP_ID byte
	rslt = BMM150_ReadByte(bmm, BMM150_CHIP_ID_ADDR, &chip_id);
	if (chip_id != 0x32) {

		//	return 0;

		}
	HAL_Delay(10);
	BMM150_Get_TrimData(bmm, &trim_debug);
	HAL_Delay(10);
	/*
//	0x4C reg bit <1:2> advanced self test, bit <3:5> data rate, bit <6:7> opmode, bit <8> normal self test
//	1. set sleep mode, opmode 11 is sleep
	BMM150_Set_OpMode(bmm, 0x06);
//	disable x,y axis, 0x4E
	rslt = BMM150_ReadByte(bmm, 0x4E, &debug);
	debug = debug | 0x18;
	HAL_Delay(1);
	rslt = BMM150_WriteByte(bmm, 0x4E, debug);
	HAL_Delay(1);
//	set Z repetitions
	rslt = BMM150_WriteByte(bmm, 0x52, 0x0F);
	HAL_Delay(1);
//	enable positive advanced self test current, 0x4C reg 11 <1:2> bit(1->8 bit) -> positive test current
	BMM150_Set_OpMode(bmm, 0xC6);
	HAL_Delay(1);
//	set force mode, readout Z and R channel, after measurement is finished
	BMM150_Set_OpMode(bmm, 0xC2);
	HAL_Delay(10);
	rslt = BMM150_ReadMultiBytes(bmm, 0x46,  &debug_array, 4);
	field_zp = (int16_t)((debug_array[1] << 8) + (debug_array[0] ));
	field_zp = field_zp/2;
	Rhall = (uint16_t)((debug_array[3] << 6) + (debug_array[2] >> 2));
	HAL_Delay(1);
	magz_p = BMM150_Compensate_z(field_zp, Rhall,  &trim_debug);
//	enable negative advanced self test current, 0x4C reg 10 <1:2> bit(1->8 bit) -> negative test current
	BMM150_Set_OpMode(bmm, 0x86);
	HAL_Delay(1);
//	set force mode, readout Z and R channel, after measurement is finished
	BMM150_Set_OpMode(bmm, 0x82);
	HAL_Delay(10);
	rslt = BMM150_ReadMultiBytes(bmm, 0x46,  &debug_array, 4);
	field_zn = (int16_t)((debug_array[1] << 8) + (debug_array[0] ));
	field_zn = field_zn/2;
	Rhall = (uint16_t)((debug_array[3] << 6) + (debug_array[2] >> 2));
	HAL_Delay(1);
	magz_n = BMM150_Compensate_z(field_zn, Rhall,  &trim_debug);
//	disable advanced self test current, 0x4C reg 00 <1:2> bit(1->8 bit) -> normal mode
	rslt = BMM150_WriteByte(bmm, 0x4C, 0x06);
//	calculate difference between the two compensated field values, result should be around 200 uT
//	perform soft-reset
*/

	HAL_Delay(10);
	BMM150_SoftReset(bmm);
	HAL_Delay(10);
//	rslt = BMM150_ReadByte(bmm, BMM150_CHIP_ID_ADDR, &chip_id);
//	HAL_Delay(10);
//	rslt = BMM150_ReadByte(bmm, BMM_PWR_CTRL_ADDR,  &debug);

	//set operation mode normal
	BMM150_Set_OpMode(bmm, opmode);
	HAL_Delay(10);
//	BMM150_ReadByte(bmm, BMM_PWR_CTRL_ADDR,  &debug);
//	HAL_Delay(10);
//	BMM150_ReadByte(bmm, BMM_PWR_CTRL_ADDR,  &debug);
//	HAL_Delay(10);
//	BMM150_ReadByte(bmm, BMM_OPMODE_REG,  &debug);
	//set ODR 20Hz
	BMM150_Set_ODR(bmm, BMM_ODR_30HZ);
	HAL_Delay(10);

	BMM150_Preset(bmm, 3, 6);
	HAL_Delay(10);

	BMM150_EN_DRDY_INT(bmm);

	return rslt;
}


HAL_StatusTypeDef	BMM150_SoftReset(BMM150 *bmm){
	HAL_StatusTypeDef rslt;
	uint8_t pwr_reg;
	rslt = BMM150_ReadByte(bmm, BMM_PWR_CTRL_ADDR,  &pwr_reg);
	pwr_reg = pwr_reg | 0b10000010;
	HAL_Delay(10);
	rslt = BMM150_WriteByte(bmm, BMM_PWR_CTRL_ADDR, pwr_reg);
	return rslt;
}

HAL_StatusTypeDef    BMM150_Set_OpMode(BMM150 *bmm, uint8_t opmode){
	HAL_StatusTypeDef rslt;
	uint8_t debug;

	rslt = BMM150_WriteByte(bmm, BMM_OPMODE_REG, opmode);
	HAL_Delay(1);
	rslt = BMM150_ReadByte(bmm, BMM_OPMODE_REG,  &debug);
	bmm->opmode = opmode;
	return rslt;
}

HAL_StatusTypeDef    BMM150_EN_DRDY_INT(BMM150 *bmm){
	HAL_StatusTypeDef rslt;
	uint8_t data;
	rslt = BMM150_ReadByte(bmm, BMM_DRDY_EN_REG,  &data);
	data = data | 0b11000000;
	HAL_Delay(1);
	rslt = BMM150_WriteByte(bmm, BMM_DRDY_EN_REG, data);

	return rslt;
}

HAL_StatusTypeDef    BMM150_Set_ODR(BMM150 *bmm, uint8_t odr){
	HAL_StatusTypeDef rslt;
	uint8_t op_ctrl_reg;

	rslt = BMM150_ReadByte(bmm, BMM_OPMODE_REG,  &op_ctrl_reg);
	op_ctrl_reg = op_ctrl_reg | odr;
	HAL_Delay(10);
	rslt = BMM150_WriteByte(bmm, BMM_OPMODE_REG, op_ctrl_reg);
	return rslt;
}

HAL_StatusTypeDef    BMM150_GetRawData(BMM150 *bmm, int16_t *field_x, int16_t *field_y, int16_t *field_z, uint16_t *Rhall, uint8_t len){
	HAL_StatusTypeDef rslt;
	uint8_t raw_field_data[8];
	int16_t msb_data;
	uint8_t lsb_data;
	rslt = BMM150_ReadMultiBytes(bmm, 0x42,  &raw_field_data, len);
//	*field_x = (int16_t)((raw_field_data[1] << 5) + (raw_field_data[0] >> 3));
//	*field_y = (int16_t)((raw_field_data[3] << 5) + (raw_field_data[2] >> 3));
//	*field_z = (int16_t)((raw_field_data[5] << 7) + (raw_field_data[4] >> 1));
	msb_data = (int16_t)((int8_t)raw_field_data[1]) * 32;
	lsb_data = (raw_field_data[0] & 0b11111000) >> 3;
	*field_x = (int16_t)(msb_data | lsb_data);

	msb_data = (int16_t)((int8_t)raw_field_data[3]) * 32;
	lsb_data = (raw_field_data[2] & 0b11111000) >> 3;
	*field_y = (int16_t)(msb_data | lsb_data);

	msb_data = (int16_t)((int8_t)raw_field_data[5]) * 128;
	lsb_data = (raw_field_data[4] & 0b11111110) >> 1;
	*field_z = (int16_t)(msb_data | lsb_data);

	lsb_data = (raw_field_data[6] & 0b11111100) >> 2;
	*Rhall = (uint16_t)((((uint16_t)raw_field_data[7]) << 6) | lsb_data);
	/*
	*field_x = (int16_t)((raw_field_data[1] << 8) + (raw_field_data[0] ));
	*field_x = *field_x/8;
	*field_y = (int16_t)((raw_field_data[3] << 8) + (raw_field_data[2] ));
	*field_y = *field_y/8;
	*field_z = (int16_t)((raw_field_data[5] << 8) + (raw_field_data[4] ));
	*field_z = *field_z/2;
	*Rhall = (uint16_t)((raw_field_data[7] << 6) + (raw_field_data[6] >> 2));*/
	return rslt;
}

HAL_StatusTypeDef    BMM150_Get_TrimData(BMM150 *bmm, BMM150_trim_data *trim){
	HAL_StatusTypeDef rslt;
	uint8_t readout1;
	uint8_t readout2;
	uint16_t temp_msb = 0;

	//dig_x1
	rslt = BMM150_ReadByte(bmm, 0x5D,  &readout1);
	trim->dig_x1 = (int8_t)readout1;
	HAL_Delay(2);
	//dig_y1
	rslt = BMM150_ReadByte(bmm, 0x5E,  &readout1);
	trim->dig_y1 = (int8_t)readout1;
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x64,  &readout1);
	trim->dig_x2 = (int8_t)readout1;
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x65,  &readout1);
	trim->dig_y2 = (int8_t)readout1;
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x71,  &readout1);
	trim->dig_xy1 = readout1;
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x70,  &readout1);
	trim->dig_xy2 = (int8_t)readout1;
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6A,  &readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6B,  &readout2);
	temp_msb = ((uint16_t)readout2) << 8;
	trim->dig_z1 = (uint16_t)(temp_msb | readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x68,  &readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x69,  &readout2);
	temp_msb = ((uint16_t)readout2) << 8;
	trim->dig_z2 = (int16_t)(temp_msb | readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6E,  &readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6F,  &readout2);
	temp_msb = ((uint16_t)readout2) << 8;
	trim->dig_z3 = (int16_t)(temp_msb | readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x62,  &readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x63,  &readout2);
	temp_msb = ((uint16_t)readout2) << 8;
	trim->dig_z4 = (int16_t)(temp_msb | readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6C,  &readout1);
	HAL_Delay(2);
	rslt = BMM150_ReadByte(bmm, 0x6D,  &readout2);
	temp_msb = ((uint16_t)readout2) << 8;
	trim->dig_xyz1 = (uint16_t)(temp_msb | readout1);

	return rslt;
}
HAL_StatusTypeDef    BMM150_GetRawData_Force(BMM150 *bmm, uint16_t *field_x, uint16_t *field_y, uint16_t *field_z, uint16_t *Rhall, uint8_t len){
	HAL_StatusTypeDef rslt;
	uint8_t debug;
	uint8_t raw_field_data[8];
	BMM150_Set_OpMode(bmm, 0x02);
	HAL_Delay(10);
	BMM150_ReadByte(bmm, 0x4C,  &debug);
	rslt = BMM150_ReadMultiBytes(bmm, 0x42,  &raw_field_data, len); //itt tovabbra is HAL_BUSY-t dob vissza
	*field_x = (uint16_t)((raw_field_data[1] << 5) + (raw_field_data[0] >> 3));
	*field_y = (uint16_t)((raw_field_data[3] << 5) + (raw_field_data[2] >> 3));
	*field_z = (uint16_t)((raw_field_data[5] << 7) + (raw_field_data[4] >> 1));
	*Rhall = (uint16_t)((raw_field_data[7] << 6) + (raw_field_data[6] >> 2));
	BMM150_ReadByte(bmm, 0x4C,  &debug); //0x4C reg value 0x02 tehát a Force Mode-ban van, miért? nem olvasom ki a data regisztereket
	return rslt;
}

HAL_StatusTypeDef    BMM150_Preset(BMM150 *bmm, uint8_t nXY, uint8_t nZ){
	HAL_StatusTypeDef rslt;
	uint8_t REPXY = (nXY-1)/2;
	uint8_t REPZ  = (nZ-1);
	//uint8_t debug;

	rslt = BMM150_WriteByte(bmm, BMM_REPXY_REG, REPXY); // nXY = 9 for 100HZ nXY = 1+2*(REPXY)
	HAL_Delay(10);
	rslt = BMM150_WriteByte(bmm, BMM_REPZ_REG, REPZ); // nZ = 15 for 100HZ nZ = 1+(REPXY)

	return rslt;
}


float    BMM150_Compensate_x(int16_t raw_mag_data_x, uint16_t raw_data_r,  BMM150_trim_data *trim){
	float compensated_X = 0;

	if (raw_mag_data_x != (-4096)) {
		if ((raw_data_r != 0) && (trim->dig_xyz1 != 0)) {
			compensated_X = ((((float)trim->dig_xyz1)*16384.0/(float)raw_data_r) - 16384.0);
		}
		else {
			compensated_X = 0.0f;
			return compensated_X;
		}
		compensated_X = ((((float)raw_mag_data_x * ((((((float)(trim->dig_xy2)) *(compensated_X*compensated_X /268435456.0) + compensated_X * ((float)(trim->dig_xy1))
		/ 16384.0)) + 256.0) *(((float)(trim->dig_x2)) + 160.0)))/ 8192.0)+ (((float)(trim->dig_x1)) *8.0)) / 16.0;
	}
	else {
		compensated_X = 0.0f;
	}
	return compensated_X;
}

//float    BMM150_Compensate_x(int16_t raw_mag_data_x, uint16_t raw_data_r,  BMM150_trim_data *trim){
//	float retval = 0;
//	float process_comp_x0;
//	float process_comp_x1;
//	float process_comp_x2;
//	float process_comp_x3;
//	float process_comp_x4;
//
//	if (raw_mag_data_x != (-4096)) {
//		if ((raw_data_r != 0) && (trim->dig_xyz1 != 0)) {
//			retval = ((((float)trim->dig_xyz1)*16384.0/(float)raw_data_r) - 16384.0);
//		}
//		else {
//			retval = 0.0f;
//			return retval;
//		}
//		process_comp_x0 = (((float)trim->dig_xyz1) * 16384.0f / raw_data_r);
//		retval = (process_comp_x0 - 16384.0f);
//		process_comp_x1 = ((float)trim->dig_xy2) * (retval * retval / 268435456.0f);
//		process_comp_x2 = process_comp_x1 + retval * ((float)trim->dig_xy1) / 16384.0f;
//		process_comp_x3 = ((float)trim->dig_x2) + 160.0f;
//		process_comp_x4 = raw_mag_data_x * ((process_comp_x2 + 256.0f) * process_comp_x3);
//		retval = ((process_comp_x4 / 8192.0f) + (((float)trim->dig_x1) * 8.0f)) / 16.0f;
//	}
//	else {
//		retval = 0.0f;
//	}
//	return retval;
//}


float   BMM150_Compensate_y(int16_t raw_mag_data_y, uint16_t raw_data_r,  BMM150_trim_data *trim){
	float compensated_Y = 0;

	if (raw_mag_data_y != (-4096)) {
		if ((raw_data_r != 0)&& (trim->dig_xyz1 != 0)) {
			compensated_Y = ((((float)(trim->dig_xyz1))* 16384.0/(float)raw_data_r) - 16384.0);
		}
		else {
			compensated_Y = 0.0f;
			return compensated_Y;
		}
		compensated_Y = ((((float)raw_mag_data_y * ((((((float)(trim->dig_xy2)) *(compensated_Y*compensated_Y/ 268435456.0) +compensated_Y
		* ((float)(trim->dig_xy1))/ 16384.0)) +256.0) *(((float)(trim->dig_y2)) + 160.0)))/ 8192.0) +(((float)(trim->dig_y1)) * 8.0))/ 16.0;
	}
	else {
		compensated_Y = 0.0f;
	}
	return compensated_Y;
}

//float   BMM150_Compensate_y(int16_t raw_mag_data_y, uint16_t raw_data_r,  BMM150_trim_data *trim){
//	float retval = 0;
//	float process_comp_y0;
//	float process_comp_y1;
//	float process_comp_y2;
//	float process_comp_y3;
//	float process_comp_y4;
//
//	if (raw_mag_data_y != (-4096)) {
//		if ((raw_data_r != 0)&& (trim->dig_xyz1 != 0)) {
//			retval = ((((float)(trim->dig_xyz1))* 16384.0/(float)raw_data_r) - 16384.0);
//		}
//		else {
//			retval = 0.0f;
//			return retval;
//		}
//		process_comp_y0 = ((float)trim->dig_xyz1) * 16384.0f / raw_data_r;
//		retval = process_comp_y0 - 16384.0f;
//		process_comp_y1 = ((float)trim->dig_xy2) * (retval * retval / 268435456.0f);
//		process_comp_y2 = process_comp_y1 + retval * ((float)trim->dig_xy1) / 16384.0f;
//		process_comp_y3 = ((float)trim->dig_y2) + 160.0f;
//		process_comp_y4 = raw_mag_data_y * (((process_comp_y2) + 256.0f) * process_comp_y3);
//		retval = ((process_comp_y4 / 8192.0f) + (((float)trim->dig_y1) * 8.0f)) / 16.0f;
//	}
//	else {
//		retval = 0.0f;
//	}
//	return retval;
//}




float    BMM150_Compensate_z(int16_t raw_mag_data_z, uint16_t raw_data_r,  BMM150_trim_data *trim){
	float compensated_Z = 0;
	if (raw_mag_data_z != (-16384)) {
		if ((trim->dig_z2 != 0)&& (trim->dig_z1 != 0)&& (trim->dig_xyz1 != 0)&& (raw_data_r != 0)) {
			compensated_Z = ((((((float)raw_mag_data_z)-((float)trim->dig_z4)) * 131072.0)-(((float)trim->dig_z3)*(((float)raw_data_r)
			-((float)trim->dig_xyz1))))/((((float)trim->dig_z2)+((float)trim->dig_z1)*((float)raw_data_r) /32768.0) * 4.0)) / 16.0;
		}
	}
	else {
		compensated_Z = 0.0f;
	}
	return (compensated_Z);
}

//float    BMM150_Compensate_z(int16_t raw_mag_data_z, uint16_t raw_data_r,  BMM150_trim_data *trim){
//	float retval = 0;
//	float process_comp_z0;
//	float process_comp_z1;
//	float process_comp_z2;
//	float process_comp_z3;
//	float process_comp_z4;
//	float process_comp_z5;
//
//
//	if (raw_mag_data_z != (-16384)) {
//		if ((trim->dig_z2 != 0)&& (trim->dig_z1 != 0)&& (trim->dig_xyz1 != 0)&& (raw_data_r != 0)) {
//			process_comp_z0 = ((float)raw_mag_data_z) - ((float)trim->dig_z4);
//			process_comp_z1 = ((float)raw_data_r) - ((float)trim->dig_xyz1);
//			process_comp_z2 = (((float)trim->dig_z3) * process_comp_z1);
//			process_comp_z3 = ((float)trim->dig_z1) * ((float)raw_data_r) / 32768.0f;
//			process_comp_z4 = ((float)trim->dig_z2) + process_comp_z3;
//			process_comp_z5 = (process_comp_z0 * 131072.0f) - process_comp_z2;
//			retval = (process_comp_z5 / ((process_comp_z4) * 4.0f)) / 16.0f;
//		}
//	}
//	else {
//		retval = 0.0f;
//	}
//	return (retval);
//}




HAL_StatusTypeDef	BMM150_ReadByte(BMM150 *bmm, uint8_t regAddr, uint8_t *data){
	return HAL_I2C_Mem_Read(bmm->hi2c_handle, BMM150_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, 1, 100);
}
HAL_StatusTypeDef	BMM150_ReadMultiBytes(BMM150 *bmm, uint8_t regAddr, uint8_t *data, uint8_t len){
	return HAL_I2C_Mem_Read(bmm->hi2c_handle, BMM150_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, data, len, 100);
}
HAL_StatusTypeDef	BMM150_WriteByte(BMM150 *bmm, uint8_t regAddr, uint8_t data){
	return HAL_I2C_Mem_Write(bmm->hi2c_handle, BMM150_ADDR << 1, regAddr, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);
}
