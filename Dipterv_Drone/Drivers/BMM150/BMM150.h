#ifndef LIB_BMM150_HAL_BMM150_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

/* Declarations and definitions ----------------------------------------------*/
#define BMM150_ADDR 				0x10
#define BMM150_CHIP_ID_ADDR			0x40
#define BMM150_CHIP_ID				0x32
#define BMM_PWR_CTRL_ON				0b00000001
#define BMM_PWR_CTRL_ADDR			0x4B


#define BMM_OPMODE_REG				0x4C
#define BMM_OPMODE_NORMAL			0x00
#define	BMM_OPMODE_FORCE			0x01

#define BMM_ODR_20HZ				0x28
#define BMM_ODR_25HZ				0x30
#define BMM_ODR_30HZ				0x38
#define BMM_ODR_10HZ				0x00

#define BMM_REPXY_REG				0x51
#define BMM_REPZ_REG				0x52

#define BMM_XDATA_REG				0x42
#define BMM_DRDY_EN_REG				0x4E


typedef struct{
	I2C_HandleTypeDef 		*hi2c_handle;
	uint8_t					*opmode;


} BMM150;

typedef struct {
    /*! trim x1 data */
    int8_t dig_x1;
    /*! trim y1 data */
    int8_t dig_y1;
    /*! trim x2 data */
    int8_t dig_x2;
    /*! trim y2 data */
    int8_t dig_y2;
    /*! trim z1 data */
    uint16_t dig_z1;
    /*! trim z2 data */
    int16_t dig_z2;
    /*! trim z3 data */
    int16_t dig_z3;
    /*! trim z4 data */
    int16_t dig_z4;
    /*! trim xy1 data */
    uint8_t dig_xy1;
    /*! trim xy2 data */
    int8_t dig_xy2;
    /*! trim xyz1 data */
    uint16_t dig_xyz1;
} BMM150_trim_data;



HAL_StatusTypeDef    BMM150_Init(BMM150 *bmm);
HAL_StatusTypeDef    BMM150_Set_OpMode(BMM150 *bmm, uint8_t opmode);
HAL_StatusTypeDef    BMM150_Preset(BMM150 *bmm, uint8_t nXY, uint8_t nZ);
HAL_StatusTypeDef    BMM150_Set_ODR(BMM150 *bmm, uint8_t odr);
HAL_StatusTypeDef    BMM150_EN_DRDY_INT(BMM150 *bmm);
HAL_StatusTypeDef    BMM150_Get_TrimData(BMM150 *bmm, BMM150_trim_data *trim);
HAL_StatusTypeDef    BMM150_GetRawData(BMM150 *bmm, int16_t *field_x, int16_t *field_y, int16_t *field_z, uint16_t *Rhall, uint8_t len);
HAL_StatusTypeDef    BMM150_GetRawData_Force(BMM150 *bmm, uint16_t *field_x, uint16_t *field_y, uint16_t *field_z, uint16_t *Rhall, uint8_t len);
float    BMM150_Compensate_x(int16_t raw_mag_data_x, uint16_t raw_data_r,  BMM150_trim_data *trim);
float    BMM150_Compensate_y(int16_t raw_mag_data_y, uint16_t raw_data_r,  BMM150_trim_data *trim);
float    BMM150_Compensate_z(int16_t raw_mag_data_z, uint16_t raw_data_r,  BMM150_trim_data *trim);


#endif
