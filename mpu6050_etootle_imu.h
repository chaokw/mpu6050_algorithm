#ifndef MPU6050_ETOOTLE_IMU_H
#define MPU6050_ETOOTLE_IMU_H

/**************************************************************
*	Include File Section
**************************************************************/
#include <math.h>

extern float bs004_imu_pitch, bs004_imu_roll, bs004_imu_yaw;
extern float locate_x, locate_y;  //chaokw


/**************************************************************
*	Prototype Declare Section
**************************************************************/
void BS004_Load_Filter_Parameter(void);
void BS004_Load_Calibration_Parameter(void);
void BS004_Quad_Calculation(void);
unsigned char BS004_IMU_Update(float ax,float ay,float az,float gx,float gy,float gz);
void BS004_Get_MPU6050_Data(signed short int ax, signed short int ay, signed short int az, signed short int gx, signed short int gy, signed short int gz);  

#endif
