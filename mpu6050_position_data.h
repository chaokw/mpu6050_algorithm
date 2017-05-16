#ifndef MPU6050_POSITION_DATA_H
#define MPU6050_POSITION_DATA_H


/**************************************************************
*	Macro Define Section
**************************************************************/
#define ACCEL_WINDOW_H  400
#define ACCEL_WINDOW_L  -400

#define ANGLE_WINDOW_H  1
#define ANGLE_WINDOW_L  -1

#define	ACC_FILTER_COUNT	50  //  1  //5   //chaokw
#define TRESHOLD_COUNT 10
#define ANGLE_FILTER_COUNT 4

#define SIGMA_FILTER_OPEN 	1   //chaokw
#define AVERGE_FILTER_OPEN  0

#define ANGLE_FILTER_AVERGE_OPEN  1
#define ANGLE_FILTER_AVERGE  1

#define GYRO_DRIFT_H  300
#define GYRO_DRIFT_L  -300

typedef signed short  int16;
typedef signed long  int32;

#if  SIGMA_FILTER_OPEN
extern int16	acc_xyz_data[3][ACC_FILTER_COUNT];
#endif

/**************************************************************
*	Prototype Declare Section
**************************************************************/
void accel_BConvertToN(int32 accel_res[3], int32 accel[3], float q[4]);
void accel_Filter(int16 accel[3], int32 acc_ave[3]);
void movement_End_Check(int32 accel_n[3], int32 vel[2][3]);
void position(int32 accel_n[2][3], int32 vel[2][3], int32 displayment[2][3]);
void sigma_Filter(int16 accel[][ACC_FILTER_COUNT], int32 accel_res[][ACC_FILTER_COUNT], int16 pos, int16 N, int16 K);
void insert_AccelData(int16 accel[3]);
int16 originalPlace_Drift(int16 gyro[3]);
void angle_Filter(int32 angle[3], int32 angle_ave[3]);
void BS004_Position_Calculation(signed short int ax, signed short int ay, signed short int az);

/**************************************************************
*	End-Multi-Include-Prevent Section
**************************************************************/
#endif
