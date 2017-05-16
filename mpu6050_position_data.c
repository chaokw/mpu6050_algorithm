
/**************************************************************
*	Include File Section
**************************************************************/
#include "math.h"
#include "mpu6050_position_data.h"

/**************************************************************
*	Macro Define Section
**************************************************************/
#define SIGMA_WIDTH  2


/**************************************************************
*	Global Variable Declare Section
**************************************************************/
#if AVERGE_FILTER_OPEN || SIGMA_FILTER_OPEN
int16 acc_xyz_data[3][ACC_FILTER_COUNT] = { 0 };
int16 acc_data_index = 0;
#endif

#if ANGLE_FILTER_AVERGE_OPEN
int16 angle_data_index;
int32 angle_xyz_data[3][ANGLE_FILTER_COUNT] = { 0 };
#endif

int32 accel_xyz_data[3][ACC_FILTER_COUNT]; 
int16 accel_pos = 0;
extern float quat[4];
extern float locate_x, locate_y;
extern signed short int bs004_mpu6050_acc_pitch_cal,bs004_mpu6050_acc_roll_cal;

/**************************************************************
*	Function Define Section
**************************************************************/

/**************************************************************
*	Name...........:	accel_BConverttoN
*	description....:	将加速度的值从载体坐标系转为地理坐标系
*	param..........:	accel_res[out]	:转换得到的值，顺序为x,y,z
				accel:欲转换的载体坐标系的加速度值
				q:四元数
**************************************************************/
void accel_BConvertToN(int32 accel_res[3], int32 accel[3], float q[4])
{
	accel_res[0] = (q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3])*accel[0] + 2 * (q[1] * q[2] - q[0] * q[3])	* accel[1] + 2 * (q[0] * q[2] + q[1] * q[3]) * accel[2];
//	accel_res[1] = 2 * (q[0] * q[3] + q[1] * q[2]) * accel[0] + (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]) * accel[1] + (-2 * q[0] * q[1] + 2 * q[2] * q[3]) * accel[2];
	accel_res[1] = 2 * (q[0] * q[3] + q[1] * q[2]) * accel[0] + (q[0] * q[0] - q[1] * q[1] + q[2] * q[2] - q[3] * q[3]) * accel[1] + 2 *(q[2] * q[3] - q[0] * q[1]) * accel[2];
	accel_res[2] = 2 * (q[1] * q[3] - q[0] * q[2]) * accel[0] + 2 * (q[0] * q[1] + q[2] * q[3]) * accel[1] + (q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]) * accel[2];
}


#if AVERGE_FILTER_OPEN
/**************************************************************
*	Name...........:	accel_Filter
*	description....:	对原始数据加速度值进行滤波
*	param..........:	accel[in,changed]	:原始数据加速度值
				acc_ave[out]	:滤波得到的加速度值
**************************************************************/
void accel_Filter(int16 accel[3], int32 acc_ave[3])
{
	int i, j;
	int32	acc_data_sum[3] = { 0 };
	//先进行一次机械窗口滤波
	for (i = 0; i < 3; i++)
	{
		if (accel[i] < ACCEL_WINDOW_H && accel[i] > ACCEL_WINDOW_L)
			accel[i] = 0;
	}
	//将i轴的加速度保存在acc_data_index列中
	for (i = 0; i<3; i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}
	//acc_data_index循环加1
	acc_data_index++;

	if (acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
	//行求和
	for (i = 0; i<3; i++)
	{
		for (j = 0; j<ACC_FILTER_COUNT; j++)
		{
			acc_data_sum[i] += acc_xyz_data[i][j];
		}
		acc_ave[i] = acc_data_sum[i] / ACC_FILTER_COUNT;
	}
	//再对acc_ave进行一次机械窗口滤波
	for (i = 0; i<3; i++)
	{
		if (acc_ave[i] < ACCEL_WINDOW_H && acc_ave[i] > ACCEL_WINDOW_L)
			acc_ave[i] = 0;
	}
}
#endif


/**************************************************************
*	Name...........:	movement_End_Check
*	description....:	如果加速度的值为0的次数超过设置值，
				则将速度的值置0
*	param..........:	accel_n	:加速度值指针
				vel[in,changed]	:速度值指针,vel[1][0-2]为当前速度，
				vel[0][0-2]为上一速度
**************************************************************/
void movement_End_Check(int32 accel_n[3], int32 vel[2][3])
{
	static unsigned int countx = 0, county = 0, countz = 0;
	//处理X轴
	if (accel_n[0] == 0) //we count the number of acceleration samples that equals cero
	{
		countx++;
	}
	else
	{
		countx = 0;
	}
	if (countx >= TRESHOLD_COUNT) //if this number exceeds TRESHOLD_COUNT, we can assume that velocity is cero
	{
		vel[1][0] = 0;
		vel[0][0] = 0;
	}
	//处理Y轴
	if (accel_n[1] == 0) //we do the same for the Y axis
	{
		county++;
	}
	else
	{
		county = 0;
	}
	if (county >= TRESHOLD_COUNT)
	{
		vel[1][1] = 0;
		vel[0][1] = 0;
	}
	//处理Z轴
	if (accel_n[2] == 0)
	{
		countz++;
	}
	if (countz >= TRESHOLD_COUNT)
	{
		vel[1][2] = 0;
		vel[0][2] = 0;
	}
}


/**************************************************************
*	Name...........:	position
*	description....:	位移计算
*	param..........:	accel_n:地理坐标系的加速度值指针，
				accel_n[1][0-2]为当前加速度，accel_n[0][0-2]为前一加速度
				vel[in,out]:速度值指针，
				vel[1][0-2]为当前速度，vel[0][0-2]为上一速度
				displayment[out]:位移指针，
				displayment[1][0-2]为当前位移，displayment[0][0-2]为上一位移
**************************************************************/
void position(int32 accel_n[2][3], int32 vel[2][3], int32 displayment[2][3])
{
	int	i;

	for (i = 0; i < 3; i++)
	{
		vel[1][i] = vel[0][i] + ((accel_n[0][i] + accel_n[1][i]) >> 1);
		displayment[1][i] = displayment[0][i] + ((vel[1][i] + vel[0][i]) >> 1);
	}
}



#if SIGMA_FILTER_OPEN
void sigma_Filter(int16 accel[][ACC_FILTER_COUNT], int32 accel_res[][ACC_FILTER_COUNT], int16 pos, int16 N, int16 K)
{
	int32 sums[3][2] = { 0 }, sum[3] = { 0 };
	int16 i, j, temp, count[3] = {0};
	double delta[3], mean;

	for (i = 0; i < 3; i++)
	{
		//以pos为数据中心，求accel[pos-N,pos+N]的和、平方和
		for (j = pos - N; j <= pos + N; j++)
		{
			temp = accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT];
			sums[i][0] += temp;
			sums[i][1] += temp * temp;
		}
		//求序列的平均数和方差，最后求出sigma范围
		mean = sums[i][0] / (2 * N + 1);
		delta[i] = sums[i][1] / (2 * N + 1) - mean * mean;
		delta[i] = SIGMA_WIDTH * sqrt(delta[i]);
		//算出在sigma范围的元素个数及和
		for (j = pos - N; j <= pos + N; j++)
		{
			if (accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT] < delta[i] + accel[i][pos] && accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT] > accel[i][pos] - delta[i])
			{
				count[i]++;
				sum[i] += accel[i][(j + ACC_FILTER_COUNT) % ACC_FILTER_COUNT];
			}
		}
		//超过阀值，则用sigma范围里的元素的平均数替代
		if (count[i] >= K)
		{
			accel_res[i][pos] = sum[i] / count[i];
		}
		else//用序列中所有元素的平均数替代，除了accel[pos]
		{
			accel_res[i][pos] = (sums[i][0] - accel[i][pos]) / (2 * N);
		}
	}
}


void insert_AccelData(int16 accel[3])
{
	int i;
	
	//将i轴的加速度保存在acc_data_index列中
	for (i = 0; i<3; i++)
	{
		acc_xyz_data[i][acc_data_index] = accel[i];
	}
	//acc_data_index循环加1
	acc_data_index++;

	if (acc_data_index == ACC_FILTER_COUNT)
	{
		acc_data_index = 0;
	}
}
#endif

int16 originalPlace_Drift(int16 gyro[3])
{
	if((gyro[0] > GYRO_DRIFT_H || gyro[0] < GYRO_DRIFT_L)  || (gyro[1] > GYRO_DRIFT_H || gyro[1] < GYRO_DRIFT_L) || (gyro[2] > GYRO_DRIFT_H || gyro[2] < GYRO_DRIFT_L))
		return 1;
	else
		return 0;
}


#if ANGLE_FILTER_AVERGE_OPEN
/**************************************************************
*	Name...........:	angle_Filter
*	description....:	对原始数据值进行滤波
*	param..........:	angle[in,changed]	:原始数据加速度值
				angle_ave[out]	:滤波得到的加速度值
**************************************************************/
void angle_Filter(int32 angle[3], int32 angle_ave[3])
{
	int i, j;
	int32	angle_data_sum[3] = { 0 };

	//先进行一次机械窗口滤波
	for (i = 0; i < 3; i++)
	{
		if (angle[i] < ANGLE_WINDOW_H && angle[i] > ANGLE_WINDOW_L)
			angle[i] = 0;
	}
	//将i轴的加速度保存在angle_data_index列中
	for (i = 0; i<3; i++)
	{
		angle_xyz_data[i][acc_data_index] = angle[i];
	}
	//angle_data_index循环加1
	angle_data_index++;

	if (angle_data_index == ANGLE_FILTER_COUNT)
	{
		angle_data_index = 0;
	}
	//行求和
	for (i = 0; i<3; i++)
	{
		for (j = 0; j<ANGLE_FILTER_COUNT; j++)
		{
			angle_data_sum[i] += angle_xyz_data[i][j];
		}
		angle_ave[i] = angle_data_sum[i] / ANGLE_FILTER_COUNT;
	}

}
#endif


void BS004_Position_Calculation(signed short int ax, signed short int ay, signed short int az)   
{
	signed short int accel[3];
 	int32 vel[2][3]={0},disp[2][3] = {0},accel_ave[3],accel_res[2][3]={0};
	int16 i;

	accel[0] = ax - bs004_mpu6050_acc_roll_cal;
	accel[1] = ay - bs004_mpu6050_acc_pitch_cal;
	accel[2] = az;
	insert_AccelData(accel);

	sigma_Filter(acc_xyz_data,accel_xyz_data,accel_pos,15,4);
	accel_ave[0] = accel_xyz_data[0][accel_pos];
	accel_ave[1] = accel_xyz_data[1][accel_pos];
	accel_ave[2] = accel_xyz_data[2][accel_pos];
			
	accel_pos++;
	if (accel_pos == ACC_FILTER_COUNT)
	{
		accel_pos = 0;
	}

	accel_BConvertToN(accel_res[1],accel_ave,quat);
						
	for(i=0;i<3;i++)
	{
		if(accel_res[1][i] < ACCEL_WINDOW_H && accel_res[1][i] > ACCEL_WINDOW_L)
		accel_res[1][i] = 0;
	}
						
	accel_res[1][2] -= 14890;	
	position(accel_res,vel,disp);

	
	locate_x = (disp[0][0] - disp[1][0])*10/16384;   //2048;  //chaokw
	locate_y = (disp[1][1] - disp[0][1])*10/16384;  //2048;

	movement_End_Check(accel_res[1],vel);

			
	for(i = 0; i < 3; i++)
	{
		vel[0][i] = vel[1][i];
		disp[0][i] = disp[1][i];
		accel_res[0][i] = accel_res[1][i];
	}
}	
