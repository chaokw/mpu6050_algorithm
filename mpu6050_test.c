/**************************************************************
*	Include File Section
**************************************************************/
#include <string.h>
#include <stdio.h>
//#include "Board_uart.h"
#include "mpu6050_etootle_imu.h"      //欧拉角
#include "mpu6050_position_data.h"     //位移


signed short int ax, ay, az, gx, gy, gz;   //存储从蓝牙接收到的6轴数据， ax,ay,az为加速度，gx,gy,gz为陀螺仪
int32 accel_xyz_data[3][ACC_FILTER_COUNT];  //chaokw
int16 accel_pos = 0;

int main()
{

	signed short int accel[3];
 	int32 vel[2][3]={0},disp[2][3] = {0},accel_ave[3],accel_res[2][3]={0};
	int16 i;

	BS004_Load_Filter_Parameter();
  	BS004_Load_Calibration_Parameter();
  
	while(1)
	{	
	
		// 姿态计算(欧拉角)
		BS004_Get_MPU6050_Data(ax, ay, az, gx, gy, gz);
		BS004_Quad_Calculation();

		// 位移计算
		accel[0] = ax;
		accel[1] = ay;
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

		extern float quat[4];
		accel_BConvertToN(accel_res[1],accel_ave,quat);
						
		for(i = 0; i < 3; i++)
		{
			if(accel_res[1][i] < ACCEL_WINDOW_H && accel_res[1][i] > ACCEL_WINDOW_L)
			accel_res[1][i] = 0;
		}
						
		accel_res[1][2] -= 14890;	
		position(accel_res,vel,disp);
	
		locate_x = (disp[0][0] - disp[1][0])*10/2048;
		locate_y = (disp[1][1] - disp[0][1])*10/2048;

		movement_End_Check(accel_res[1],vel);
			
		for(i = 0; i < 3; i++)
		{
			vel[0][i] = vel[1][i];
			disp[0][i] = disp[1][i];
			accel_res[0][i] = accel_res[1][i];
		}


 		//串口打印计算出的姿态和位移
 		char strTemp[128];
		sprintf(strTemp, "pitch:%f, roll:%f, yaw:%f, x:%f, y:%f\r\n",bs004_imu_pitch, bs004_imu_roll, bs004_imu_yaw, locate_x, locate_y);                                                                                                                        
		UART_WriteTransport((uint8*)strTemp, strlen(strTemp)); 

		sleep(5);
		
	}
}
