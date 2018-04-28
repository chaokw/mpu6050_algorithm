

/**************************************************************
*	Include File Section
**************************************************************/
#include "mpu6050_etootle_imu.h" 
#include <string.h>
#include <stdio.h>

//#include "sensor.h"
//#include "bsp_i2c.h"


/**************************************************************
*	Global Variable Declare Section
**************************************************************/
float bs004_mpu6050_gyro_scale=0,bs004_mpu6050_pi_scale=0,bs004_gyro_to_rad_scale=0,bs004_hmc5883l_mag_scale=0;
float bs004_mpu6050_gyro_angel_pitch_ave,bs004_mpu6050_gyro_angel_roll_ave,bs004_mpu6050_gyro_angel_yaw_ave;
float bs004_mpu6050_acc_angel_pitch_ave,bs004_mpu6050_acc_angel_roll_ave,bs004_mpu6050_acc_angel_yaw_ave;

float q0=1,q1=0,q2=0,q3=0;	
float quat[4];
float exInt=0,eyInt=0,ezInt=0;	
float bs004_imu_pitch=0,bs004_imu_roll=0,bs004_imu_yaw=0;
float bs004_quad_Kp=0,bs004_quad_Ki=0,bs004_quad_halfT=0;

float a0=0, v0=0, vt=0;
float locate_x0=0, locate_x=0, locate_y=0;

signed short int bs004_mpu6050_acc_pitch_raw=0,bs004_mpu6050_acc_roll_raw=0,bs004_mpu6050_acc_yaw_raw=0;
signed short int bs004_mpu6050_gyro_pitch_raw=0,bs004_mpu6050_gyro_roll_raw=0,bs004_mpu6050_gyro_yaw_raw=0;
float  bs004_mpu6050_acc_pitch_com=0,bs004_mpu6050_acc_roll_com=0;
signed short int bs004_mpu6050_acc_pitch_cal=0,bs004_mpu6050_acc_roll_cal=0,bs004_mpu6050_acc_yaw_cal=0;
signed short int bs004_mpu6050_gyro_pitch_cal=0,bs004_mpu6050_gyro_roll_cal=0,bs004_mpu6050_gyro_yaw_cal=0;
float  bs004_filter_high=0,bs004_filter_low=0,bs004_filter_time=0;

unsigned int bs004_sys_timer_period=999;

float bs004_mpu6050_acc_scale=0,bs004_gyro_to_acc_scale=0;


/**************************************************************
*	Function Define Section
**************************************************************/
void BS004_Get_MPU6050_Data(signed short int ax, signed short int ay, signed short int az, signed short int gx, signed short int gy, signed short int gz)   
{
	
	bs004_mpu6050_acc_roll_raw=(signed short int )ax-bs004_mpu6050_acc_roll_cal;
	bs004_mpu6050_acc_pitch_raw=(signed short int)ay -bs004_mpu6050_acc_pitch_cal;
	bs004_mpu6050_acc_yaw_raw=(signed short int )az;
	bs004_mpu6050_gyro_pitch_raw=(signed short int )gx-bs004_mpu6050_gyro_pitch_cal;
	bs004_mpu6050_gyro_roll_raw=(signed short int )gy-bs004_mpu6050_gyro_roll_cal;
	bs004_mpu6050_gyro_yaw_raw=(signed short int )gz-bs004_mpu6050_gyro_yaw_cal;

	bs004_mpu6050_acc_pitch_com=bs004_filter_high*bs004_mpu6050_acc_pitch_com+bs004_filter_low*bs004_mpu6050_acc_pitch_raw;	
	bs004_mpu6050_acc_roll_com =bs004_filter_high*bs004_mpu6050_acc_roll_com +bs004_filter_low*bs004_mpu6050_acc_roll_raw;
	
       bs004_mpu6050_acc_angel_pitch_ave=(bs004_mpu6050_acc_angel_pitch_ave+bs004_mpu6050_acc_pitch_com)/2.0f;
	bs004_mpu6050_acc_angel_roll_ave =(bs004_mpu6050_acc_angel_roll_ave +bs004_mpu6050_acc_roll_com)/2.0f;
	bs004_mpu6050_acc_angel_yaw_ave  =(bs004_mpu6050_acc_angel_yaw_ave  +bs004_mpu6050_acc_yaw_raw)/2.0f;
	
	bs004_mpu6050_gyro_angel_pitch_ave=(bs004_mpu6050_gyro_angel_pitch_ave+bs004_mpu6050_gyro_pitch_raw)/2.0f;
	bs004_mpu6050_gyro_angel_roll_ave =(bs004_mpu6050_gyro_angel_roll_ave +bs004_mpu6050_gyro_roll_raw)/2.0f;
	bs004_mpu6050_gyro_angel_yaw_ave  =(bs004_mpu6050_gyro_angel_yaw_ave  +bs004_mpu6050_gyro_yaw_raw)/2.0f;
}


void BS004_Load_Filter_Parameter(void)
{
	int bs004_filter_par[12];
	
	bs004_filter_par[0]=950;
	bs004_filter_par[1]=50;
	bs004_filter_par[2]=5;   // 1;
	bs004_filter_par[3]=1000;
	bs004_filter_par[4]=1640;   //8192;    //1640;
	bs004_filter_par[5]=5730;   //8725;    //5730;
	bs004_filter_par[6]=1000;
	bs004_filter_par[7]=1000;
	bs004_filter_par[8]=36;
	bs004_filter_par[9]=5;  //6;   // 1;    chaolw
	bs004_filter_par[10]=1600;   //1600;   //1600  2000   chaokw   //pdf  10000
	bs004_filter_par[11]=1;    // 1;		// 1  5   //pdf  8
	
	bs004_filter_high=(float)bs004_filter_par[0]/1000.0f;				//滤波参数
	bs004_filter_low=(float)bs004_filter_par[1]/1000.0f;				//滤波参数	
	bs004_filter_time=(float)bs004_filter_par[2]/1000.0f;				//滤波参数		
	bs004_sys_timer_period=(unsigned int)bs004_filter_par[3]-1; 		//传感器采样频率
	
	bs004_mpu6050_gyro_scale=(float)bs004_filter_par[4]/100.0f;			//陀螺仪灵敏度
	
	bs004_mpu6050_pi_scale=(float)bs004_filter_par[5]/100.0f;		//弧度系数
	//bs004_hmc5883l_mag_scale=(float)bs004_filter_par[6]/1000.0f;		//磁力计灵敏度
	bs004_quad_halfT=(float)bs004_filter_par[9]/1000.0f;				//四元数时间系数
	bs004_quad_Kp=(float)bs004_filter_par[10]/1000.0f;				//四元数比例系数
	bs004_quad_Ki=(float)bs004_filter_par[11]/1000.0f;				//四元数积分系数
	
}


#if 0  //chaokw
void BS004_Cal_MPU6050_Data(int *bs004_cal_data)   
{
	unsigned char i,j;
	unsigned char bs004_mpu6050_cal_data_buffer[14];
	signed short int bs004_mpu6050_cal_data[10][7];   //
	int bs004_mpu6050_cal_sum[7];
	
	for(i=0;i<7;i++) bs004_mpu6050_cal_sum[i]=0;
	
	for(j=0;j<10;j++) 
	{
	       bspI2cSelect(BSP_I2C_INTERFACE_1,0x68);
              sensorReadReg(0x3b, (unsigned char*)bs004_mpu6050_cal_data_buffer, 14); 				
		bspI2cDeselect(); 
		for(i=0;i<7;i++) bs004_mpu6050_cal_data[j][i]=(((signed short int)bs004_mpu6050_cal_data_buffer[i*2]) << 8) | bs004_mpu6050_cal_data_buffer[i*2+1];
	}
	
	for(i=0;i<7;i++)
	{
		for(j=0;j<10;j++) bs004_mpu6050_cal_sum[i]=bs004_mpu6050_cal_sum[i]+(int)bs004_mpu6050_cal_data[j][i];   //
	}
	
	bs004_cal_data[0]=bs004_mpu6050_cal_sum[4]/10;
	bs004_cal_data[1]=bs004_mpu6050_cal_sum[5]/10;
	bs004_cal_data[2]=bs004_mpu6050_cal_sum[6]/10;
	bs004_cal_data[3]=bs004_mpu6050_cal_sum[0]/10;
	bs004_cal_data[4]=bs004_mpu6050_cal_sum[1]/10;
	bs004_cal_data[5]=bs004_mpu6050_cal_sum[2]/10;;
	bs004_cal_data[6]=0;
	bs004_cal_data[7]=0;
	bs004_cal_data[8]=0;

}
#endif


#if 1    //chaokw       Calibration to be done by mpu6050
void BS004_Load_Calibration_Parameter(void)
{
//	int bs004_cal_par[12];

#if 0
	BS004_Cal_MPU6050_Data(bs004_cal_par);
#endif


#if 0   //chaokw
	bs004_mpu6050_gyro_pitch_cal=(signed short int)bs004_cal_par[0];	//陀螺仪校验参数
	bs004_mpu6050_gyro_roll_cal=(signed short int)bs004_cal_par[1];   //陀螺仪校验参数
	bs004_mpu6050_gyro_yaw_cal=(signed short int)bs004_cal_par[2];    //陀螺仪校验参数
	bs004_mpu6050_acc_roll_cal=(signed short int)bs004_cal_par[3];		//加速度校验参数
	bs004_mpu6050_acc_pitch_cal=(signed short int)bs004_cal_par[4];   //加速度校验参数
	bs004_mpu6050_acc_yaw_cal=(signed short int)bs004_cal_par[5];     //加速度校验参数
	//bs004_hmc5883l_mag_pitch_cal=(signed short int)bs004_cal_par[6];  //磁力计校验参数
	//bs004_hmc5883l_mag_roll_cal=(signed short int)bs004_cal_par[7];   //磁力计校验参数
	//bs004_hmc5883l_mag_yaw_cal=(signed short int)bs004_cal_par[8];    //磁力计校验参数
#else
	bs004_mpu6050_gyro_pitch_cal=-4;   //-15;    水平
	bs004_mpu6050_gyro_roll_cal=26;     //14;
	bs004_mpu6050_gyro_yaw_cal=-4;    //5;
	bs004_mpu6050_acc_roll_cal=80;    //-34;
	bs004_mpu6050_acc_pitch_cal=62;   //48;
	bs004_mpu6050_acc_yaw_cal=1935;   //2048;
#endif

       //DISPLACEMENT
//	signed short int accel[3];
//	accel[0] = bs004_mpu6050_acc_roll_cal;
//	accel[1] = bs004_mpu6050_acc_pitch_cal;
//	accel[2] = bs004_mpu6050_acc_yaw_cal;
//	insert_AccelData(accel);


}
#endif


void BS004_Quad_Calculation(void)
{
	float ax=0,ay=0,az=0,gx=0,gy=0,gz=0;
	
	bs004_gyro_to_rad_scale=bs004_mpu6050_pi_scale*bs004_mpu6050_gyro_scale;

	gx=bs004_mpu6050_gyro_angel_pitch_ave/bs004_gyro_to_rad_scale;
	gy=bs004_mpu6050_gyro_angel_roll_ave/bs004_gyro_to_rad_scale;
	gz=bs004_mpu6050_gyro_angel_yaw_ave/bs004_gyro_to_rad_scale;		
	ax=bs004_mpu6050_acc_angel_roll_ave;
	ay=bs004_mpu6050_acc_angel_pitch_ave;	
	az=bs004_mpu6050_acc_angel_yaw_ave;		
	
	BS004_IMU_Update(ax,ay,az,gx,gy,gz);
}

unsigned char BS004_IMU_Update(float ax,float ay,float az,float gx,float gy,float gz) 
{
	float norm;
	float vx, vy, vz;
	float ex, ey, ez;  
	float gz_input;  //chaokw
	
	//四元数乘法运算
	float q0q0 = q0 * q0;							
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q1q1 = q1 * q1;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
		
	gz_input=gz*bs004_quad_halfT;
	
	//归一化处理
	norm = sqrt(ax*ax + ay*ay + az*az);     
	if(norm==0) return 0;	
	ax = ax / norm;
	ay = ay / norm;
	az = az / norm;   
  
	//建立小四轴坐标系	
	vx = 2*(q1q3 - q0q2);								
	vy = 2*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	
	//坐标系和重力叉积运算
	ex = (ay*vz - az*vy);								
	ey = (az*vx - ax*vz);
	ez = (ax*vy - ay*vx);
	
	//比例运算
	exInt = exInt + ex*bs004_quad_Ki;
	eyInt = eyInt + ey*bs004_quad_Ki;
	ezInt = ezInt + ez*bs004_quad_Ki;
	
	//陀螺仪融合
	gx = gx + bs004_quad_Kp*ex + exInt;
	gy = gy + bs004_quad_Kp*ey + eyInt;
	gz = gz + bs004_quad_Kp*ez + ezInt;
	
	//整合四元数率
	q0 = q0 + (-q1*gx - q2*gy - q3*gz)*bs004_quad_halfT;
	q1 = q1 + (q0*gx + q2*gz - q3*gy)*bs004_quad_halfT;
	q2 = q2 + (q0*gy - q1*gz + q3*gx)*bs004_quad_halfT;
	q3 = q3 + (q0*gz + q1*gy - q2*gx)*bs004_quad_halfT;  
	
	//归一化处理
	norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
	if(norm==0) return 0;	
	q0 = q0 / norm;
	q1 = q1 / norm;
	q2 = q2 / norm;
	q3 = q3 / norm;

       //chaokw
	quat[0] = q0;
	quat[1] = q1;
	quat[2] = q2;
	quat[3] = q3;

	
	//欧拉角转换
	bs004_imu_roll = asin(-2*q1q3 + 2*q0q2)*57.30f;
 	bs004_imu_pitch = atan2(2*q2q3 + 2*q0q1, -2*q1q1-2*q2q2 + 1)*57.30f; 
		
	bs004_imu_yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2*q2 - 2 * q3* q3 + 1)*57.30f;   //chaokw  20160907
       //bs004_imu_yaw = atan2(2*(q1*q2 + q0*q3), q0q0+q1q1-q2q2-q3q3)*57.30f;    //  ok

	return 1;	
}


