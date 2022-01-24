/* Includes ------------------------------------------------------------------*/
#include "main.h"

void Init_LSM303DLH_ACC(void)
{
	//unsigned char reg_temp;
	//LSM303DLH 加速度传感器配置
	I2C_Write_Byte(ACC_ADDR,LSM303DLH_CTRL_REG1_A,0x27);//Power mode:normal mode，data rate:10Hz，xyx：enable
	I2C_Write_Byte(ACC_ADDR,LSM303DLH_CTRL_REG2_A,0x00);//boot,hpm,fds,hpen,hpcf
	I2C_Write_Byte(ACC_ADDR,LSM303DLH_CTRL_REG3_A,0x00);//ihl,pp_od,lir,i2_cfg,i1_cfg
	I2C_Write_Byte(ACC_ADDR,LSM303DLH_CTRL_REG4_A,0x00);//bdu,ble,fs,stsign,st
	I2C_Write_Byte(ACC_ADDR,LSM303DLH_CTRL_REG5_A,0x00);//turn on
	//LSM303DLH 滤波器设置
}


void  Read_ACC_Data_Task(void)
{
	//X轴旋转表示Pitch----俯仰角
	//Y轴旋转表示Yaw----航向角
	//Z轴旋转表示Roll
	unsigned char acc_xl,acc_xh,acc_yl,acc_yh,acc_zl,acc_zh;
	short int acc_x,acc_y,acc_z;
	double a_x,a_y,a_z;

	
	acc_xl=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_X_L_A);
	acc_xh=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_X_H_A);
	acc_yl=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_Y_L_A);
	acc_yh=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_Y_H_A);
	acc_zl=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_Z_L_A);
	acc_zh=I2C1_Read_Byte(ACC_ADDR,LSM303DLH_OUT_Z_H_A);
	
	acc_x= (acc_xh<<8) | acc_xl ;
	acc_y= (acc_yh<<8) | acc_yl ;
	acc_z= (acc_zh<<8) | acc_zl ;
	
	a_x=acc_x/32.768*2;       //换算为加速度值
	a_y=acc_y/32.768*2;
	a_z=acc_z/32.768*2;
	
	Angle_x=(atan2(a_x,sqrt(a_y*a_y+a_z*a_z)))*180/3.1415926;
	Angle_y=(atan2(a_y,sqrt(a_x*a_x+a_z*a_z)))*180/3.1415926;
	Angle_z=(atan2(a_z,sqrt(a_x*a_x+a_y*a_y)))*180/3.1415926;
	
	//log_info("\r\n");
	
	//log_info("X轴角度：%f\r\n",Angle_x);
	//log_info("Y轴角度：%f\r\n",Angle_y);
	//log_info("Z轴角度：%f\r\n",Angle_z);
	
	
}


void LSM303DLH_MagInit(void)
{
	I2C_Write_Byte(MAG_ADDR,LSM303DLH_CRA_REG_M,0x00);//
	I2C_Write_Byte(MAG_ADDR,LSM303DLH_CRB_REG_M,0x00);//
	I2C_Write_Byte(MAG_ADDR,LSM303DLH_MR_REG_M,0x00);//
	
}







