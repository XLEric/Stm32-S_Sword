#ifndef __IMU_H
#define __IMU_H	 		  
#include "sys.h" 


void Element_4_int(float acx ,float acy ,float acz, float a);
void IMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx,int mx,int my,int mz,bool flag_compass ) ;
void Mag();
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)  ;
void IMU_AHRSupdateComPass(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)  ;
float invSqrt(float x);

void AHRS_AHRSupdate_K(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass);
void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass);
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass);
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass);
void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass);
#define DF float
//#include "Include.h"

//-------------------------------------------//
//函数名称：MatrixAdd
//函数功能：矩阵加法
//函数描述：
//参数说明：
//返回值：
//创建时间：2004.05.18
//修改时间：2004.05.28
//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
void MatrixAdd( float* fMatrixA,float* fMatrixB,float* Result,
		unsigned int m,unsigned int n )
;

//-------------------------------------------//
//函数名称：MatrixSub
//函数功能：矩阵减法
//函数描述:
//参数说明:
//返回值：
//创建时间：2004.05.18
//修改时间：2004.05.28
//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
void MatrixSub( float* fMatrixA,float* fMatrixB,float* Result,
		unsigned int m,unsigned int n )
;
//-------------------------------------------//
//函数名称：MatrixMultiply
//函数功能：矩阵乘法
//函数描述：
//参数说明：
//返回值：
//创建时间：2004.05.18
//修改时间：2004.05.28(改为指针操作)、2004.06.02(加入乘前的判断条件)
//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//

void MatrixMultiply( float* fMatrixA,unsigned int uRowA,unsigned int uColA,
float* fMatrixB,unsigned int uRowB,unsigned int uColB,float* MatrixResult )
;

//-------------------------------------------//
//函数名称：MatrixTranspose
//函数功能：矩阵转置
//函数描述：
//参数说明：
//返回值：
//创建时间：2004.05.18
//修改时间?2004.05.28?//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
void MatrixTranspose(float* fMatrixA,unsigned int m,unsigned n,float* fMatrixB)
;
void MatrixProduct(float* A, int m, int n, float* B, int k, float* C)							 
;
//-------------------------------------------//
//函数名称：MatrixE
//函数功能：单位矩阵生成?//函数描述：
//参数说明：
//返回值：
//创建时间：2004.05.19
//修改时间：
//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
void MatrixE(float* fMatrixA,unsigned int n)
;

//-------------------------------------------//
//函数名称：MatrixDet2
//函数功能：2阶矩阵行列式的值
//函数描述：
//参数说明：
//返回值：
//创建时间：2004.05.19
//修改时间：
//测试时间：
//测试方法：
//-------------------------------------------//
double MatrixDet2(float* fMatrixA)
;
//-------------------------------------------//
//函数名称：MatrixInverse2
//函数功能：2阶矩阵求逆
//函数描述：
//参数说明
//返回值：
//创建时间：2004.05.18
//修改时间：
//测试时间:
//测试方法：
//-------------------------------------------//
int MatrixInverse2(float* fMatrixA,float* fMatrixB)
;

//-------------------------------------------//
//函数名称：
//函数功能：矩阵求逆
//函数描述：
//参数说明：
//返回值：
//创建时间：004.05.18
//修改时间：?//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
int MatrixInverse(float* fMatrixA,int n,float ep)
;
void UD(float * A,int  n,float * U,float * D);
//-------------------------------------------//
//函数名称：Norm
//函数功能：求矩阵范数
//函数描述：||A||
//参数说明：
//返回值：
//创建时间：2004.07.28
//修改时间：?//测试时间：2004.07.28
//测试方法：功能测试
//-------------------------------------------//
DF Norm(float*fMatrixA,int iRow,int iCol)
;
#endif
