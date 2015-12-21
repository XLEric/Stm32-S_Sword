/***********************************************************/
// 版本 Version 1.0
// 日期：2015-1-18
// 当获得第一次 加速度后 且，程序循环体外执行 Element_4_int，初始化四元数 
// 在一定定时时间内 循环更新 四元数 执行 函数 IMU
// (算法时间由IMU 及 通过SPI 获取 三轴加速度 角速度 此偏量 共同决定)；
// 运算一次总时间为：T= 传感器 9参数获取时间 + IMU 时间       （之后要考虑通信时间）

#include<math.h>
#include "sys.h"
#include "imu.h"
#include<math.h>
#include "key.h"

#define DF float
#define PI 3.1415926
//#define Kp 2.01f    //误差增益     IMU              
//#define Ki 0.001f   

#define Kp 1.800f    //误差增益    HARS              
#define Ki 0.002f  
 
#define MAG_ADDRESS2    0x18   //磁场地址            
#define	GYRO_ADDRESS2   0xD0	  //陀螺地址

//-------------------------------------------------
#define sampleFreq	512.0f			// sample frequency in Hz
#define twoKpDef	(2.1f * 0.5f)	// 2 * proportional gain
#define twoKiDef	(2.0f * 0.0001f)	// 2 * integral gain

#define betaDef		0.1f		// 2 * proportional gain

// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

volatile float q0 = 1, q1 = 0, q2 = 0, q3 = 0,w1,w2,w3;        // 四元数
volatile float q0r = 1, q1r = 0, q2r = 0, q3r = 0;
volatile float exInt = 0, eyInt = 0, ezInt = 0;       //误差

// 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool
volatile float Pitch;//弧度
volatile float Rool ;
volatile float Yaw  ;

volatile short Pitch_angle;//角度
volatile short Rool_angle ;
volatile short Yaw_angle  ;

volatile float G0=9.7999;//初始化
float Yaw_JZ;//航向角 校正参数

u8 key_num;

//---------------------------------------------------------------------------------------------------
/************************ 四元数初始化 ***********************/
//	入口参数：
// 	acx  x轴重力加速度
// 	acy  y轴重力加速度
// 	acz  z轴重力加速度
//      a    初始化为 0

//	输出全局：四元数

void Element_4_int(float acx ,float acy ,float acz, float a)
{
	float rot_xx= atan2(acy,acz);//弧度 单位
	float rot_yy= -asin(acx/G0);

	float rot_zz= a;
	//****** 四元数初始化 ******//
	q0 = cos(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	q1 = sin(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)-cos(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	q2 = cos(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2);

	q3 = cos(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2)-sin(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2);

}

/***********************************************   四元数更新函数   *********************************************/
void IMU_AHRSupdateComPass(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass) 
{
 	float rot_xx;//弧度 单位
	float rot_yy;

	float rot_zz;
  float p,r,y;
  float Hx,Hy;
  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;
  	float t11;
	float t12;
	float t13;
	float t21;
	float t22;
	float t23;
	float t31;
	float t32;
	float t33;

	

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;    

    //int mx_int=mx;
	//int my_int=my;
	//int mz_int=mz;

	//int mx_int=mx-95.4753f;  //Company Blue
	//int my_int=my-176.1358f;
	//int mz_int=mz+23.2192f;

	//int mx_int=mx-60.2483f;  //Company
	//int my_int=my-218.7394f;
	//int mz_int=mz+78.3695f;

	//int mx_int=mx-57.8186;// HOME
	//int my_int=my-219.3857;
	//int mz_int=mz+47.5079;

	int mx_int=mx-90.2641f;// HOME Blue
	int my_int=my-160.1873f;
	int mz_int=mz+5.1121f;

	 //-------------------
	 mx=mx-90.4753f;
	 my=my-160.1358f;
	 mz=mz+5.2192f;
	 /************************/
	 key_num=KEY_Scan();
	 if(key_num==3)
	 {
	 	q0=1;q1=0;q2=0;q3=0;
	 }

	 if(abs(gz)<2.0f/PI)
		{
			ezInt=0;
		}

	// 重力坐标系
	//矫正零偏
//		ax=(ax-0.5)/9.85;
//		ay=(ay-0.5)/9.85;
//		az=(az+0.675)/10.025;

   // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
//        norm = sqrt(mx*mx + my*my + mz*mz);          
//        mx = mx / norm;
//        my = my / norm;
//        mz = mz / norm;         
//        
//        // compute reference direction of flux
//        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
//        bx = sqrt((hx*hx) + (hy*hy));
//        bz = hz;        
        
        // estimated direction of gravity and flux (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
		//vz=2*q0q0 - 1.0f + 2*q3q3	;
//        wx = bx*(0.5 - q2q2 - q3*q3) + bz*(q1q3 - q0q2);
//        wy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
//        wz = bx*(q0q2 + q1q3) + bz*(0.5 - q1q1 - q2q2);  

	//	halfvx = q1q3 - q0q2;
	//	halfvy = q0q1 + q2q3;
	//	halfvz = q0q0 - 0.5f + q3q3;
        
        
        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
        ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);
        
        // integral error scaled integral gain

		
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

	//	printf("&%d *%d ",(int)(gx*100000),(int)(ex*100000));
        
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        // integrate quaternion rate and normalise
        q0r = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1r = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2r = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3r = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

		q0=q0r;q1=q1r;q2=q2r;q3=q3r;
        
        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;
//------------------------------------------------------------------------------------------------------

	//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	Rool = atan2(t23,t33);
	Pitch = -asin(t13);

	Hx=mx*sin(Rool)*sin(Pitch)+my*cos(Pitch)-mz*cos(Rool)*sin(Pitch);
	Hy=mx*cos(Rool)+mz*sin(Rool);

	
	Yaw=atan2(Hx,Hy);
	Yaw_angle=180.0*Yaw/3.1415926;

	if (Yaw_angle < 0) Yaw_angle += 360;

	//------------------
	rot_xx=Rool;
	rot_yy=Pitch;
	rot_zz=Yaw;
	q0 = cos(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	q1 = sin(rot_xx/2)*cos(rot_yy/2)*cos(rot_zz/2)-cos(rot_xx/2)*sin(rot_yy/2)*sin(rot_zz/2);

	q2 = cos(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2)+sin(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2);

	q3 = cos(rot_xx/2)*cos(rot_yy/2)*sin(rot_zz/2)-sin(rot_xx/2)*sin(rot_yy/2)*cos(rot_zz/2);


//------------------------------------------------------------------------------------------------------
//将上位机的磁场移到下位机
//	Rool=atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
//	Pitch=asin(2*(q0*q2-q0*q3));
//	Yaw=atan2(2*(q0*q1+q0*q3),1-2*(q3*q3+q1*q1));
//
//	////--- 弧度转为角度
//	Rool_angle = Rool/3.1415926*180;
//	Pitch_angle = Pitch/3.1415926*180;
//	Yaw_angle = Yaw/3.1415926*180;
//
//	if (Yaw_angle < 0) Yaw_angle += 360;
//
//	//------------------------------------------
//	p=Pitch;
//	r=Rool;
//
//	Hx=mx*sin(r)*sin(p)+my*cos(p)-mz*cos(r)*sin(p);
//	Hy=mx*cos(r)+mz*sin(r);
//
//	
//	Yaw=atan2(Hx,Hy);
//	Yaw_angle=180.0*Yaw/3.1415926;
//
//	if (Yaw_angle < 0) Yaw_angle += 360;
//
//	
//
//	q0=cos(r)*cos(p)*cos(y) + sin(r)*sin(p)*sin(y);
//	q1=sin(r)*cos(p)*cos(y) - cos(r)*sin(p)*sin(y);
//	q2=cos(r)*sin(p)*cos(y) + sin(r)*cos(p)*sin(y);
//	q3=cos(r)*cos(p)*sin(y) - sin(r)*sin(p)*cos(y);


//------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------

	//更新方向余弦矩阵
//	t11=q0*q0+q1*q1-q2*q2-q3*q3;
//	t12=2.0*(q1*q2+q0*q3);
//	t13=2.0*(q1*q3-q0*q2);
//	t21=2.0*(q1*q2-q0*q3);
//	t22=q0*q0-q1*q1+q2*q2-q3*q3;
//	t23=2.0*(q2*q3+q0*q1);
//	t31=2.0*(q1*q3+q0*q2);
//	t32=2.0*(q2*q3-q0*q1);
//	t33=q0*q0-q1*q1-q2*q2+q3*q3;
//
//	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
//	Rool = atan2(t23,t33);
//	Pitch = -asin(t13);
////	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
//	{
//	Yaw = atan2(t12,t11);
//
//	if (Yaw < 0) Yaw += 360/180*PI;
//	}
//	//Yaw += 360/180*PI;
////	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;
//
//	//--- 弧度转为角度
//	Rool_angle = Rool/PI*180;
//	Pitch_angle = Pitch/PI*180;
//	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%06d", (int)(q0*10000));
		 printf("%06d", (int)(q1*10000));
		 printf("%06d", (int)(q2*10000));
		 printf("%06d", (int)(q3*10000));

		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

		 printf("%04d",(int)(gx*100));
		 printf("%04d",(int)(gy*100));
		 printf("%04d",(int)(gz*100));

		// printf("%04d",(int)(ax*1000));
		// printf("%04d",(int)(ay*1000));
		// printf("%04d",(int)(az*1000));
//


		 
		 
//
//		 printf("%05d",(int)(hx*1000));
//		 printf("%05d",(int)(hy*1000));
//		 printf("%05d",(int)(hz*1000));

		
	}
}

//	入口参数： 
//	gx x轴角速度（弧度）
//	gy y轴角速度（弧度）
//	gz z轴角速度（弧度）

//	ax x轴重力加速度
//	ay y轴重力加速度
//	az z轴重力加速度

//	halfT 积分时间
//	flag_control Yaw漂移校正使能  1使能  0 不使能
//	Magnetic_Anglex Yaw漂移校正 参数

//	输出全局：四元数（角度 ，弧度）

void IMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,int mx,int my,int mz,bool flag_compass) 
{
        float norm;
        float vx, vy, vz;
        float ex=0, ey=0, ez=0;   
		float xh,yh;
	float t11;
	float t12;
	float t13;
	float t21;
	float t22;
	float t23;
	float t31;
	float t32;
	float t33;
	

	int outx;
	int outy;
	int outz;

	int flag_ax=0,flag_ay=0,flag_az=0;



		if(abs(gz)<2/PI)
		{
			ezInt=0;
		}

        // 重力坐标系
		//矫正零偏
		ax=(ax-0.5)/9.85;
		ay=(ay-0.5)/9.85;
		az=(az+0.675)/10.025;

		if(ax<0) flag_ax=1;
		if(ay<0) flag_ay=1;
		if(az<0) flag_az=1;

		outx=(int)(ax*100.0f);
		outy=(int)(ay*100.0f);
		outz=(int)(az*100.0f);

        norm = sqrt(ax*ax + ay*ay + az*az);      
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;      
		
        // 角度坐标系
        vx = 2*(q1*q3 - q0*q2);
        vy = 2*(q0*q1 + q2*q3);
        vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        // 双坐标系误差校正
	//if(abs((ay*vz - az*vy)-exxx)>0.031)
        ex = (ay*vz - az*vy);//-exxx;
	//if(abs((az*vx - ax*vz)-eyyy)>0.031)
        ey = (az*vx - ax*vz);//-eyyy;
	//if(abs((ax*vy - ay*vx)-ezzz)>0.031)
        ez = (ax*vy - ay*vx);//-ezzz;
/*
#if 0  //误差冗余 自校正
		exx=exx+ex;
		eyy=eyy+ey;
		ezz=ezz+ez;

		cnt_e++;
		if(cnt_e>=30)
		{
			exxx=exx/31;
			eyyy=eyy/31;
			ezzz=ezz/31;
			exx=0;
			eyy=0;
			ezz=0;
			cnt_e=0;
		}
#endif
   */
        // 积分误差增益
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;
		// 在这加入检验机制

        // 根据误差 调整坐标系参数 
		
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;

        // 更新四元数
        q0r = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1r = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2r = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3r = q3 + (q0*gz + q1*gy - q2*gx)*halfT;

		q0=q0r;
		q1=q1r;
		q2=q2r;
		q3=q3r;
  
	//四元数规范化
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;

	//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	Rool = atan2(t23,t33);
	Pitch = -asin(t13);
	Yaw = atan2(t12,t11);
	if (Yaw < 0) Yaw += 360/180*PI;
	
	//--- 弧度转为角度
	Rool_angle = Rool/PI*180;
	Pitch_angle = Pitch/PI*180;
	Yaw_angle = Yaw/PI*180;

	//-----------------
	if(flagx)
	{   
		 mx=mx-61.0385;
		 my=my-218.5758;
		 mz=mz+82.8705;
		 printf("XW");
		 printf("%03d", Yaw_angle);
		 printf("%04d", Rool_angle);
		 printf("%04d", Pitch_angle);
		 printf("%04d",mx);
		 printf("%04d",my);
		 printf("%04d",mz);
		 printf("%d%d%d",flag_ax,flag_ay,flag_az);

//		 printf("%04d",(int)(ax*100));
//		 printf("%04d",(int)(ay*100));
//		 printf("%04d",(int)(az*100));

		 printf("%05d",outx);
		 printf("%05d",outy);
		 printf("%05d",outz);

		//	printf("& %03d\n", Yaw_angle);
	}

}

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass) 
{

  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;
  float tempq0,tempq1,tempq2,tempq3;
  	float t11;
	float t12;
	float t13;
	float t21;
	float t22;
	float t23;
	float t31;
	float t32;
	float t33;

	

  // 先把这些用得到的值算好
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;   
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;    

    //int mx_int=mx;
	//int my_int=my;
	//int mz_int=mz;

	int mx_int=mx-95.4753f;  //Company Blue
	int my_int=my-176.1358f;
	int mz_int=mz+23.2192f;

	//int mx_int=mx-60.2483f;  //Company
	//int my_int=my-218.7394f;
	//int mz_int=mz+78.3695f;

	//int mx_int=mx-57.8186;// HOME
	//int my_int=my-219.3857;
	//int mz_int=mz+47.5079;

	//int mx_int=mx-90.2641f;// HOME Blue
	//int my_int=my-160.1873f;
	//int mz_int=mz+5.1121f;

	 //-------------------
//	 mx=mx-95.4753f;
//	 my=my-176.1358f;
//	 mz=mz+23.2192f;
	 /************************/
	 key_num=KEY_Scan();
	 if(key_num==3)
	 {
	 	q0=1;q1=0;q2=0;q3=0;
	 }

	 if(abs(gz)<2.0f/PI)
		{
			ezInt=0;
		}

	// 重力坐标系
	//矫正零偏
//		ax=(ax-0.5)/9.85;
//		ay=(ay-0.5)/9.85;
//		az=(az+0.675)/10.025;

   // normalise the measurements
        norm = sqrt(ax*ax + ay*ay + az*az);       
        ax = ax / norm;
        ay = ay / norm;
        az = az / norm;
//        norm = sqrt(mx*mx + my*my + mz*mz);          
//        mx = mx / norm;
//        my = my / norm;
//        mz = mz / norm;         
//        
//        // compute reference direction of flux
//        hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
//        hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
//        hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);         
//        bx = sqrt((hx*hx) + (hy*hy));
//        bz = hz;        
        
        // estimated direction of gravity and flux (v and w)
        vx = 2*(q1q3 - q0q2);
        vy = 2*(q0q1 + q2q3);
        vz = q0q0 - q1q1 - q2q2 + q3q3;
		//vz=2*q0q0 - 1.0f + 2*q3q3	;
//        wx = bx*(0.5 - q2q2 - q3*q3) + bz*(q1q3 - q0q2);
//        wy = bx*(q1q2 - q0q3) + bz*(q0q1 + q2q3);
//        wz = bx*(q0q2 + q1q3) + bz*(0.5 - q1q1 - q2q2);  

	//	halfvx = q1q3 - q0q2;
	//	halfvy = q0q1 + q2q3;
	//	halfvz = q0q0 - 0.5f + q3q3;
        
        
        // error is sum of cross product between reference direction of fields and direction measured by sensors
        ex = (ay*vz - az*vy);// + (my*wz - mz*wy);
        ey = (az*vx - ax*vz);// + (mz*wx - mx*wz);
        ez = (ax*vy - ay*vx);// + (mx*wy - my*wx);
        
        // integral error scaled integral gain

		
        exInt = exInt + ex*Ki;
        eyInt = eyInt + ey*Ki;
        ezInt = ezInt + ez*Ki;

	//	printf("&%d *%d ",(int)(gx*100000),(int)(ex*100000));
        
        // adjusted gyroscope measurements
        gx = gx + Kp*ex + exInt;
        gy = gy + Kp*ey + eyInt;
        gz = gz + Kp*ez + ezInt;
        
        // integrate quaternion rate and normalise
        q0r = q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
        q1r = q1 + (q0*gx + q2*gz - q3*gy)*halfT;
        q2r = q2 + (q0*gy - q1*gz + q3*gx)*halfT;
        q3r = q3 + (q0*gz + q1*gy - q2*gx)*halfT;  

		q0=q0r;q1=q1r;q2=q2r;q3=q3r;
        
        // normalise quaternion
        norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
        q0 = q0 / norm;
        q1 = q1 / norm;
        q2 = q2 / norm;
        q3 = q3 / norm;

	//更新方向余弦矩阵
//	t11=q0*q0+q1*q1-q2*q2-q3*q3;
//	t12=2.0*(q1*q2+q0*q3);
//	t13=2.0*(q1*q3-q0*q2);
//	t21=2.0*(q1*q2-q0*q3);
//	t22=q0*q0-q1*q1+q2*q2-q3*q3;
//	t23=2.0*(q2*q3+q0*q1);
//	t31=2.0*(q1*q3+q0*q2);
//	t32=2.0*(q2*q3-q0*q1);
//	t33=q0*q0-q1*q1-q2*q2+q3*q3;
//
//	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
//	Rool = atan2(t23,t33);
//	Pitch = -asin(t13);
////	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
//	{
//	Yaw = atan2(t12,t11);
//
//	if (Yaw < 0) Yaw += 360/180*PI;
//	}
//	//Yaw += 360/180*PI;
////	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;
//
//	//--- 弧度转为角度
//	Rool_angle = Rool/PI*180;
//	Pitch_angle = Pitch/PI*180;
//	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%06d", (int)(q0*10000));
		 printf("%06d", (int)(q1*10000));
		 printf("%06d", (int)(q2*10000));
		 printf("%06d", (int)(q3*10000));

		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

		 printf("%04d",(int)(gx*100));
		 printf("%04d",(int)(gy*100));
		 printf("%04d",(int)(gz*100));

		// printf("%04d",(int)(ax*1000));
		// printf("%04d",(int)(ay*1000));
		// printf("%04d",(int)(az*1000));
//


		 
		 
//
//		 printf("%05d",(int)(hx*1000));
//		 printf("%05d",(int)(hy*1000));
//		 printf("%05d",(int)(hz*1000));

		
	}
}

// Fast inverse square-root
/**************************实现函数********************************************
*函数原型:	   float invSqrt(float x)
*功　　能:	   快速计算 1/Sqrt(x) 	
输入参数： 要计算的值
输出参数： 结果
*******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

/**************************实现函数********************************************
*函数原型:	   void IMU_AHRSupdate
*功　　能:	 更新AHRS 更新四元数 
输入参数： 当前的测量值。
输出参数：没有
*******************************************************************************/

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)
{
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;

	float t11,t12,t13,t21,t22,t23,t31,t32,t33;

	int mx_int=mx;
	int my_int=my;
	int mz_int=mz;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1 * q3 - q0 * q2;
		halfvy = q0 * q1 + q2 * q3;
		halfvz = q0 * q0 - 0.5f + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			integralFBx += twoKi * halfex * (2.0f*halfT);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (2.0f*halfT);
			integralFBz += twoKi * halfez * (2.0f*halfT);
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	// Integrate rate of change of quaternion
	gx *= halfT;		// pre-multiply common factors
	gy *= halfT;
	gz *= halfT;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

			//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	Rool = atan2(t23,t33);
	Pitch = -asin(t13);
//	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
	{
	Yaw = atan2(t12,t11);

	if (Yaw < 0) Yaw += 360/180*PI;
	}
	//Yaw += 360/180*PI;
//	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;

	//--- 弧度转为角度
	Rool_angle = Rool/PI*180;
	Pitch_angle = Pitch/PI*180;
	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%03d", Yaw_angle);
		 printf("%04d", Rool_angle);
		 printf("%04d", Pitch_angle);
		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

	//	printf("& %03d\n", Yaw_angle);
	}
}



void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)
{
	float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;  
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;

	int mx_int=mx-61.0385;
	int my_int=my-218.5758;
	int mz_int=mz+82.8705;
	
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
	{
		 mx=mx-61.0385;
		 my=my-218.5758;
		 mz=mz+82.8705;
		 MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az,halfT,flag_control,Magnetic_Anglex,flagx ,(int)mx, (int)my, (int)mz, flag_compass) ;
	
		return;
	}

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
	{

		 mx=mx-61.0385;
		 my=my-218.5758;
		 mz=mz+82.8705;
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;     

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;   

        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;   

        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);  
	
		// Error is sum of cross product between estimated direction and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) 
		{

			integralFBx += twoKi * halfex * (2.0f*halfT);	// integral error scaled by Ki
			integralFBy += twoKi * halfey * (2.0f*halfT);
			integralFBz += twoKi * halfez * (2.0f*halfT);
			//gx += integralFBx;	// apply integral feedback
			//gy += integralFBy;
			//gz += integralFBz;
		}
		else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKp * halfex;
		gy += twoKp * halfey;
		gz += twoKp * halfez;
	}
	
	
	// Integrate rate of change of quaternion
	gx *=  halfT;		// pre-multiply common factors
	gy *= halfT;
	gz *= halfT;
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx); 
	
	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;


		//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	Rool = atan2(t23,t33);
	Pitch = -asin(t13);
//	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
	{
	Yaw = atan2(t12,t11);

	if (Yaw < 0) Yaw += 360/180*PI;
	}
	//Yaw += 360/180*PI;
//	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;

	//--- 弧度转为角度
	Rool_angle = Rool/PI*180;
	Pitch_angle = Pitch/PI*180;
	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%03d", Yaw_angle);
		 printf("%04d", Rool_angle);
		 printf("%04d", Pitch_angle);
		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

	//	printf("& %03d\n", Yaw_angle);
	}
}
/********************************************************************************************************/
//---------------------------------------------------------------------------------------------------
// MadgwickAHRSupdate algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;

	int mx_int=mx-61.0385;
	int my_int=my-218.5758;
	int mz_int=mz+82.8705;

	//float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3; 
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) 
	{
		 mx=mx-61.0385;
		 my=my-218.5758;
		 mz=mz+82.8705;
		MadgwickAHRSupdateIMU(gx,gy,gz,ax,ay,az,halfT,flag_control,Magnetic_Anglex,flagx ,mx,my,mz,flag_compass);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
		 mx=mx-61.0385;
		 my=my-218.5758;
		 mz=mz+82.8705;
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (2.0f*halfT);
	q1 += qDot2 * (2.0f*halfT);
	q2 += qDot3 * (2.0f*halfT);
	q3 += qDot4 * (2.0f*halfT);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

		//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	Rool = atan2(t23,t33);
	Pitch = -asin(t13);
//	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
	{
	Yaw = atan2(t12,t11);

	if (Yaw < 0) Yaw += 360/180*PI;
	}
	//Yaw += 360/180*PI;
//	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;

	//--- 弧度转为角度
	Rool_angle = Rool/PI*180;
	Pitch_angle = Pitch/PI*180;
	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%03d", Yaw_angle);
		 printf("%04d", Rool_angle);
		 printf("%04d", Pitch_angle);
		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

	//	printf("& %03d\n", Yaw_angle);
	}
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az,float halfT,
         bool flag_control,int Magnetic_Anglex,bool flagx ,float mx,float my,float mz,bool flag_compass)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float t11,t12,t13,t21,t22,t23,t31,t32,t33;

	int mx_int=mx;
	int my_int=my;
	int mz_int=mz;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * (2.0f*halfT);
	q1 += qDot2 * (2.0f*halfT);
	q2 += qDot3 * (2.0f*halfT);
	q3 += qDot4 * (2.0f*halfT);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;

		//更新方向余弦矩阵
	t11=q0*q0+q1*q1-q2*q2-q3*q3;
	t12=2.0*(q1*q2+q0*q3);
	t13=2.0*(q1*q3-q0*q2);
	t21=2.0*(q1*q2-q0*q3);
	t22=q0*q0-q1*q1+q2*q2-q3*q3;
	t23=2.0*(q2*q3+q0*q1);
	t31=2.0*(q1*q3+q0*q2);
	t32=2.0*(q2*q3-q0*q1);
	t33=q0*q0-q1*q1-q2*q2+q3*q3;

	//------ 偏航角——Yaw,俯仰角——Pitch,翻滚角——Rool  （弧度）
	Rool = atan2(t23,t33);
	Pitch = -asin(t13);
//	if(abs(gx)<1/3.1159 && abs(gy)<1/3.1159 && abs(gz)<1/3.1159)
	{
	Yaw = atan2(t12,t11);

	if (Yaw < 0) Yaw += 360/180*PI;
	}
	//Yaw += 360/180*PI;
//	else	if (Yaw > 2*PI) Yaw = Yaw-360/180*PI;

	//--- 弧度转为角度
	Rool_angle = Rool/PI*180;
	Pitch_angle = Pitch/PI*180;
	Yaw_angle = Yaw/PI*180;

	if(flagx)
	{   
		 printf("XW");
		 printf("%03d", Yaw_angle);
		 printf("%04d", Rool_angle);
		 printf("%04d", Pitch_angle);
		 printf("%04d",mx_int);
		 printf("%04d",my_int);
		 printf("%04d",mz_int);

	//	printf("& %03d\n", Yaw_angle);
	}
}


