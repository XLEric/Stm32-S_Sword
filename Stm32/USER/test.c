#include <stm32f10x_lib.h>
#include "sys.h"
#include "usart.h"		
#include "delay.h"	
#include "led.h" 
#include "key.h"
#include "exti.h"
#include "wdg.h"
#include "timer.h"
#include "lcd.h"	   
#include "rtc.h"
#include "wkup.h"
#include "adc.h"
#include "dma.h"
#include "24cxx.h"
#include "flash.h"
#include "touch.h"
#include "24l01.h"
#include "myiic.h"
#include "imu.h"
#include "math.h"
//Mini STM32�����巶������19
//����ͨ��ʵ��
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//****************************************
// ����MPU9250�ڲ���ַ
//****************************************
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG1			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ졢������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)
#define	ACCEL_XOUT_H	0x3B
#define	ACCEL_XOUT_L	0x3C
#define	ACCEL_YOUT_H	0x3D
#define	ACCEL_YOUT_L	0x3E
#define	ACCEL_ZOUT_H	0x3F
#define	ACCEL_ZOUT_L	0x40
#define	TEMP_OUT_H		0x41
#define	TEMP_OUT_L		0x42
#define	GYRO_XOUT_H		0x43
#define	GYRO_XOUT_L		0x44	
#define	GYRO_YOUT_H		0x45
#define	GYRO_YOUT_L		0x46
#define	GYRO_ZOUT_H		0x47
#define	GYRO_ZOUT_L		0x48
#define	USER_CTRL		0x6A

#define MAG_XOUT_L		0x03
#define MAG_XOUT_H		0x04
#define MAG_YOUT_L		0x05
#define MAG_YOUT_H		0x06
#define MAG_ZOUT_L		0x07
#define MAG_ZOUT_H		0x08

#define MAG_ADDRESS    0x18   //�ų���ַ


#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)
#define	SlaveAddress	0xD0	//IICд��ʱ�ĵ�ַ�ֽ����ݣ�+1Ϊ��ȡ

#define	SlaveAddressH   0x3C	  //��������5883��IIC�����еĴӵ�ַ
#define Kp 2.01    //�������                  
#define Ki 0.003      

typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;

uchar aaa,bbb,ccc,ddd;
unsigned short time_us,time_ms;

void InitMPU9250()
{
	NRF24L01_Write_Reg(PWR_MGMT_1, 0x00);	//�������״̬
	NRF24L01_Write_Reg(SMPLRT_DIV, 0x07);
	NRF24L01_Write_Reg(CONFIG1, 0x06);
	NRF24L01_Write_Reg(GYRO_CONFIG, 0x18);
	NRF24L01_Write_Reg(ACCEL_CONFIG, 0x04);
	NRF24L01_Write_Reg(CONFIG1, 0x01); //���ٶȵ�ͨ�˲�Ƶ��
	NRF24L01_Write_Reg(0x1D, 0x05);//���ٶ��˲���

	AT24CXX_Init();		//�ų������� IIC��ʼ��
}
																		   
//�շ�������һ����������,ͨ��������ȷ�����뷢��ģʽ,���ǽ���ģʽ		    									    		    	 		  
int main(void)
{	
	u8 key,mode;
	u16 t=0;			 
	u8 tmp_buf[33];
	u8 i=0;
	uchar device_ID=0;
	unsigned char H_x,L_x,H_y,L_y,H_z,L_z;
	bool  flag_compass=0;
	
//	unsigned char HH;
	short value_x,value_y,value_z;
	short value_gx,value_gy,value_gz;
	

	float value_xf,value_yf,value_zf;
	float value_gxf,value_gyf,value_gzf;
	float value_gxf2,value_gyf2,value_gzf2;
	float value_gxfs,value_gyfs,value_gzfs;
	float mx,my,mz;
	float half_time;

	float Anglef;
	short Angle;
	float norm;

	u8 len;
	u8 command[2]={0};

  	Stm32_Clock_Init(9);//ϵͳʱ������
	delay_init(72);		//��ʱ��ʼ��
	uart_init(72,19200); //����1��ʼ��  	  

	LED_Init();         //LED��ʼ��
	KEY_Init();
	NRF24L01_Init();    //��ʼ��NRF24L01 

	InitMPU9250();
	NRF24L01_Write_Reg(USER_CTRL, 0x10);

	Element_4_int(1,0,9.8,0);

	for(i=0;i<20;i++)
	{
		IIC_WriteOneByte(MAG_ADDRESS,0x0A,0x12);
   		delay_ms(10);
	}

	while(1)
	{
	//	device_ID=NRF24L01_Read_Reg(0xF5);

			H_x=NRF24L01_Read_Reg(0xBB);
			L_x=NRF24L01_Read_Reg(0xBC);
			value_x=(H_x<<8)+L_x;
			value_xf=(float)(value_x)/1.6384*0.00098;

			H_y=NRF24L01_Read_Reg(0xBD);
			L_y=NRF24L01_Read_Reg(0xBE);
			value_y=(H_y<<8)+L_y;
			value_yf=(float)(value_y)/1.6384*0.00098;
			
			H_z=NRF24L01_Read_Reg(0xBF);
			L_z=NRF24L01_Read_Reg(0xC0);
			value_z=(H_z<<8)|L_z;
			value_zf=(float)(value_z)/1.6384*0.00098;

		  // printf("%d  %d  %d \n",value_x,value_y,value_z);
		  // printf("%d\n",device_ID);
			
			//delay_ms(5);

			H_x=NRF24L01_Read_Reg(0xC3);
			L_x=NRF24L01_Read_Reg(0xC4);
			value_gx=(H_x<<8)+L_x;
			value_gx=(value_gx)/164;
			if(abs(value_gx)<2)value_gx=0;
			value_gxf=(float)(value_gx)*10*3.14159/180;

			H_y=NRF24L01_Read_Reg(0xC5);
			L_y=NRF24L01_Read_Reg(0xC6);
			value_gy=(H_y<<8)+L_y;
			value_gy=(value_gy)/164;
			if(abs(value_gy)<2)value_gy=0;
			value_gyf=(float)(value_gy)*10*3.14159/180;

			H_z=NRF24L01_Read_Reg(0xC7);
			L_z=NRF24L01_Read_Reg(0xC8);
			value_gz=(H_z<<8)+L_z;
			value_gz=(value_gz)/164;
			if(abs(value_gz)<2)value_gz=0;
			value_gzf=(float)(value_gz)*10*3.14159/180;
	  // printf("%d  %d  %d \n",value_gx,value_gy,value_gz);

		
#if 1
		IIC_WriteOneByte(GYRO_ADDRESS,0x37,0x02);// on Bypass Mode 
   		//delay_ms(2);
	
   		IIC_WriteOneByte(MAG_ADDRESS,0x0A,0x12);
   		//delay_ms(2);

		L_x=IIC_ReadOneByte(MAG_ADDRESS,0x03);
	 	H_x=IIC_ReadOneByte(MAG_ADDRESS,0x04);
	  	value_gx =(H_x<<8) +L_x;

	  	L_y=IIC_ReadOneByte(MAG_ADDRESS,0x05);
	 	H_y=IIC_ReadOneByte(MAG_ADDRESS,0x06);
	  	value_gy =(H_y<<8) +L_y;
		//value_gy=value_gy;

	  	L_z=IIC_ReadOneByte(MAG_ADDRESS,0x07);
	 	H_z=IIC_ReadOneByte(MAG_ADDRESS,0x08);
	  	value_gz =(H_z<<8) +L_z;

//		Anglef=atan2(value_gx,value_gy);
//		Anglef=((float)(Anglef)* (180 / 3.14159));
//		Angle=(int)(Anglef)+180;
//		Angle=360-Angle;
		//printf("A- %d (%d, %d, %d)\n",Angle,value_gx,value_gy,value_gz);
		//printf("(%d, %d, %d)\n",value_gx,value_gy,value_gz);
#endif
	  	/*****************************************/
		TIM3->CR1&=0<<0;    //��ʹ�ܶ�ʱ��3
		time_us=TIM3->CNT;	//����ʱ
		time_ms=TIM3_IRQHandler();
		 
	  //printf("ms =  %d us =%d\n",time_ms,time_us);
		 	
	   Timerx_Init(1000,71);
	   //�������ʱ��
	   half_time=(float)(time_us)/1000000;
	   half_time=half_time+(float)(time_ms)/1000;
	   half_time=half_time/2.0f;
		
	   //printf("---half_time =  %f \n",half_time); 
	   

	   /********************************************/
			   //flagw=1;
		if(USART_RX_STA&0x80)
		{					   
			len=USART_RX_STA&0x3f;//�õ��˴ν��յ������ݳ���
			if(len==2)
			{
			
			//printf("\n�����͵���ϢΪ:\n");
			for(t=0;t<len;t++)
			{
				command[t]=	USART_RX_BUF[t];
				
			}
			//flagw=1;
					
					 if(command[0]==0x43)
					 {
						switch(command[1])
						{
						  case 0xa1:flagw=1;break	;
						  //case 0xa1:printf("%d",10);break	;
						  //case 0xa2:printf("%d",20);break	;
						  //case 0xa3:printf("%d",30);break	;
						  //case 0xa4:printf("%d",40);break	;
						  //case 0xa5:printf("%d",50);break  ;
						  //case 0xa6:printf("%d",60);break  ;
						  default:break;// printf("%d",100);break  ;
						}
					 }
			}
				USART_RX_STA=0;
		}

				  
	   /********************************************/
	   //ƽ��ֵ
//	   value_gxfs=(value_gxf2+value_gxf)/2;
//	   value_gyfs=(value_gyf2+value_gyf)/2;
//	   value_gzfs=(value_gzf2+value_gzf)/2;

	   //����ƽ��ֵ
	   value_gxfs=value_gxf;
	   value_gyfs=value_gyf;
	   value_gzfs=value_gzf;

	   //norm=value_xf*value_xf+value_yf*value_yf+value_zf*value_zf;
	   //norm=sqrt(norm);

	   if((value_xf!=0 |value_yf!=0|value_zf!=0) && (value_gx!=0 | value_gy!=0 | value_gz!=0))
	   {
	   		
	   	   // IMU(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,value_gx,value_gy,value_gz, flag_compass) ;
	   		mx=(float)(value_gx);//-61.0385;
	   		my=(float)(value_gy);//- 218.5758  ;
	   		mz=(float)(value_gz);//+82.8705;

	   //	MahonyAHRSupdate(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,mx,my,mz, flag_compass);
	   //MadgwickAHRSupdate(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,mx,my,mz, flag_compass);
	 	
		//delay_ms(5);
		//IMU_AHRSupdate(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,mx,my,mz, flag_compass);
		IMU_AHRSupdateComPass(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,mx,my,mz, flag_compass);
	    //AHRS_AHRSupdate_K(value_gxfs,value_gyfs,value_gzfs, value_xf,value_yf,value_zf,half_time,0,Angle,flagw,mx,my,mz, flag_compass);
	   	
	   
	   }
	   
	   /*
	   else
	   printf("erron!\n");
	   */
	   flagw=0;

	   value_gxf2=value_gxf;
	   value_gyf2=value_gyf;
	   value_gzf2=value_gzf;
	  
	//	LED0=!LED0;//DS0��˸
	}
	
/*************************************************************************************************/				 
/*	while(NRF24L01_Check())//��ⲻ��24L01
	{
		LCD_ShowString(60,130,"24L01 Check Failed!");
		delay_ms(500);
		LCD_ShowString(60,130,"Please Check!      ");
		delay_ms(500);
		LED0=!LED0;//DS0��˸
	}
	LCD_ShowString(60,130,"24L01 Ready!");
	LCD_ShowString(10,150,"KEY0:RX_Mode  KEY1:TX_Mode");


	while(1)//�ڸò���ȷ�������ĸ�ģʽ!
	{
		key=KEY_Scan();
		if(key==1)
		{
			mode=0;   
			break;
		}else if(key==2)
		{
			mode=1;
			break;
		}
		t++;
		if(t==100) //��˸��ʾ��ʾ��Ϣ
		{
			LCD_ShowString(10,150,"                          ");//�����ʾ  
		}	
		if(t==200)
		{
			t=0;
			LCD_ShowString(10,150,"KEY0:RX_Mode  KEY1:TX_Mode");
		}
		delay_ms(5);

	}
	LCD_Fill(10,150,240,166,WHITE);//����������ʾ		  
 	POINT_COLOR=BLUE;//��������Ϊ��ɫ	   
	if(mode==0)//RXģʽ
	{
		LCD_ShowString(60,150,"NRF24L01 RX_Mode");	
		LCD_ShowString(60,170,"Received DATA:");	
		RX_Mode();		  
		while(1)
		{	  		    		    				 
			if(NRF24L01_RxPacket(tmp_buf)==0)//һ�����յ���Ϣ,����ʾ����.
			{
				tmp_buf[32]=0;//�����ַ���������
				LCD_ShowString(0,190,tmp_buf); 
			}else delay_us(100);	   
			t++;
			if(t==10000)//��Լ1s�Ӹı�һ��״̬
			{
				t=0;
				LED0=!LED0;
			} 				    
		};	
	}else//TXģʽ
	{							    
		LCD_ShowString(60,150,"NRF24L01 TX_Mode");	
		TX_Mode();
		mode=' ';//�ӿո����ʼ  
		while(1)
		{	  		   				 
			if(NRF24L01_TxPacket(tmp_buf)==TX_OK)
			{
				LCD_ShowString(60,170,"Sended DATA:");	
				LCD_ShowString(0,190,tmp_buf); 
				key=mode;
				for(t=0;t<32;t++)
				{
					key++;
					if(key>('~'))key=' ';
					tmp_buf[t]=key;	
				}
				mode++; 
				if(mode>'~')mode=' ';  	  
				tmp_buf[32]=0;//���������		   
			}else
			{										   	
 				LCD_ShowString(60,170,"Send Failed "); 
				LCD_Fill(0,188,240,218,WHITE);//����������ʾ			   
			};
			LED0=!LED0;
			delay_ms(1500);				    
		};
	}     */	  
}


				 






