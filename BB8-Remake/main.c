/**
* @par Copyright (C): 2018-2028, DarrenPig
* @file         // main.c
* @author       // lly
* @version      // V1.0
* @date         // 250726
* @brief        // ������� Program entry
* @details      
* @par History  // �޸���ʷ��¼�б�ÿ���޸ļ�¼Ӧ�����޸����ڡ��޸��߼�
*               // �޸����ݼ���  Modification history list, each modification record should include the modification date, modifier and a brief description of the modification content
*/ 

#include "AllHeader.h"
#include "intsever.h"
//ע��:������������ʱ��Ҫ�ж��Ƿ���������ѹ
//Attention: When operating the buzzer, check if it is at normal voltage

uint8_t GET_Angle_Way=2;                             //��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�  //Algorithm for obtaining angles, 1: Quaternion 2: Kalman 3: Complementary filtering
float Angle_Balance,Gyro_Balance,Gyro_Turn;     		//ƽ����� ƽ�������� ת�������� //Balance tilt angle balance gyroscope steering gyroscope
int Motor_Left,Motor_Right;                 	  		//���PWM���� //Motor PWM variable
int Temperature;                                		//�¶ȱ��� 		//Temperature variable
float Acceleration_Z;                           		//Z����ٶȼ�  //Z-axis accelerometer
int Mid_Angle;                          						//��е��ֵ  //Mechanical median
float Move_X,Move_Z; //Move_X:ǰ���ٶ�  Move_Z��ת���ٶ�  //Move_X: Forward speed Move_Z: Steering speed
u8 Stop_Flag = 1; //0:��ʼ 1:ֹͣ  //0: Start 1: Stop


char showbuf[20]={'\0'};
u8* CCDShowBuf = NULL;

extern u8 newLineReceived;//�������� //Bluetooth reception
extern u8 g_lidar_go_flag;
extern u8 bulettohflag;

int main(void)
{	
		
	bsp_init();//���������ʼ�� //Basic peripheral initialization
	
	Mode_select(); //���°�������ģʽѡ�� //Press the button to end mode selection
	
	bsp_mode_init();//����ģʽ��ʼ����չ���� //Initialize and expand peripherals based on the pattern
	
	
	MPU6050_EXTI_Init();		//���жϷ������ŵ����  //This interrupt service function is placed last
	
	
	OLED_Draw_Line("put down key start!", 2, false, true); 
	
	while(!Key1_State(1) && Stop_Flag ==1 );
	Stop_Flag = 0; //��ʼ����  //Start controlling

	
	OLED_Draw_Line("start control!        ", 2, false, true); 
	


	while(1)
	{
		if(mode >= LiDar_avoid)//�����״�ģʽ ���յ����� //Activate radar mode to receive data
		{
				if(lidar_new_pack == 1)  //����������״�����,�������жϽ���  //Put it here to solve radar data, do not interrupt the calculation
				{
					lidar_new_pack = 0;
					Deal_lidardata();
				}
		}
		

		if(mode == Normal || mode == Weight_M)//����ģʽ������ģʽ  //Normal mode, load mode
		{
			if (newLineReceived) //����ң�ط���  //Bluetooth remote control service
			{
				ProtocolCpyData();
				Protocol();
			}
			if(bulettohflag == 1) 
			{
				bulettohflag = 0;
				SendAutoUp();//�����Զ��ϱ����� Bluetooth automatically reports data 
			}
					
			sprintf(showbuf,"dis =%d mm   ",g_distance);
			OLED_Draw_Line(showbuf, 3, false, true); 
		}
		
		else if(mode == U_Follow) //����������ģʽ Ultrasonic follow mode
		{
			sprintf(showbuf,"dis =%d mm   ",g_distance);
			OLED_Draw_Line(showbuf, 3, false, true); 
		}
		
		else if(mode == U_Avoid) //����ģʽ  //Obstacle avoidance mode
		{
			APP_avoid();
		}
		
		else if(mode == PS2_Control) //PS2����ģʽ   //PS2 control mode
		{
			sprintf(showbuf,"speed = %d  ",speed_flag);
			OLED_Draw_Line(showbuf, 3, false, true); 
//			PS2_Contorl_Car(); //��ƽ���жϷ������� �ӿ���Ӧ In the balance interrupt service processing to speed up the response
		}
		
		else if(mode == Line_Track || mode == Diff_Line_track) //4·Ѳ��ģʽ  //4-way patrol mode
		{
			sprintf(showbuf,"x1 = %d  x2 = %d    ",IN_X1,IN_X2);
			OLED_Draw_Line(showbuf, 2, false, true); 
			sprintf(showbuf,"x3 = %d  x4 = %d    ",IN_X3,IN_X4);
			OLED_Draw_Line(showbuf, 3, false, true); 
		}
		
		else if(mode == CCD_Mode) //CCDѲ��ģʽ  //CCD patrol mode
		{
			OLED_Show_CCD_Image(CCDShowBuf);//CCD��ʾ //CCD display
		}
		
		else if(mode == ElE_Mode) //���Ѳ��ģʽ //Electromagnetic patrol mode
		{
			EleDataDeal();//��ʾ���  //Display results
		}
		
		else if(mode == K210_QR) //ʶ���ά��ģʽ  //Identify QR code patterns
		{
			Change_state_QR();//ʶ��  //Identify
		}
		else if(mode == K210_SelfLearn) //����ѧϰģʽ //Self directed learning mode
		{
			Change_state_self();
		}
		else if(mode == K210_mnist) //ʶ������ģʽ  //Identify numerical patterns
		{
			Change_state_minst();
		}
		
		else if(mode == LiDar_avoid)//�״����ģʽ  //Radar obstacle avoidance mode
		{
			Car_Avoid();//����  //Obstacle avoidance
		}
		else if(mode == LiDar_Follow)//�״����ģʽ  //Radar Follow Mode
		{
			Car_Follow();//���� //Follow me
		}
		else if(mode == LiDar_aralm)//�״ﾯ��ģʽ  //Radar security mode
		{
			Car_Alarm();//����  //Security Guard
		}
		else if(mode == LiDar_Patrol)//�״�Ѳ��ģʽ  //Radar patrol mode
		{
			Car_Patrol();//Ѳ�� //Patrol
		}
		else if(mode == LiDar_Line)//�״�ֱ��ģʽ  //Radar linear mode
		{
			if(g_lidar_go_flag == 1) //ֻ����һ��  //Operate only once
			{
				g_lidar_go_flag = 0;
				OLED_Draw_Line("start track!     ", 3, false, true);
			}
		}
		
		
		
	}
}


