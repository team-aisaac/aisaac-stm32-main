/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  *
  *20190909 git sharing test
  *
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
/* #include "main.h" */

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "SEGGER_RTT.h"
#include "string.h"
#include "sd_hal_mpu6050.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
//#define M_PI 3.1415926535
#ifndef M_PI
#define M_PI 3.1415926535
#endif
#define TIM_CLOCK 72000000;

//#define TODAY_CHARGE_KICK_ENABLE	// for charge-kick enable robot

//#define BALL_SENSOR_ENABLE		// enable when robot want to kick without sensor
//#define DRIBBLER_ENABLE			// enable when robot doesn't use dribbler

#define MD_CHECK_MODE				// enable to be Motor-Driver check mode. Motors on the robot move forward 1[s], backward 1[s], move left 1[s], move right 1[s]

float rad_coef=M_PI/180.0;

char debug_str[100];
/*CAN*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t RxData[8]={0,0,0,0,0,0,0,0};
uint32_t TxMailbox=0;

CAN_FilterTypeDef sFilterConfig;
/*CAN*/

/*gyro I2C*/
bool I2C_check=0;
uint8_t aRxBuffer[14] = {0};
/*gyro I2C*/

/*kicker*/
bool kicker_charge_state=0;
/*kicker*/

uint32_t debug_change_count=0;
uint8_t dir=0;

uint16_t send_data1=0,send_data2=0;

/*For machine angle PID control*/
/* Control theorem: output=(theta_tar-theta_now)*K_p_theta */
float K_p_theta=1.5;
float theta_tar=0, theta_now=0;	//Target angle, Now angle
const float D_machine=176;	//Machine diameter: unit[mm]
float D_wheel=52;		//Wheel diameter: unit[mm]

float error_theta=0;
//float c_Vr=0;	//Yaw rotation speed: unit[mm/s]

/*For motor output*/
int16_t motor_out1=0,motor_out2=0,motor_out3=0,motor_out4=0;	//motor speed 0~4095
uint16_t m_data1=0,m_data2=0,m_data3=0,m_data4=0;	//motor speed 0~4095
float coef1=0, coef2=0, coef3=0;	//Convert constant: xy vector -> wheel rotation speed
const float motor_coef=3.2*60/52/M_PI*4095/4000;	//motor rotation speed convertion constant [mm/s] -> 0~4095
float Vx=0,Vy=0,Theta_tar=0;	//Target speed,angle,x,y,yaw speed, unit:[mm/s],[deg]
float Vr=0;	//縲???��?��??��?��逶???��?��??��?��讓呵???��?��??��?��帝�溷???��?��??��?��???��?��??��?��[deg/s]
int16_t motor_output=0;

/*Gyro*/
SD_MPU6050 mpu1;
SD_MPU6050_Result result;
float g_x,g_y,g_z;
float a_x,a_y,a_z;
float z_axis_deg=0;
int tmp=0;
int ticked_time=0;
float ticker_interval=1000;
float g_z_drift_ave=0, g_z_drift_sum=0;
float g_z_drift_cor=0.145;	//Gyro drift correction value
float g_z_pre=0, g_z_pre_pre=0;
bool gyro_calib_flag=1;

/*Ball sensor*/
int ball_existance_count=0;
bool ball_existance=0;
const int ball_th=1000;	//threshold
/*Dribbler*/
TIM_MasterConfigTypeDef sMasterConfig;
TIM_OC_InitTypeDef sConfigOC;

bool sw1_state=0;

char rxdata[10];

int a=0;

// for xbee
const char HEAD_BYTE=0x7D;    // use as header byte
const char ESCAPE_BYTE=0x7E;  // use as escape byte
const char ESCAPE_MASK=0x20;  // use as escape mask; see reference ->  https://qiita.com/hideakitai/items/347985528656be03b620


uint8_t data[17]={0x7D};         // queue; received data is stored in this array
uint16_t send_data[17]={0};    // send_data available when receiving
uint8_t receive_data[1]={0};  // receiving buffer; from xbee module
char debug[100]={0};       // transmitting buffer; to PC
bool available = false;       // when received data set moves from data[17] to send_data[17], this flag change to true
bool data_valid=false;        // when available changes false to true, use checksum byte to validate received data
int irq_count=0;              // count how many times irq function is called

int16_t x_vector=0,y_vector=0,th_vector=0;
float x_vector_f=0,y_vector_f=0,th_vector_f=0;
int16_t calib_data=0;
int16_t target_th_vector=0;

#define DISABLE 0
#define ENABLE 1
#define FORCE_KICK 2
#define KICKER_FINISH 3

bool finish_charging_flag=false;

enum enum_kicker_state
{
	off=0x20,
	shortpass=0x10,
	middlepass=0x8,
	longpass=0x4,
	shoot=0x2,
	clear=0x1,
};

enum enum_kick_type
{
	tip=0,
	straight=1,
};


int16_t command=0;
int sensor_enable=DISABLE;
//bool kicker_enable=DISABLE;
int kick_type=tip;
int kicker_state=off;

int ary_index=1;
uint8_t prev_receive_data=0;
int valid_count=0;

bool some=false;

int ms_counter=0,s_counter=0;
float elapsed_time=0;

int xbee_timeout=500;		//[ms]
bool xbee_available=true;
int xbee_session_time=0;	//[ms]

// for xbee



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/*
 void calc_out(float c_Vx,float c_Vy,float c_Vr) {

	motor_out1=(int16_t)(coef2*c_Vx+coef1*c_Vy+c_Vr);
	motor_out2=(int16_t)(coef2*c_Vx-coef1*c_Vy+c_Vr);
	motor_out3=(int16_t)(-coef3*c_Vx-coef3*c_Vy+c_Vr);
	motor_out4=(int16_t)(-coef3*c_Vx+coef3*c_Vy+c_Vr);

	if(motor_out1>=0) {
		m_data1=motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
	} else {
		m_data1=-motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
		TxData[0]=TxData[0]|0x10;
	}
	if(motor_out2>=0) {
		m_data2=motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
	} else {
		m_data2=-motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
		TxData[2]=TxData[2]|0x10;
	}
	if(motor_out3>=0) {
		m_data3=motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
	} else {
		m_data3=-motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
		TxData[4]=TxData[4]|0x10;
	}
	if(motor_out4>=0) {
		m_data4=motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
	} else {
		m_data4=-motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
		TxData[6]=TxData[6]|0x10;
	}



}
~2019/8/8*/
void calc_out_old(float c_Vx, float c_Vy, float c_Vr){
	// 隗帝�溷???��?��??��?��???��?��??��?��蛻???��?��??��?��蠕｡逕ｨ縺???��?��??��?��縺???��?��??��?��繧???��?��??��?��繝｡繝ｳ繝医??��?��?繧???��?��??��?��繝�
	//蟋ｿ蜍｢隗単ID
//	theta_now=z_axis_deg;
//	error_theta=c_theta_tar-theta_now;
	//迚ｹ逡???��?��??��?��轤???��?��??��?��隗｣豸???��?��??��?��
//	if(error_theta>180) error_theta-=360;
//	else if(error_theta<-180) error_theta+=360;
//	if(fabs(error_theta)>3.0)c_Vr=error_theta*K_p_theta*D_machine*M_PI/360;

	//縺???��?��??��?��縺???��?��??��?��縺???��?��??��?��縺???��?��??��?��縺???��?��??��?��
	motor_out1=(int16_t)(coef2*c_Vx+coef1*c_Vy+c_Vr);
	motor_out2=(int16_t)(coef2*c_Vx-coef1*c_Vy+c_Vr);
	motor_out3=(int16_t)(-coef3*c_Vx-coef3*c_Vy+c_Vr);
	motor_out4=(int16_t)(-coef3*c_Vx+coef3*c_Vy+c_Vr);

	/*
	//??��?��???��?��??��?��縺励???��?��??��?��縺???��?��??��?��
	motor_out1=(int16_t)((coef1*c_Vx-coef2*c_Vy+c_Vr)*motor_coef);
	motor_out2=(int16_t)((-coef1*c_Vx-coef2*c_Vy+c_Vr)*motor_coef);
	motor_out3=(int16_t)((-coef3*c_Vx+coef3*c_Vy+c_Vr)*motor_coef);
	motor_out4=(int16_t)((coef3*c_Vx+coef3*c_Vy+c_Vr)*motor_coef);
	 */
	/*
	motor_out1=motor_output;
	motor_out2=motor_output;
	motor_out3=motor_output;
	motor_out4=motor_output;
	*/
	if(motor_out1>=0) {
		m_data1=motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
	} else {
		m_data1=-motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
		TxData[0]=TxData[0]|0x10;
	}
	if(motor_out2>=0) {
		m_data2=motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
	} else {
		m_data2=-motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
		TxData[2]=TxData[2]|0x10;
	}
	if(motor_out3>=0) {
		m_data3=motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
	} else {
		m_data3=-motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
		TxData[4]=TxData[4]|0x10;
	}
	if(motor_out4>=0) {
		m_data4=motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
	} else {
		m_data4=-motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
		TxData[6]=TxData[6]|0x10;
	}
}

void calc_out(float c_Vx,float c_Vy,float c_theta_tar) {
	//荳???��?��??��?��譎ら噪縺???��?��??��?��霑ｽ蜉�20190808豺???��?��??��?��螟� ??��?��???��?��??��?��豎�
	float c_Vr=0;
	//Machine angle PID
	theta_now=z_axis_deg;
	error_theta=c_theta_tar-theta_now;
	//Singular point cancellation
	if(error_theta>180) error_theta-=360;
	else if(error_theta<-180) error_theta+=360;
	if(fabs(error_theta)>3.0)c_Vr=error_theta*K_p_theta*D_machine*M_PI/360;

	//Previous
	/*motor_out1=(int16_t)(coef2*c_Vx+coef1*c_Vy+c_Vr);
	motor_out2=(int16_t)(coef2*c_Vx-coef1*c_Vy+c_Vr);
	motor_out3=(int16_t)(-coef3*c_Vx-coef3*c_Vy+c_Vr);
	motor_out4=(int16_t)(-coef3*c_Vx+coef3*c_Vy+c_Vr);*/

	//New
	motor_out1=(int16_t)((coef1*c_Vx-coef2*c_Vy+c_Vr)*motor_coef);
	motor_out2=(int16_t)((-coef1*c_Vx-coef2*c_Vy+c_Vr)*motor_coef);
	motor_out3=(int16_t)((-coef3*c_Vx+coef3*c_Vy+c_Vr)*motor_coef);
	motor_out4=(int16_t)((coef3*c_Vx+coef3*c_Vy+c_Vr)*motor_coef);

	if(motor_out1>=0) {
		m_data1=motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
	} else {
		m_data1=-motor_out1;
		TxData[0]=m_data1>>8;
		TxData[1]=m_data1&255;
		TxData[0]=TxData[0]|0x10;
	}
	if(motor_out2>=0) {
		m_data2=motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
	} else {
		m_data2=-motor_out2;
		TxData[2]=m_data2>>8;
		TxData[3]=m_data2&255;
		TxData[2]=TxData[2]|0x10;
	}
	if(motor_out3>=0) {
		m_data3=motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
	} else {
		m_data3=-motor_out3;
		TxData[4]=m_data3>>8;
		TxData[5]=m_data3&255;
		TxData[4]=TxData[4]|0x10;
	}
	if(motor_out4>=0) {
		m_data4=motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
	} else {
		m_data4=-motor_out4;
		TxData[6]=m_data4>>8;
		TxData[7]=m_data4&255;
		TxData[6]=TxData[6]|0x10;
	}
}

int ball_finder(){
	int ball=0;

	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1,10);
	//if(HAL_ADC_GetState(&hadc1)&HAL_ADC_STATE_EOC_REG){
		int sensor_data = HAL_ADC_GetValue(&hadc1);

		if(sensor_data<ball_th){
			ball=1;
		} else if(sensor_data>=ball_th){
			ball=0;
		}
		//HAL_Delay(100);
	//}
		sprintf(debug_str,"photo = %d, ball = %d\r\n",sensor_data,ball);
		SEGGER_RTT_Write(0, debug_str, strlen(debug_str));

	HAL_ADC_Stop(&hadc1);

	return ball;
}

void dribble_motor(int direction){
#ifdef DRIBBLER_ENABLE
	if(direction==1){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,80);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	} else if(direction==-1){
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,80);
	} else {
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
		__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
	}
#else
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_1,0);
	__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_2,0);
#endif
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int return_value=HAL_OK;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/*
 *
 if(htim->Instance == TIM1)
	{
		//TxData[0]=send_data1>>8;
		//TxData[1]=send_data1&255;
		//TxData[2]=send_data2>>8;
		//TxData[3]=send_data2&255;
		HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
	}
	~2019/8/8 */

	if(htim->Instance == TIM1)
	{
		/* CAN騾壻???��?��??��?��???��?��??��?��縺???��?��??��?��??��?��???��?��??��?��??��?��????��?��??��?�� */
		/*TxData[0]=send_data1>>8;
		TxData[1]=send_data1&255;
		TxData[2]=send_data2>>8;
		TxData[3]=send_data2&255;*/
//		calc_out(Vx,Vy,Theta_tar);
		calc_out_old(Vx,Vy,Vr);
		HAL_CAN_AddTxMessage(&hcan,&TxHeader,TxData,&TxMailbox);
		//a++;


//		if(sensor_enable==1){
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
//		}else{
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,RESET);
//		}

		/* 繝懊???��?��??��?��???��?��??��?��繝ｫ繧???��?��??��?��繝ｳ繧???��?��??��?��縺???��?��??��?��??��?��???��?��??��?��??��?��????��?��??��?�� */
		if(ball_finder()==1){
			ball_existance_count++;
		} else {
			ball_existance_count=0;
			ball_existance=0;
		}
		if(ball_existance_count>3){
			ball_existance=1;
		}
//		sprintf(debug_str,"ball_existance = %d\r\n",ball_existance);
//		SEGGER_RTT_Write(0, debug_str, strlen(debug_str));

		/* 繝峨Μ繝悶Λ繝ｼ縺???��?��??��?��蛻???��?��??��?��蠕｡ */
//		if(irq_count<=10 || (x_vector==0 && y_vector==0 && th_vector==0))dribble_motor(0);
//		else dribble_motor(1);
//		drible_motor(1);

		/* 繧???��?��??��?��繝｣繧???��?��??��?��繝ｭ縺???��?��??��?��??��?��???��?��??��?��??��?��????��?��??��?�� */
		/*Gyro Culculation*/
		if(ticked_time%2==0){
//			return_value=SD_MPU6050_ReadGyroscope(&hi2c2,&mpu1);
		}

		//譛�蛻???��?��??��?��10[s]縺???��?��??��?��繝峨Μ繝輔ヨ蛟､縺???��?��??��?��繧???��?��??��?��繝｣繝ｪ繝悶Ξ繝ｼ繧???��?��??��?��繝ｧ繝ｳ
		if(ticked_time < (10/ticker_interval) && gyro_calib_flag){
		   //ticked_time++;
		   g_z_drift_sum += mpu1.Gyroscope_Z * mpu1.Gyro_Mult;
		}
		//繝峨Μ繝輔ヨ譬???��?��??��?��??��?��???��?��??��?��蛟､縺???��?��??��?��譖ｸ縺肴鋤縺???��?��??��?��
		else if(ticked_time == (10/ticker_interval) && gyro_calib_flag) {
			//ticked_time++;
		   	g_z_drift_cor = g_z_drift_sum / ticked_time;
		   	gyro_calib_flag=false;
		}
		//繝ｨ繝ｼ隗�???��?��???��?��??��?��???��?��??��?��險育???��?��??��?��???��?��??��?��
		else {
			g_z = mpu1.Gyroscope_Z * mpu1.Gyro_Mult - g_z_drift_cor;
			z_axis_deg += (g_z + g_z_pre + g_z_pre_pre) / 3  * ticker_interval;
		    g_z_pre_pre = g_z_pre;
		    g_z_pre = g_z;
		    if(g_z>=360) g_z=g_z-360;
		    else if(g_z<0) g_z=g_z+360;
		}

		ticked_time++;
		if(ticked_time==100000) ticked_time=0;

		ms_counter++;
		if(ms_counter==1000){
			s_counter++;
			ms_counter=0;
		}
		elapsed_time=s_counter+(float)ms_counter/1000;
	}
}

/*
 * data set 0 (datatype=0, velocity vector)
 * | 0-7(8) | 8-10(3) | 11-23(13) | 24-36(13) | 37-48(12) | 49-55(7) | 56-63(8) |
 * |--------|---------|-----------|-----------|-----------|----------|----------|
 * |HEADER  |DATATYPE |X_VECTOR   |Y_VECTOR   |TH_VECTOR  |     0    |CHECKSUM  |
 * |--------|---------|-----------|-----------|-----------|----------|----------|
 *
 * data set 1 (datatype=1, rotation calibration)
 * | 0-7(8) | 8-10(3) | 11-23(13) | 23-30(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |CALIB_DATA | CHECKSUM |
 * |--------|---------|-----------|----------|
 *
 * data set 2 (datatype=2, kicker command & robot states)
 * | 0-7(8) | 8-10(3) | 11-15(5)  | 16-23(8) |
 * |--------|---------|-----------|----------|
 * |HEADER  |DATATYPE |COMMAND    |CHECKSUM  |
 * |--------|---------|-----------|----------|
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle){
	  static bool in_escape_sequence=false;
	  static int checksum=HEAD_BYTE;
	  static bool is_first_loop=true;

	  if(receive_data[0]==HEAD_BYTE)
	  {
		if(is_first_loop)is_first_loop=false;
		else
		{
			int j_max=(ary_index<17)?ary_index:16;
			for(int j=0;j<j_max;j++)send_data[j]=data[j];
			if(((checksum-prev_receive_data)&0xFF)==send_data[j_max-1])
			{
				data_valid=true;
				valid_count++;
			}
			else data_valid=false;
	//		data_valid=check(checksum-prev_receive_data,j_max);

			data[0]=HEAD_BYTE;

			in_escape_sequence=false;
			checksum=HEAD_BYTE;
			for(int j=1;j<j_max;j++)data[j]=0;
			ary_index=1;
		}
	  }
	  else //not header byte
	  {
	    if(ary_index>16)
	      return;
	    else // not full buffer
	    {
	      if(receive_data[0]==ESCAPE_BYTE)
	      {
	        in_escape_sequence=true;
	      }
	      else // not receiving escape-byte
	      {
	        if(in_escape_sequence) // if previous data is escape-byte
	        {
	          checksum+=receive_data[0];
	          prev_receive_data=receive_data[0];
	          receive_data[0]^=ESCAPE_MASK;
	          in_escape_sequence=false;
	        }
	        else
	        {
	          checksum+=receive_data[0];
	          prev_receive_data=receive_data[0];
	        }
	        data[ary_index]=receive_data[0];
	        ary_index++;
	      }
	    }
	  }
	  irq_count++;

	//  sprintf(debug,"receive_data=%3d\n",receive_data[0]);
	//  SEGGER_RTT_Write(0,debug,strlen(debug));

	  HAL_UART_Receive_IT(&huart2, (uint8_t *)receive_data, 1);
}

void send_to_kicker(uint8_t a,uint8_t b){
	if(a==0){	// straight kick
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
		HAL_Delay(30);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
	}else{		// chip kick
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		HAL_Delay(30);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
	}

	kicker_state=0;

//	switch(kicker_state)
//	{
//	case shortpass:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
//		HAL_Delay(4);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		break;
//
//	case middlepass:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
//		HAL_Delay(8);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		break;
//
//	case longpass:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
//		HAL_Delay(12);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		break;
//
//	case shoot:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
//		HAL_Delay(15);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		break;
//
//	case clear:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
//		HAL_Delay(15);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		break;
//
//	case off:
//	default:
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
//		break;
//	}
//	kicker_state=off;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	coef1=cos(rad_coef*27.5);//37.5
	coef2=cos(rad_coef*62.5);//52.5
	coef3=cos(rad_coef*45);//45
  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  	ticker_interval=2*((float)htim1.Init.Prescaler + 1) * ((float)htim1.Init.Period + 1) / TIM_CLOCK;
  	//result=SD_MPU6050_Init(&hi2c2,&mpu1,SD_MPU6050_Device_0,SD_MPU6050_Accelerometer_2G,SD_MPU6050_Gyroscope_1000s );
//	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart2,(uint8_t *)rxdata,3);
	TxHeader.DLC=8;
	TxHeader.StdId=0x0001;
	TxHeader.ExtId=0x00000001;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.RTR=CAN_RTR_DATA;;
	TxHeader.TransmitGlobalTime=DISABLE;

	sFilterConfig.FilterBank = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.SlaveStartFilterBank = 0;

	if (HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	if(HAL_CAN_Start(&hcan)!=HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) {
//		_Error_Handler(__FILE__, __LINE__);
		Error_Handler();
	}
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, RESET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, RESET);
	HAL_Delay(500);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, SET);
	//HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	bool led_state=RESET;
	uint16_t debug_count=0;
	bool finish_flag=0;
	uint16_t finish_count=0;
#ifndef TODAY_CHARGE_KICK_ENABLE
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
#else
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
#endif
	HAL_Delay(1000);

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);

	HAL_TIM_Base_Start_IT(&htim1);	// 繧???��?��??��?��繧???��?��??��?��繝槫牡繧願ｾ???��?��??��?��縺???��?��??��?��髢句???��?��??��?��???��?��??��?��
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);	//繝峨Μ繝悶Λ�???��?��??��?��pwm
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);	//繝峨Μ繝悶Λ�???��?��??��?��pwm

//	// for test start
//
//	sensor_enable=0;
//	kick_type=0;
//	x_vector=10;
//	y_vector=10;
//	th_vector=10;
//	irq_count=11;
//
//	int test_count=0;
//
//	while(1){
//		test_count++;
//		switch(test_count%4==0){
//		case 0:
//		case 1:
//		case 2:
//		case 3:
//		default:
//			break;
//		}
//
//		if(sensor_enable){
//			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
//			//Added 8/17 by Suzuki
//			if(ball_existance){
//				HAL_Delay(50);
//				if(ball_existance)send_to_kicker(kick_type,kicker_state);
//			}
//			dribble_motor(0);
//
//		}else{
//			if(irq_count<=10 || (x_vector==0 && y_vector==0 && th_vector==0) )
//			{
//				dribble_motor(0);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,RESET);
//			}
//			else{
//				dribble_motor(1);
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
//			}
//		}
//
//		HAL_Delay(1000);
//
//	}
//		//for test end


#ifdef MD_CHECK_MODE
	while(1){
		//move forward
		Vx=1000;Vy=0;Vr=0;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

		//move backward
		Vx=-1000;Vy=0;Vr=0;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

		//move left
		Vx=0;Vy=1000;Vr=0;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

		//move right
		Vx=0;Vy=-1000;Vr=0;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

		//turn CCW
		Vx=0;Vy=0;Vr=100;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

		//turn CW
		Vx=0;Vy=0;Vr=-100;
		HAL_Delay(1000);
		Vx=0;Vy=0;Vr=0;
		HAL_Delay(1000);

	}
#else
	while(1)
	{
		if(data_valid){
			data_valid=false;
			if((send_data[1]>>5)==7){
				x_vector=(((int16_t)send_data[1]&0x1F)<<11)+((int16_t)send_data[2]<<3)+(send_data[3]>>5);
				Vr=(((int16_t)send_data[3]&0x1F)<<11)+((int16_t)send_data[4]<<3)+(send_data[5]>>5);
//				th_vector=(((int16_t)send_data[5]&0x1F)<<7)+((int16_t)send_data[6]>>1);
				th_vector=((int16_t)send_data[7]<<8|(int16_t)send_data[8]);
				}else if((send_data[1]>>5)==1){
					calib_data=(((int16_t)send_data[1]&0x1F)<<8)+send_data[2];
				}else if((send_data[1]>>5)==2){
					command=(send_data[1]&0x1F)<<4 | ((send_data[2]&0xF0)>>4);

					sensor_enable=(command>>7)&0x3;
					if(sensor_enable>2)sensor_enable=0; //out of range exception

					kick_type=(command>>6)&0x1;
					kicker_state=command&0x3F;
					if(kicker_state<0||5<kicker_state){
						kicker_state=0;
					}
					if(sensor_enable){
//						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
						/*Added 8/17 by Suzuki*/
#ifdef BALL_SENSOR_ENABLE
						if(ball_existance){
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,SET);
						}
#else
						HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,SET);
#endif
						dribble_motor(0);

					}else{
						if(irq_count<=10 || (x_vector==0 && y_vector==0 && th_vector==0) )
						{
							dribble_motor(0);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,RESET);
//							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,RESET);
						}
						else{
							dribble_motor(1);
							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_6,RESET);
//							HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
						}

//						__set_BASEPRI(1 << 4);
//						send_to_kicker(kick_type,kicker_state);
//						__set_BASEPRI(0);

					}
				}

//				__set_BASEPRI(1 << 4);
//			  if(x_vector==0 && y_vector==0 && th_vector==0 && command==0 );
//			  else{
//				  sprintf((char*)debug,"type=%d, x=%d, y=%d, th=%d, cmd=%d\n",send_data[1]>>5,x_vector,y_vector,th_vector,command);
//				  SEGGER_RTT_Write(0, debug, strlen((char*)debug));
//			  }
//				__set_BASEPRI(0);
			sprintf((char*)debug,"type=%d, x=%d, y=%d, th=%d, cmd=%d(sensor=%1d,type=%1d,state=%2d)\n",send_data[1]>>5,x_vector,y_vector,th_vector,command,sensor_enable,kick_type,kicker_state);
			SEGGER_RTT_Write(0, debug, strlen((char*)debug));

//			finish_charging_flag = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
//			if(finish_charging_flag){
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,SET);
//			}else{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,RESET);
//			}

			xbee_session_time=elapsed_time;
		  }else{
			  if(xbee_session_time-elapsed_time > xbee_timeout){
				  // xbee timeout
				  x_vector=0;
				  y_vector=0;
				  th_vector=0;
				  dribble_motor(0);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
				  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			  }
		  }
		sw1_state=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_8);
		if(sw1_state==RESET) {
			finish_count=0;
			if(finish_flag) break;
		} else finish_count++;
		if(finish_count>10000) {
			finish_count=10000;
			finish_flag=1;
		}
		Vx = x_vector;
		Vy = y_vector;
		if(Vx<=-4095)Vx=-4095;
		if(Vx>=4095)Vx=4095;
		if(Vy<=-4095)Vy=-4095;
		if(Vy>=4095)Vy=4095;
//		Theta_tar = (float)th_vector/10;
		Vr = (float)th_vector/10;

		debug_count++;
		if(debug_count>10000) {

//			sprintf(debug_str,"count:%d m1:%d m2:%d m3:%d m4:%d\r\n",debug_change_count,motor_out1,motor_out2,motor_out3,motor_out4);
//			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			debug_count=0;
		}

		debug_change_count++;

		if(debug_change_count>600000) {
			debug_change_count=0;
			dir++;
			if(dir>7) dir=0;
		}


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
	debug_count=0;

	kicker_state=off;
	__set_BASEPRI(1 << 4);
	send_to_kicker(kick_type,kicker_state);
	__set_BASEPRI(0);
	while(1) {
		kicker_state=off;
		__set_BASEPRI(1 << 4);
		send_to_kicker(kick_type,kicker_state);
		__set_BASEPRI(0);
		Vx=0;
		Vy=0;
		Theta_tar=0;
//		calc_out(Vx,Vy,Vr);
		debug_count++;
		if(debug_count>10000) {
			sprintf(debug_str,"finish\r\n");
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			debug_count=0;
		}
	}
#endif
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Adc1ClockSelection = RCC_ADC1PLLCLK_DIV1;

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config 
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x2000090E;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter 
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter 
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 9;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 57600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_2;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENABLE_OUT_Pin|STRAIGHT_OUT_Pin|CHIP_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE_OUT_Pin STRAIGHT_OUT_Pin CHIP_OUT_Pin */
  GPIO_InitStruct.Pin = ENABLE_OUT_Pin|STRAIGHT_OUT_Pin|CHIP_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CHARGE_DONE_IN_Pin */
  GPIO_InitStruct.Pin = CHARGE_DONE_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHARGE_DONE_IN_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
	sprintf(debug_str,"in Error_Handler\n");
	SEGGER_RTT_Write(0,debug_str,strlen(debug_str));
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
