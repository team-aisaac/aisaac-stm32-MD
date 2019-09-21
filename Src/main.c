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
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "SEGGER_RTT.h"
#include "string.h"
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
CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define M_PI 3.1415926535
#define MAX_SPEED 1800
//�?大duty�?(~/2000)
#define MIN_SPEED 50
//�?小duty�?(~/2000)
#define CALC_HZ 100.0
//速度計算�?�CAN受信のタイマ割込み周�?(Hz)
float RAD_COEF=1.0/500.0*CALC_HZ*2*M_PI; //エンコー�?から読んだ回転速度をrad/sにする
float DATA_COEF=1.0/4095.0*4000/60.0*2*M_PI;

#define Kp 0.00008
#define Ki 0
#define Kd 0

/*CAN*/
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t RxData[8]={0,0,0,0,0,0,0,0};
uint32_t TxMailbox=0;

CAN_FilterTypeDef sFilterConfig;

uint16_t receive_data=0;
uint8_t can_i1=0,can_i2=0;
int16_t test_data=0;
/*CAN*/

/*debug text*/
char debug_str[100];
int16_t debug_data=0;
uint8_t print_count=0;
bool tuning_flag=0;
uint8_t tuning_count=0;
/*debug text*/

bool select_sw1=0,select_sw2=0;
bool start_sw=0;
uint8_t board_number=0;

int16_t encoder_count=0,old_encoder_count=0;

uint8_t hall_state=0,old_hall_state=0;
bool hall_state1=0,hall_state2=0,hall_state3=0;

float PID_speed=0;
int16_t motor_speed=0;
int16_t old_motor_speed=0;
float target_speed=0;
float speed_intercept=130,speed_slope=3.0;

bool drive_flag=0;
float now_speed=0;
float diff_speed=0,old_diff_speed=0;

int direction=1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void motor_PID(float target_PID_speed) {//引数:rad/s,出�?(motor_speed):duty
	float motor_P=0,motor_I=0,motor_D=0;
	if(target_PID_speed==0) {
		PID_speed=0;
		motor_speed=0;
		old_motor_speed=0;
		diff_speed=0;
		old_diff_speed=0;
	} else {
		diff_speed=target_PID_speed-now_speed;
		motor_P=Kp*diff_speed;
		motor_D=Kd*(diff_speed-old_diff_speed);
		PID_speed+=(motor_P+motor_I+motor_D);
		if(target_PID_speed>0){
			motor_speed=(int16_t)(speed_intercept+speed_slope*target_PID_speed+PID_speed);
		}
		else if(target_PID_speed<0){
			motor_speed=(int16_t)((-1)*speed_intercept+speed_slope*target_PID_speed+PID_speed);
		}
		if(motor_speed>MAX_SPEED) motor_speed=MAX_SPEED;
		if(motor_speed<-MAX_SPEED) motor_speed=-MAX_SPEED;
		if(0<motor_speed && motor_speed<MIN_SPEED) motor_speed=MIN_SPEED;
		if(0>motor_speed && motor_speed>-MIN_SPEED) motor_speed=-MIN_SPEED;
		if(target_PID_speed*motor_speed<0) {
			motor_speed=0;
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		} else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		old_diff_speed=diff_speed;
	}
	//motor_speed=direction*300;
	//motor_speed=0;
}


void BLDC_drive(int16_t BLDC_speed) {//3相の�?�?
	if(BLDC_speed>MAX_SPEED) BLDC_speed=MAX_SPEED;
	else if(BLDC_speed<-MAX_SPEED) BLDC_speed=-MAX_SPEED;
	if(BLDC_speed>0) {
		if(hall_state==1) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);


			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		} else if(hall_state==3) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		} else if(hall_state==2) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else if(hall_state==6) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else if(hall_state==4) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		}
	} else if(BLDC_speed<0) {
		BLDC_speed=-BLDC_speed;
		if(hall_state==1) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else if(hall_state==3) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else if(hall_state==2) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		} else if(hall_state==6) {
			/*__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);*/

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		} else if(hall_state==4) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, RESET);
		}
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, SET);
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		now_speed=(encoder_count-old_encoder_count)*RAD_COEF;
		if(encoder_count>20000 || encoder_count<-20000) encoder_count=0;
		old_encoder_count=encoder_count;
		HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,RxData);
		if(RxHeader.StdId==0x0001) {//受け取ったデータは出力�?�大きさ12bitと符号1bit
			//receive_data=(int16_t)(((RxData[can_i1]&0xF)<<8) | RxData[can_i2]);
			receive_data=(uint16_t)(((RxData[can_i1]&0xF)<<8) | RxData[can_i2]);
			if(((RxData[can_i1]&0x10)==0x10)) {
				target_speed=-receive_data;
				debug_data=-receive_data;
			} else {
				target_speed=receive_data;
				debug_data=receive_data;
			}
			target_speed=target_speed*DATA_COEF;
		} else {
			receive_data=0;
			target_speed=0;
		}
		/*test_data=-2000;
		target_speed=test_data*DATA_COEF;*/
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
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
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
	HAL_TIM_Base_Start_IT(&htim2);

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
		Error_Handler();
	}

	if(HAL_CAN_Start(&hcan)!=HAL_OK) {
		Error_Handler();
	}

	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) {
		Error_Handler();
	}

	select_sw1=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	select_sw1=!select_sw1;
	select_sw2=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	select_sw2=!select_sw2;
	board_number=(select_sw2<<1) | select_sw1;

	if(board_number==0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		can_i1=0;
		can_i2=1;
	} else if(board_number==1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		can_i1=2;
		can_i2=3;
	} else if(board_number==2) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		can_i1=4;
		can_i2=5;
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		can_i1=6;
		can_i2=7;
	}

	//can_i1=board_number;
	//can_i2=board_number+1;
	motor_speed=0;

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	BLDC_drive(0);
	HAL_Delay(1000);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	hall_state1=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	hall_state2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	hall_state3=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	hall_state=(hall_state3<<2) | (hall_state2<<1) | (hall_state1);
	old_hall_state=hall_state;
	old_motor_speed=0;
	while(1) {
		print_count++;
		if(print_count>100) {
			sprintf(debug_str,"board:%d hall:%d target:%f debug:%d omega:%f output:%d\r\n",board_number,hall_state,target_speed,debug_data,now_speed,motor_speed);
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			//SEGGER_RTT_WriteString(0, debug_str);
			print_count=0;
		}
		motor_PID(target_speed);
		if(old_motor_speed==0 && motor_speed!=0) drive_flag=true;
		if(drive_flag==true) {
			if(old_hall_state==hall_state) {
				BLDC_drive(motor_speed);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
			}
			else {
				drive_flag=false;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
			}
		}
		old_motor_speed=motor_speed;
		old_hall_state=hall_state;
		start_sw=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
		//if(start_sw==RESET) break;
	}
	motor_speed=0;
	PID_speed=0;
	tuning_count=0;
	tuning_flag=0;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	while(1) {
		if(now_speed==0) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		sprintf(debug_str,"stand by flag:%d count:%d encoder:%d omega:%d\r\n",tuning_flag,tuning_count,encoder_count,encoder_count-old_encoder_count);
		SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
		HAL_Delay(500);
		start_sw=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
		if(start_sw==SET) tuning_count++;
		else tuning_count=0;
		if(tuning_count>3) tuning_flag=1;
		if(tuning_flag==1 && start_sw==RESET) break;
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
	while(1) {
		sprintf(debug_str,"hall:%d 1:%d 2:%d 3:%d\r\n",hall_state,hall_state1,hall_state2,hall_state3);
		SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
		HAL_Delay(500);
	}
	////
	while(1) {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		select_sw1=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
		select_sw2=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
		board_number=(select_sw1<<1) | select_sw2;
		if(!select_sw1) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		if(!select_sw2) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		start_sw=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);

		motor_speed=0;
		if(start_sw==RESET) break;

		if(receive_data==50) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		else HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
	while (1)
	{
		//motor_speed=-600;
		sprintf(debug_str,"hall:%d 1:%d 2:%d 3:%d\r\n",hall_state,hall_state1,hall_state2,hall_state3);
		SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
		//HAL_Delay(500);
		motor_speed=-300;
		HAL_Delay(1000);
		motor_speed=-600;
		HAL_Delay(1000);
		motor_speed=-300;
		HAL_Delay(1000);
		motor_speed=-100;
		HAL_Delay(1000);
		motor_speed=100;
		HAL_Delay(1000);
		motor_speed=300;
		HAL_Delay(1000);
		motor_speed=600;
		HAL_Delay(1000);
		motor_speed=300;
		HAL_Delay(1000);
		motor_speed=100;
		HAL_Delay(1000);
		motor_speed=-100;
		HAL_Delay(1000);
		/*if(board_number==0) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		} else if(board_number==1) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		} else if(board_number==2) {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		} else {
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, SET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
		HAL_Delay(1000);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, RESET);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
		HAL_Delay(1000);*/
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	}
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
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
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED3_Pin|LED4_Pin|LOW3_Pin|LOW2_Pin 
                          |LOW1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_B_Pin EN_A_Pin HALL3_Pin HALL1_Pin 
                           HALL2_Pin */
  GPIO_InitStruct.Pin = EN_B_Pin|EN_A_Pin|HALL3_Pin|HALL1_Pin 
                          |HALL2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : CURRENT_Pin */
  GPIO_InitStruct.Pin = CURRENT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CURRENT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED3_Pin LED4_Pin LOW3_Pin LOW2_Pin 
                           LOW1_Pin */
  GPIO_InitStruct.Pin = LED3_Pin|LED4_Pin|LOW3_Pin|LOW2_Pin 
                          |LOW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SELECT2_Pin SELECT1_Pin */
  GPIO_InitStruct.Pin = SELECT2_Pin|SELECT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	bool en_stateA=0,en_stateB=0;
	if(GPIO_Pin==GPIO_PIN_1) {
		en_stateA=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
		en_stateB=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0);
		if(en_stateA==GPIO_PIN_SET && en_stateB==GPIO_PIN_RESET) encoder_count--;
		else if(en_stateA==GPIO_PIN_SET && en_stateB==GPIO_PIN_SET) encoder_count++;
	}

	if(GPIO_Pin==GPIO_PIN_3 || GPIO_Pin==GPIO_PIN_4 || GPIO_Pin==GPIO_PIN_5) {
		hall_state1=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
		hall_state2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
		hall_state3=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
		hall_state=(hall_state3<<2) | (hall_state2<<1) | (hall_state1);
		BLDC_drive(motor_speed);
	}
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{
	}
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
