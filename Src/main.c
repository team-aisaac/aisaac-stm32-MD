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
#include "can.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdbool.h"
#include "math.h"
#include "SEGGER_RTT.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_SPEED 1900
#define MIN_SPEED 100
#define MAX_RPM 4000
#define CALC_HZ 1000
#define ENC_RES 2000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/*CAN*/
#define MD_USE_ID1 0x0001
#define MD_USE_ID2 0x0002
#define MD_USE_ID3 0x0003
#define MD_USE_ID4 0x0004
CAN_TxHeaderTypeDef TxHeader;
CAN_RxHeaderTypeDef RxHeader;
uint8_t TxData[8]={0,0,0,0,0,0,0,0};
uint8_t RxData[8]={0,0,0,0,0,0,0,0};
uint32_t TxMailbox=0;

CAN_FilterTypeDef sFilterConfig;

uint32_t MD_ID=0;
int16_t receive_data=0;
//uint16_t receive_data=0;
uint8_t can_i1=0,can_i2=0;
int16_t test_data=0;
/*CAN*/

/*HALL*/
uint8_t hall_state=0,old_hall_state=0;
bool hall_state1=0,hall_state2=0,hall_state3=0;
/*HALL*/

/*MOTOR*/
bool drive_flag=0;
int16_t motor_speed=0,old_motor_speed=0;
/*MOTOR*/

/*ENCODER*/
int16_t encoder_count=0,old_encoder_count=0;
uint8_t enc_state=0,old_enc_state=0;
bool enc_stateA=0,enc_stateB=0;
/*ENCODER*/

/*PID*/
float RAD_COEF=1.0/((float)ENC_RES)*2*M_PI*CALC_HZ;
float DATA_COEF=1.0/32767.0*MAX_RPM/60.0*2*M_PI;
float DATA_COEF2=1.0/32768.0*MAX_RPM/60.0*2*M_PI;
float Kp=0.012;
float Ki=0;
float Kd=0;
float now_speed=0;
float diff_speed=0,old_diff_speed=0;
float target_speed=0;
float PID_speed=0;
/*PID*/

/*DEBUG*/
char debug_str[100];
uint16_t printf_count=0;
/*DEBUG*/

/*SETTING*/
bool select_sw1=0,select_sw2=0;
uint8_t board_number=0;
bool MD_enable=0;
/*SETTING*/
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void BLDC_drive(int16_t BLDC_speed);
void motor_PID(float target_PID_speed);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM6) {
		now_speed=(encoder_count-old_encoder_count)*RAD_COEF;
		if(encoder_count>30000 || encoder_count<-30000) encoder_count=0;
		old_encoder_count=encoder_count;
		motor_PID(target_speed);
		if(old_motor_speed==0 && motor_speed!=0) drive_flag=true;
		old_motor_speed=motor_speed;
	} else if (htim->Instance == TIM7) {
		HAL_CAN_GetRxMessage(&hcan,CAN_RX_FIFO0,&RxHeader,RxData);
		if(RxHeader.StdId==MD_ID) {
			receive_data=(int16_t)((RxData[0]<<8) | RxData[1]);
			MD_enable=(bool)RxData[4];
			if(receive_data<0) {
				target_speed=receive_data*DATA_COEF2;
			} else {
				target_speed=receive_data*DATA_COEF;
			}
		}
	}
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM6_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	select_sw1=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
	select_sw1=!select_sw1;
	select_sw2=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
	select_sw2=!select_sw2;
	board_number=(select_sw2<<1) | select_sw1;

	if(board_number==0) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		MD_ID=MD_USE_ID1;
	} else if(board_number==1) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, RESET);
		MD_ID=MD_USE_ID2;
	} else if(board_number==2) {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		MD_ID=MD_USE_ID3;
	} else {
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, SET);
		MD_ID=MD_USE_ID4;
	}

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

	if (HAL_CAN_ConfigFilter(&hcan,&sFilterConfig)!=HAL_OK) Error_Handler();
	if(HAL_CAN_Start(&hcan)!=HAL_OK) Error_Handler();
	if(HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)!=HAL_OK) Error_Handler();

	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) Error_Handler();
	if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3) != HAL_OK) Error_Handler();
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_GPIO_WritePin(GPIOA, GREEN_LED_Pin, SET);

	while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15));

	hall_state1=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	hall_state2=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	hall_state3=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	hall_state=(hall_state3<<2) | (hall_state2<<1) | (hall_state1);
	old_hall_state=hall_state;
	old_motor_speed=0;
	enc_state=(enc_stateB<<1) | (enc_stateA);
	old_enc_state=enc_state;
	target_speed=0*DATA_COEF;
	motor_speed=0;
	while (1)
	{
		/*if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) motor_speed=0;
		else motor_speed=-1000;*/
		//if(old_motor_speed==0 && motor_speed!=0) drive_flag=true;
		if(drive_flag==true) {
			if(old_hall_state==hall_state) {
				BLDC_drive(motor_speed);
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, SET);
			}
			else {
				drive_flag=false;
				//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, RESET);
			}
		}
		//old_motor_speed=motor_speed;
		old_hall_state=hall_state;

		if(printf_count>10000) {
			//sprintf(debug_str,"target:%d-omega:%d-duty:%d\r\n",(int16_t)(target_speed),(int16_t)(now_speed),motor_speed);
			sprintf(debug_str,"ID:%d-receive:%d-Rx:%d\r\n",MD_ID,receive_data,RxData[1]);
			SEGGER_RTT_Write(0, debug_str, strlen(debug_str));
			printf_count=0;
		}
		printf_count++;
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

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin==ENC_A_Pin || GPIO_Pin==ENC_B_Pin) {
		enc_stateA=HAL_GPIO_ReadPin(GPIOA, ENC_A_Pin);
		enc_stateB=HAL_GPIO_ReadPin(GPIOA, ENC_B_Pin);
		enc_state=(enc_stateB<<1) | (enc_stateA);
		if(enc_state==1) {
			if(old_enc_state==0) encoder_count--;
			else if(old_enc_state==3) encoder_count++;
		} else if(enc_state==3) {
			if(old_enc_state==1) encoder_count--;
			else if(old_enc_state==2) encoder_count++;
		} else if(enc_state==2) {
			if(old_enc_state==3) encoder_count--;
			else if(old_enc_state==0) encoder_count++;
		} else if(enc_state==0) {
			if(old_enc_state==2) encoder_count--;
			else if(old_enc_state==1) encoder_count++;
		}
		old_enc_state=enc_state;
	} else if(GPIO_Pin==HALL1_Pin || GPIO_Pin==HALL2_Pin || GPIO_Pin==HALL3_Pin) {
		hall_state1=HAL_GPIO_ReadPin(GPIOA, HALL1_Pin);
		hall_state2=HAL_GPIO_ReadPin(GPIOA, HALL2_Pin);
		hall_state3=HAL_GPIO_ReadPin(GPIOA, HALL3_Pin);
		hall_state=(hall_state3<<2) | (hall_state2<<1) | (hall_state1);
		BLDC_drive(motor_speed);
	}
}

void BLDC_drive(int16_t BLDC_speed) {//3‘Š‚Ì‹ì“®
	if(!MD_enable) BLDC_speed=0;
	if(BLDC_speed>MAX_SPEED) BLDC_speed=MAX_SPEED;
	else if(BLDC_speed<-MAX_SPEED) BLDC_speed=-MAX_SPEED;
	if(BLDC_speed>0) {
		if(hall_state==1) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, SET);
		} else if(hall_state==3) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, SET);
		} else if(hall_state==2) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else if(hall_state==6) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else if(hall_state==4) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		}
	} else if(BLDC_speed<0) {
		BLDC_speed=-BLDC_speed;
		if(hall_state==1) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else if(hall_state==3) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else if(hall_state==2) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		} else if(hall_state==6) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, BLDC_speed);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, SET);
		} else if(hall_state==4) {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, SET);
		} else {
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, BLDC_speed);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

			HAL_GPIO_WritePin(GPIOB, LOW1_Pin, SET);
			HAL_GPIO_WritePin(GPIOB, LOW2_Pin, RESET);
			HAL_GPIO_WritePin(GPIOB, LOW3_Pin, RESET);
		}
	} else {
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

		HAL_GPIO_WritePin(GPIOB, LOW1_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LOW2_Pin, SET);
		HAL_GPIO_WritePin(GPIOB, LOW3_Pin, SET);
	}
}

void motor_PID(float target_PID_speed) {
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
		motor_speed=(int16_t)PID_speed;
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
