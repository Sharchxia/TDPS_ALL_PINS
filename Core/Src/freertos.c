/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "tim.h"
#include "usart.h"
#include "i2c.h"
#include "gpio.h"
#include "timers.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct{
	float velCurrent;
	float errorN0;
	float errorN1;
	float errorN2;
	float kp,ki,kd;
}PID;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// those first five macros are used to print out some important debugging info.
// if some of them is not required, just annotate it, and vice verse.
//#define MY_DEBUG_START 				0 // verify if processes are started
//#define MY_DEBUG_STACK				0 // verify how large the space of stack leaves
//#define MY_DEBUG_OUT_ULTRA_INFO			0 // verify the measured distance from the two ultrasonics
//#define MY_DEBUG_OUT_VELOCITY		0 // verify the velcoity
//#define MY_DEBUG_OUT_ANGLE			0 // verify the measured angle

#define PATIO_1 		1
#define PATIO_2 		2

#define ADDRESS_CLK 0x68<<1

#define MPU_7BITS_ADDRESS1 		0x68 //pin AD0=0
#define MPU_7BITS_ADDRESS2 		0x69 //pin AD0=1
#define MPU_WRITE_ADDRESS1 		MPU_7BITS_ADDRESS1<<1 //0xd0 actual device address
#define MPU_WRITE_ADDRESS2 		MPU_7BITS_ADDRESS2<<1

#define TEMP_ADDRESS 			0x41 //2*8 measured temperature memory address
#define GYRO_ADDRESS 			0x43 //6*8 measured angular acceleration memory address
#define ACC_ADDRESS 			0x3b //6*8 measured acceleration memory address

#define WHO_AM_I 				0x75
#define INT_EN_ADDRESS 			0x38 // 1*8 address of interrupt register
#define BATT_1_ADDRESS 			0x6b //1*8 power register 1
#define BATT_2_ADDRESS 			0x6c //1*8 power register 2
#define FIFO_ADDRESS 			0x23 //1*8 FIPO register
#define CON_ADDRESS 			0x1a //1*8 config register
#define SAMP_FREQ_DIV_ADDRESS 	0x19 //1*8
#define ACC_CON_ADDRESS 		0x1c //1*8
#define GYRO_CON_ADDRESS 		0x1b //1*8

#define MPU_GYRO_ERROR 			0.3 // degree/s
#define MPU_ACC_ERROR			0.5
#define ANGLE_PARA 				1.02 // determine the constant parameter of the angle calculation
#define DIS_PARA				1.03


#define DELTA_OFFSET			1.2 // determine the tuning magnitude in patio2
#define GYRO_OFFSET				0.8 // determine the static offset of the angle calculation
#define VEL_OFFSET				1.2 // determine

#define G 						9.78 // acceleration of gravity
#define PI 						3.1415

// the followings are the parameters of the two PID used, better to not modify them
#define K_P 					10
#define K_I 					0.6
#define K_D 					0.3

#define KP 						7.34
#define KD 						1.2
#define KI 						0.3

#define CAMERA_FRONT 			24 // cm determine the assuming distance between the camera and the road in patio1
#define WHEEL_GAP 				24 // cm

#define MAX_DUTY_CYCLE 				0.5
#define WHEEL_RADIUS 				0.02 //m
#define DIS_PER_ROUND 				(float)WHEEL_RADIUS*2*PI
#define MAX_SPEED_RIGHT 			0.71 // m/s
#define MAX_SPEED_LEFT				0.76
#define MAX_TUNE_SPEED 				0.06
#define MAX_TUNE_PULSE_R			MOTOR_PWM_PERIOD_PULSE*MAX_TUNE_SPEED*2/MAX_SPEED_RIGHT
#define MAX_TUNE_PULSE_L			MOTOR_PWM_PERIOD_PULSE*MAX_TUNE_SPEED*2/MAX_SPEED_LEFT
#define INI_SPEED 					0.18
#define REAL_MAX_SPEED 				0.2 // m/s
#define REAL_MAX_SPEED_PULSE 		REAL_MAX_SPEED/MAX_SPEED*MOTOR_PWM_PERIOD_PULSE
#define MOTOR_PWM_PERIOD_PULSE 		20000 //20ms
#define MOTOR_PWM_STEP 				0.0001 //1us, 0.001ms
#define MOTOR_PWM_FREQ 				1000/MOTOR_PWM_STEP/MOTOR_PWM_PERIOD_PULSE

#define RIGHT_WHEEL 				1
#define LEFT_WHEEL					2

#define PATIO_NOTHING 				99

#define PATIO1_FIRST_TRACK 			0
#define PATIO1_CLOSE_SIGN 			1
#define PATIO1_TURN_CORNER1 		2
#define PATIO1_THROUGH_BRIDGE 		3
#define PATIO1_TURN_CORNER2 		4
#define PATIO1_SECOND_TRACK 		5
#define PATIO1_CLOSE_DOOR 			6
#define PATIO1_FINISH_TASK 			7

#define PATIO2_MOVE_CLOSER 			0
#define PATIO2_RECOGNISE 			1
#define PATIO2_KNOCK_ARROW 			2
#define PATIO2_RETURN_ORIGIN 		3
#define PATIO2_TURN_CORNER1 		4
#define PATIO2_FINISH_CORNER1 		5
#define PATIO2_TURN_OR_GO 			6
#define PATIO2_GO 					7
#define PATIO2_TURN_CORNER2 		8
#define PATIO2_FINISH_CORNER2 		9
#define PATIO2_TURN_CORNER3 		10
#define PATIO2_FINISH_CORNER3 		11
#define PATIO2_TURN_CORNER4 		12
#define PATIO2_FINISH_CORNER4 		13
#define PATIO2_TURN_CORNER5 		14
#define PATIO2_FINISH_CORNER5 		15
#define PATIO2_ARRIVE_BASKET 		16
#define PATIO2_FINISH_BASKET 		17
#define PATIO2_TURN_CORNER6 		17
#define PATIO2_FINISH_CORNER6 		18
#define PATIO2_TRANS_INFO 			19
#define PATIO2_FINISH_TRANS 		20
#define PATIO2_FINISH_TASK 			21

#define RECOGNISE_TIME 			7 // determine the recognising time in patio2

#define ARROW_FORWARD 			1
#define ARROW_LEFT 				2
#define ARROW_RIGHT 			3

#define ENGINE_CLOSE_PULSE 		1000 // 0.5ms
#define ENGINE_OPEN_PULSE 		500 // 2ms

#define DIS_MARGIN 				20 // cm
#define CORNER_MARGIN 			20 // degree
#define ANGLE_MARGIN 			12 // (degree) determine the tolerance of turning, not such important
#define	GYRO_LENGTH				3 // determine, not bigger than 4

#define PATIO1_A_X 		620 // cm
#define PATIO1_A_Y 		400
#define PATIO1_B_X		720
#define PATIO1_B_Y 		400
#define PATIO1_B_A 		-90
#define PATIO1_C_X 		720
#define PATIO1_C_Y		-150
#define PATIO1_C_A 		0
#define PATIO1_D_X 		870
#define PATIO1_D_Y 		-150

#define PATIO2_A_X 		162
#define PATIO2_A_Y 		0
#define PATIO2_B_X 		216
#define PAITO2_B_Y 		0
#define PATIO2_C_X 		270
#define PAITO2_C_Y 		0
#define PATIO2_D_X 		432
#define PATIO2_D_Y 		216
#define PATIO2_D_A 		45
#define PATIO2_E_X 		216
#define PATIO2_E_Y 		432
#define PATIO2_E_A 		90
#define PATIO2_F_X 		0
#define PATIO2_F_Y 		216
#define PATIO2_F_A 		135
#define PATIO2_G_X 		432
#define PATIO2_G_Y 		702
#define PATIO2_G_A 		90
#define PATIO2_H_X 		882
#define PATIO2_H_Y 		702
#define PATIO2_H_A 		0
#define PATIO2_I_X 		882
#define PATIO2_I_Y 		1032
#define PATIO2_I_A 		90
#define PATIO2_J_X 		1902
#define PATIO2_J_Y 		1032
#define PATIO2_J_A 		0
#define PATIO2_K_X 		1902
#define PATIO2_K_Y 		1392
#define PATIO2_K_A 		90
#define PATIO2_L_X 		597
#define PATIO2_L_Y 		1392
#define PATIO2_L_A 		180
#define PATIO2_M_X 		0
#define PATIO2_M_Y 		1392


#define PATIO1_OA_T		41140 //ms determine the processing time of every subprocess, the most important parameters
#define PATIO1_BC_T		9710
#define PATIO1_CD_T		2850
#define PATIO1_DE_T		2000

#define PATIO2_OA_T		9000
#define PATIO2_AC_T		200
#define PATIO2_CB_T		0 // 4100
#define PATIO2_BD_T		12173
#define PATIO2_BE_T		17746
#define PATIO2_BF_T		13273
#define PATIO2_DG_T 	23300
#define PATIO2_EG_T		14000
#define PATIO2_FG_T		23300
#define PAITO2_DH_T		12000 //16000
#define PAITO2_EH_T		20000 //24000
#define PAITO2_FH_T 	25000 //32000
#define PATIO2_HI_T 	1000 //10000
#define PATIO2_IJ_T		30000 //36000
#define PATIO2_JK_T		11000
#define PATIO2_KL_T		50000
#define PATIO2_LM_T		10000 // 22000
#define TIME_TIME		2500

#define FLOOR_W			30 //cm
#define P1_V			0.3
#define P1_V_D1			0.12
#define P1_V_O1			P1_V+P1_V_D1
#define P1_V_I1			P1_V-P1_V_D1
#define P1_V_D2			P1_V/3.5
#define P1_V_O2			P1_V+P1_V_D2
#define P1_V_I2			P1_V-P1_V_D2
#define P1_V_D3			0.12
#define P1_V_O3			P1_V+P1_V_D3
#define P1_V_I3			P1_V-P1_V_D3
#define P1_V_D4			P1_V/5
#define P1_V_O4			P1_V+P1_V_D4
#define P1_V_I4			P1_V-P1_V_D4
#define P1_V_D5			0.04
#define P1_V_O5			P1_V+P1_V_D5
#define P1_V_I5			P1_V-P1_V_D5

#define P1_1_T			13500
#define P1_2_T			2600
#define P1_3_T			11300
#define P1_4_T			3000
#define P1_5_T			27000
#define P1_6_T			2600
#define P1_7_T			9700
#define P1_8_T			3800
#define P1_9_T			19900
#define P1_11_T			10000
#define P1_12_T			0
#define P1_13_T			17000

#define P1_1			1
#define P1_2			2
#define P1_3			3
#define P1_4			4
#define P1_5			5
#define P1_6			6
#define P1_7			7
#define P1_8			8
#define P1_9			9
#define P1_10			10
#define P1_11			11
#define P1_12			12
#define P1_13			13
#define P1_14			14

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
uint8_t which_patio = 0;

float acc_xE1, acc_yE1, acc_zE1, gyro_xE1, gyro_yE1, gyro_zE1;// initialization error
float acc_xE2, acc_yE2, acc_zE2, gyro_xE2, gyro_yE2, gyro_zE2;
//float xPos=0, yPos=0, xV=0;
float zAngle=0;
float xP1Door = 0;
float ultra1dis[5] = {400, 400, 400, 400, 400}, ultra2dis[3] = {400, 400, 400};
float targetVelR=REAL_MAX_SPEED, targetVelL=REAL_MAX_SPEED;
uint32_t tAll = 0;

uint8_t patio1Tasks = 0;
uint8_t patio2Tasks = 0;
uint8_t patio1_flag = 0;
uint8_t patio2_flag = 0;

uint8_t finalDirection 			= 0;
uint8_t finishBasketFlag 		= 0;
uint8_t finishTransFlag 		= 0;
uint8_t patio2_recognise_finish = 0;

float P1deltaV = 0;

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for patio1ComApp */
osThreadId_t patio1ComAppHandle;
const osThreadAttr_t patio1ComApp_attributes = {
  .name = "patio1ComApp",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for patio1MotorCtrl */
osThreadId_t patio1MotorCtrlHandle;
const osThreadAttr_t patio1MotorCtrl_attributes = {
  .name = "patio1MotorCtrl",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for patio1RangeApp */
osThreadId_t patio1RangeAppHandle;
const osThreadAttr_t patio1RangeApp_attributes = {
  .name = "patio1RangeApp",
  .stack_size = 344 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for patio1Ultra1App */
osThreadId_t patio1Ultra1AppHandle;
const osThreadAttr_t patio1Ultra1App_attributes = {
  .name = "patio1Ultra1App",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for patio1Ultra2App */
osThreadId_t patio1Ultra2AppHandle;
const osThreadAttr_t patio1Ultra2App_attributes = {
  .name = "patio1Ultra2App",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for patio2ComApp */
osThreadId_t patio2ComAppHandle;
const osThreadAttr_t patio2ComApp_attributes = {
  .name = "patio2ComApp",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for patio2MotorCtrl */
osThreadId_t patio2MotorCtrlHandle;
const osThreadAttr_t patio2MotorCtrl_attributes = {
  .name = "patio2MotorCtrl",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal1,
};
/* Definitions for patio2RangeApp */
osThreadId_t patio2RangeAppHandle;
const osThreadAttr_t patio2RangeApp_attributes = {
  .name = "patio2RangeApp",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal2,
};
/* Definitions for patio2Ultra1App */
osThreadId_t patio2Ultra1AppHandle;
const osThreadAttr_t patio2Ultra1App_attributes = {
  .name = "patio2Ultra1App",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for patio2EngineApp */
osThreadId_t patio2EngineAppHandle;
const osThreadAttr_t patio2EngineApp_attributes = {
  .name = "patio2EngineApp",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for patio2TransApp */
osThreadId_t patio2TransAppHandle;
const osThreadAttr_t patio2TransApp_attributes = {
  .name = "patio2TransApp",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal4,
};
/* Definitions for patio2Ultra2App */
osThreadId_t patio2Ultra2AppHandle;
const osThreadAttr_t patio2Ultra2App_attributes = {
  .name = "patio2Ultra2App",
  .stack_size = 288 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void write_to_mpu(uint16_t subadd, uint8_t data);
void mpu_ini();
void read_from_mpu(uint16_t subadd, uint8_t* buffer1, uint8_t* buffer2, uint16_t length);
void read_data_from_mpu(uint16_t subadd, int16_t *data1, int16_t* data2);
void calculate_error();
void get_data_from_mpu(float*tmpF, float*acc_xF, float*acc_yF, float*acc_zF, float*gyro_xF, float*gyro_yF, float*gyro_zF);
void calculate_pos(float acc_x, float gyro_z, uint32_t t_gap);
uint8_t patio1_compare_pos();
uint8_t patio1_need_pid_R();
uint8_t patio1_need_pid_L();
uint8_t patio1_need_com();

void right_back();
void left_back();
void move_back();
void right_forward();
void left_forward();
void move_forward();
void robot_stop();
void right_forward_only();
void left_forward_only();
void left_forward_right_back();
void right_forward_left_back();
int16_t pid_algorithm(float targetVel, PID* pidPtr, uint8_t whichW);
void patio1_calculate_targetVel(int theta, int rho);
void calculate_vel_from_encoder(uint16_t encoder_data, PID* pidPtr, TickType_t t);
float vel_from_round(float round);
int16_t calculate_pulse_from_vel(float vel, uint8_t whichW);
void tune_velocity(float targetAngle, PID* pidPtr);
float pid_algorithm_ang(float deltaAng, PID* pidPtr);

uint8_t which_is_most(uint8_t forward, uint8_t left, uint8_t right);
uint8_t patio2_compare_pos();
uint8_t patio2_need_pid_R();
uint8_t patio2_need_pid_L();
void patio2_engine_open();
void patio2_engine_close();

float calculate_weight_average(float *buffer, uint8_t length);

void write_time();
void read_time();

void stack_debug(uint32_t leave);
void start_info_debug(const char* buffer);

void decode_from_mv_p1(uint8_t* buffer, int* rho, int* theta);
void decode_from_mv_p2(uint8_t* buffer, uint8_t* data);
void fifo_queue(float* array, uint8_t length, float data);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void patio1ComFcn(void *argument);
void patio1MotorCtrlFcn(void *argument);
void patio1RangeFcn(void *argument);
void patio1Ultra1Fcn(void *argument);
void patio1Ultra2Fcn(void *argument);
void patio2ComFcn(void *argument);
void patio2MotorCtrlFcn(void *argument);
void patio2RangeFcn(void *argument);
void patio2Ultra1Fcn(void *argument);
void patio2EngineFcn(void *argument);
void patio2TransFcn(void *argument);
void patio2Ultra2Fcn(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN PREPOSTSLEEP */
__weak void PreSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	HAL_SuspendTick(); //
/* place for user code */
}

__weak void PostSleepProcessing(uint32_t *ulExpectedIdleTime)
{
	HAL_ResumeTick(); //
/* place for user code */
}
/* USER CODE END PREPOSTSLEEP */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of patio1ComApp */
  patio1ComAppHandle = osThreadNew(patio1ComFcn, NULL, &patio1ComApp_attributes);

  /* creation of patio1MotorCtrl */
  patio1MotorCtrlHandle = osThreadNew(patio1MotorCtrlFcn, NULL, &patio1MotorCtrl_attributes);

  /* creation of patio1RangeApp */
  patio1RangeAppHandle = osThreadNew(patio1RangeFcn, NULL, &patio1RangeApp_attributes);

  /* creation of patio1Ultra1App */
  patio1Ultra1AppHandle = osThreadNew(patio1Ultra1Fcn, NULL, &patio1Ultra1App_attributes);

  /* creation of patio1Ultra2App */
  patio1Ultra2AppHandle = osThreadNew(patio1Ultra2Fcn, NULL, &patio1Ultra2App_attributes);

  /* creation of patio2ComApp */
  patio2ComAppHandle = osThreadNew(patio2ComFcn, NULL, &patio2ComApp_attributes);

  /* creation of patio2MotorCtrl */
  patio2MotorCtrlHandle = osThreadNew(patio2MotorCtrlFcn, NULL, &patio2MotorCtrl_attributes);

  /* creation of patio2RangeApp */
  patio2RangeAppHandle = osThreadNew(patio2RangeFcn, NULL, &patio2RangeApp_attributes);

  /* creation of patio2Ultra1App */
  patio2Ultra1AppHandle = osThreadNew(patio2Ultra1Fcn, NULL, &patio2Ultra1App_attributes);

  /* creation of patio2EngineApp */
  patio2EngineAppHandle = osThreadNew(patio2EngineFcn, NULL, &patio2EngineApp_attributes);

  /* creation of patio2TransApp */
  patio2TransAppHandle = osThreadNew(patio2TransFcn, NULL, &patio2TransApp_attributes);

  /* creation of patio2Ultra2App */
  patio2Ultra2AppHandle = osThreadNew(patio2Ultra2Fcn, NULL, &patio2Ultra2App_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
	uint16_t pulse=0;

	robot_stop(); // stop the car
	HAL_Delay(3000); //
	HAL_TIM_Base_Start(&htim5); // engine and motor
	HAL_TIM_Base_Start(&htim3); // ultrosanic
	HAL_TIM_Base_Start(&htim2); // ultrosanic capture base
	HAL_TIM_Base_Start(&htim8); // ultrosanic capture base

	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1); // 生成控制电机的PWM信号
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);// ultrosanic PWM

	mpu_ini(); //
	calculate_error(); //

//	read_time();

#ifdef MY_DEBUG_START
	start_info_debug("TIM5");
	char buffer[10] = {0};
	sprintf(buffer, "%s", "TIM3");
	start_info_debug(buffer);
	sprintf(buffer, "%s", "ultro_PWM");
	start_info_debug(buffer);
	sprintf(buffer, "%s", "TIM2");
	start_info_debug(buffer);
	sprintf(buffer, "%s", "TIM8");
	start_info_debug(buffer);
#endif


	if(HAL_GPIO_ReadPin(Selection_GPIO_Port, Selection_Pin)==GPIO_PIN_SET){ // 选择pin口的电平为低，确认程序执行patio1
		which_patio = PATIO_1;

		xTaskNotifyGive(patio1ComAppHandle); // start to commuicate with openMV
//		xTaskNotifyGive(patio1MotorCtrlHandle);
		xTaskNotifyGive(patio1RangeAppHandle);
		xTaskNotifyGive(patio1Ultra1AppHandle);

#ifdef MY_DEBUG_START
		HAL_UART_Transmit(&huart5, (uint8_t*)"Patio1 1\n", 9, 9);
#endif
	}else if(HAL_GPIO_ReadPin(Selection_GPIO_Port, Selection_Pin)==GPIO_PIN_RESET){ // patio2 执行
		which_patio = PATIO_2;

		HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3); // 启动舵机控制PWM信号
		patio2_engine_close();

#ifdef MY_DEBUG_START
		start_info_debug("Engine");
#endif
		xTaskNotifyGive(patio2RangeAppHandle);
		xTaskNotifyGive(patio2MotorCtrlHandle);
#ifdef MY_DEBUG_START
		HAL_UART_Transmit(&huart5, (uint8_t*)"Patio 2\n", 9, 9);
#endif
	}
	pulse = (uint16_t)(INI_SPEED/MAX_SPEED_RIGHT*MOTOR_PWM_PERIOD_PULSE);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, pulse);
	pulse = (uint16_t)(INI_SPEED/MAX_SPEED_LEFT*MOTOR_PWM_PERIOD_PULSE);
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, pulse);

  for(;;)
  {
    osDelay(1000);

    stack_debug(0);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_patio1ComFcn */
/**
* @brief Function implementing the patio1ComApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio1ComFcn */
void patio1ComFcn(void *argument)
{
  /* USER CODE BEGIN patio1ComFcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待进程启动
#ifdef MY_DEBUG_START
	start_info_debug("P1_Com_P");
#endif
	xTaskNotifyGive(patio1MotorCtrlHandle); // 通知电机控制进程启动
	uint8_t buffer[20]={0};
	HAL_StatusTypeDef state;
	int theta=0, rho=0;

  for(;;)
  {
	  if(patio1_need_com() == 1){ // 不处于转弯阶段和过桥阶段
//		  HAL_UART_Transmit(&huart5, (uint8_t*)"OK\n", 4, 4);
		  state = HAL_UART_Receive(&huart4, buffer, 20, 10);
		  if(state==HAL_OK){ // 获取数据
			  decode_from_mv_p1(buffer, &rho, &theta);
			  patio1_calculate_targetVel(theta, rho); // 算出两侧的�?�度
//			  HAL_UART_Transmit(&huart5, (uint8_t*)"KO\n", 4, 4);
//			  HAL_UART_Transmit(&huart5, buffer, 20, 10);
		  }else if(state==HAL_BUSY){
//			  HAL_UART_Transmit(&huart5, (uint8_t*)"KK\n", 4, 4);
		  }else{
//			  HAL_UART_Transmit(&huart5, (uint8_t*)"OO\n", 4, 4);
		  }
	  }

	  osDelay(20);
	  stack_debug(1);
  }
  /* USER CODE END patio1ComFcn */
}

/* USER CODE BEGIN Header_patio1MotorCtrlFcn */
/**
* @brief Function implementing the patio1MotorCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio1MotorCtrlFcn */
void patio1MotorCtrlFcn(void *argument)
{
  /* USER CODE BEGIN patio1MotorCtrlFcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待进程启动
#ifdef MY_DEBUG_START
	start_info_debug("P1MOT_P");
#endif
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动编码器（右轮编码器）
#ifdef MY_DEBUG_START
	start_info_debug("Enc_R");
#endif
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动编码器（左轮编码器）
#ifdef MY_DEBUG_START
	start_info_debug("Enc_L");
#endif

	PID pidR; // 初始化用于PID算法的数据等
	pidR.kd=K_D, pidR.ki=K_I,pidR.kp=K_P,pidR.errorN0=REAL_MAX_SPEED,pidR.errorN1=REAL_MAX_SPEED,pidR.errorN2=REAL_MAX_SPEED,pidR.velCurrent=0;
	PID pidL;
	pidL.kd=K_D, pidL.ki=K_I,pidL.kp=K_P,pidL.errorN0=REAL_MAX_SPEED,pidL.errorN1=REAL_MAX_SPEED,pidL.errorN2=REAL_MAX_SPEED,pidL.velCurrent=0;
	int16_t pulseR, pulseL;
	uint16_t encoderR=0, encoderL=0; // 初始化数�?????????
	int16_t pulseChangeR, pulseChangeL;
	TickType_t t = xTaskGetTickCount(); // ms,记录程序运行间隔时间

  for(;;)
  {
	  encoderR = __HAL_TIM_GET_COUNTER(&htim1); // 获取编码器数
	  encoderL = __HAL_TIM_GET_COUNTER(&htim4);
	  __HAL_TIM_SET_COUNTER(&htim1, 0); // 获取之后，重置编码器
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  t = xTaskGetTickCount() - t>20?xTaskGetTickCount() - t:20;; // 获取距离上一次计算�?�度时刻的时
	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1) && encoderR > 30000)
		  encoderR = 65535 - encoderR;
	  calculate_vel_from_encoder(encoderR, &pidR, t); // 由编码器数据得到小车速度
	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4) && encoderL > 30000)
		  encoderL = 65535 - encoderL;
	  calculate_vel_from_encoder(encoderL, &pidL, t);
	  t = xTaskGetTickCount();
#ifdef MY_DEBUG_OUT_VELOCITY
	  uint8_t bufferVelR[20]={0}, bufferVelL[20]={0};
	  sprintf((char*)bufferVelR, "realVelR=%1.3f\n", pidR.velCurrent);
	  HAL_UART_Transmit(&huart5, bufferVelR, 17, 20);
	  sprintf((char*)bufferVelL, "realVelL=%1.3f\n", pidL.velCurrent);
	  HAL_UART_Transmit(&huart5, bufferVelL, 17, 20);
#endif

	  if(patio1_need_pid_R() == 1)
	  	  pulseChangeR = pid_algorithm(targetVelR, &pidR, RIGHT_WHEEL); // 根据PID算法算出要改变的pulse
	  else
		  pulseChangeR = 0;
	  if(patio1_need_pid_L() == 1)
	  	  pulseChangeL = pid_algorithm(targetVelL, &pidL, LEFT_WHEEL);
	  else
		  pulseChangeL = 0;

	  pulseChangeR = pulseChangeR<(int16_t)(MAX_TUNE_PULSE_R*2)?pulseChangeR:(int16_t)(MAX_TUNE_PULSE_R*2);
	  pulseChangeL = pulseChangeL<(int16_t)(MAX_TUNE_PULSE_L*2)?pulseChangeL:(int16_t)(MAX_TUNE_PULSE_L*2);
	  pulseChangeR = pulseChangeR>-(int16_t)(MAX_TUNE_PULSE_R*2)?pulseChangeR:-(int16_t)(MAX_TUNE_PULSE_R*2);
	  pulseChangeL = pulseChangeL>-(int16_t)(MAX_TUNE_PULSE_L*2)?pulseChangeL:-(int16_t)(MAX_TUNE_PULSE_L*2);

	  pulseR = (int16_t)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) + pulseChangeR; // 计算新的pulse
	  pulseL = (int16_t)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) + pulseChangeL;
	  pulseR = pulseR<1?1:pulseR;
	  pulseL = pulseL<1?1:pulseL;

//	  uint8_t bufferVelR[20]={0}, bufferVelL[20]={0};
//	  sprintf((char*)bufferVelR, "pulseR=%d\n", pulseR);
//	  HAL_UART_Transmit(&huart5, bufferVelR, 17, 20);
//	  sprintf((char*)bufferVelL, "pulseL=%d\n", pulseL);
//	  HAL_UART_Transmit(&huart5, bufferVelL, 17, 20);

	  if(pulseR < MOTOR_PWM_PERIOD_PULSE)
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)pulseR);
	  else
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, MOTOR_PWM_PERIOD_PULSE);
	  if(pulseL < MOTOR_PWM_PERIOD_PULSE)
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)pulseL);
	  else
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, MOTOR_PWM_PERIOD_PULSE);

	  osDelay(15);

	  stack_debug(2);
  }
  /* USER CODE END patio1MotorCtrlFcn */
}

/* USER CODE BEGIN Header_patio1RangeFcn */
/**
* @brief Function implementing the patio1RangeApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio1RangeFcn */
void patio1RangeFcn(void *argument)
{
  /* USER CODE BEGIN patio1RangeFcn */
	  /* Infinite loop */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待启动里程
#ifdef MY_DEBUG_START
		start_info_debug("P1RNG_P");
#endif
		float tmp, accX, accY, gyroZ, accZ, gyroX, gyroY; //
		float gyroData[GYRO_LENGTH] = {0};
		PID pid;
		pid.errorN0=0,pid.errorN1=0,pid.errorN2=0,pid.kd=KD,pid.ki=KI,pid.kp=KP;

		TickType_t t = xTaskGetTickCount(); //
//		uint8_t turnOnFlag = 0; //

	  for(;;)
	  {
	    get_data_from_mpu(&tmp, &accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ); // 获取数据
	    fifo_queue(gyroData, GYRO_LENGTH, gyroZ);

	    t = xTaskGetTickCount() - t>5?xTaskGetTickCount() - t:5;; //
	    calculate_pos(accX, calculate_weight_average(gyroData, GYRO_LENGTH), t); //
	    t = xTaskGetTickCount(); //

	    switch(patio1_compare_pos()){
	    case P1_1: //
	    	move_forward(); //
	    	tune_velocity(0, &pid);
	    	break;
	    case P1_2:
	    	move_forward();
	    	P1deltaV = 0;
	    	targetVelR = P1_V_O1;
	    	targetVelL = P1_V_I1;
	    	break;
	    case P1_3:
	    	move_forward();
	    	tune_velocity(180+GYRO_OFFSET*2, &pid);
	    	break;
	    case P1_4:
	    	move_forward();
	    	P1deltaV = 0;
	    	targetVelR = P1_V_I2;
	    	targetVelL = P1_V_O2;
	    	break;
	    case P1_5:
	    	move_forward();
	    	tune_velocity(0, &pid);
	    	break;
	    case P1_6:
	    	move_forward();
	    	P1deltaV = 0;
	    	targetVelR = P1_V_O3;
	    	targetVelL = P1_V_I3;
	    	break;
	    case P1_7:
	    	move_forward();
	    	tune_velocity(180+GYRO_OFFSET*2, &pid);
	    	break;
	    case P1_8:
	    	move_forward();
	    	P1deltaV = 0;
	    	targetVelR = P1_V_I4;
	    	targetVelL = P1_V_O4;
	    	break;
	    case P1_9:
	    	 //
	    	move_forward();
	    	tune_velocity(0+GYRO_OFFSET, &pid);
	    	break;
	    case P1_10:
	    	left_forward_right_back();
	    	targetVelL = REAL_MAX_SPEED;
	    	targetVelR = targetVelL;
	    	break;
	    case P1_11:
	    	move_forward();
	    	tune_velocity(-90, &pid);
	    	break;
	    case P1_12:
	    	right_forward_left_back();
	    	targetVelL = REAL_MAX_SPEED;
	    	targetVelR = targetVelL;
	    	break;
	    case P1_13:
	    	P1deltaV = 0;
	    	move_forward();
	    	tune_velocity(0, &pid);
	    	break;
	    case P1_14:
	    	robot_stop();
	    	break;
	    default:
	    	move_forward();
	    }

	    osDelay(15);

	    stack_debug(3);
	  }
  /* USER CODE END patio1RangeFcn */
}

/* USER CODE BEGIN Header_patio1Ultra1Fcn */
/**
* @brief Function implementing the patio1Ultra1App thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio1Ultra1Fcn */
void patio1Ultra1Fcn(void *argument)
{
  /* USER CODE BEGIN patio1Ultra1Fcn */
    /* Infinite loop */
  	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待
#ifdef MY_DEBUG_START
  	start_info_debug("P1U1_P");
#endif
//  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // 启动对超声波1返回信号的捕
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
#ifdef MY_DEBUG_START
  	start_info_debug("ul1_cap");
#endif

  	uint32_t pulse=0;
  	float dis=0;
    for(;;)
    {
  	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待捕获完成
  	  pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
  	  dis = (float)pulse * 0.001*170; //cm 计算测量距离
  	  if(dis > 2){
  		  fifo_queue(ultra1dis, 5, dis);
  	  }

#ifdef MY_DEBUG_OUT_ULTRA_INFO
  	  uint8_t bufferDis[20]={0};
  	  sprintf((char*)bufferDis, "P1U1Dis:%4.3f\n", dis);
  	  HAL_UART_Transmit(&huart5, bufferDis, 18, 20);
#endif

  	  stack_debug(4);
    }
  /* USER CODE END patio1Ultra1Fcn */
}

/* USER CODE BEGIN Header_patio1Ultra2Fcn */
/**
* @brief Function implementing the patio1Ultra2App thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio1Ultra2Fcn */
void patio1Ultra2Fcn(void *argument)
{
  /* USER CODE BEGIN patio1Ultra2Fcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待启动进程
#ifdef MY_DEBUG_START
  	start_info_debug("P1U2_P");
#endif
//	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1); //
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
#ifdef MY_DEBUG_START
  	start_info_debug("ul2_cap");
#endif
	uint32_t pulse=0;
	float dis=0;
	for(;;){
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待捕获完成
	  pulse = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2);
	  dis = (float)pulse * 0.001*170; //cm，计算测
	  if(dis > 2){
	  	fifo_queue(ultra2dis, 3, dis);
	  }

#ifdef MY_DEBUG_OUT_ULTRA_INFO
	  uint8_t bufferDis[20]={0};
  	  sprintf((char*)bufferDis, "P1U2Dis:%4.3f\n", dis);
  	  HAL_UART_Transmit(&huart5, bufferDis, 18, 20);
#endif
	  	  stack_debug(5);
	    }
  /* USER CODE END patio1Ultra2Fcn */
}

/* USER CODE BEGIN Header_patio2ComFcn */
/**
* @brief Function implementing the patio2ComApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2ComFcn */
void patio2ComFcn(void *argument)
{
  /* USER CODE BEGIN patio2ComFcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef MY_DEBUG_START
  	start_info_debug("P2COM_P");
#endif
//	uint8_t buffer[20];
	uint8_t receiveTime = 0;
	uint8_t data[] = {0, 0};
	uint8_t forward=0, left=0, right=0;

  for(;;)
  {
	  robot_stop();
	  HAL_UART_Receive(&huart4, data, 1, HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart5, data,2, HAL_MAX_DELAY);
//	  HAL_UART_Transmit(&huart5,(uint8_t*)"ok\n",3, HAL_MAX_DELAY);

	  receiveTime ++;
	  if(data[0] == ARROW_FORWARD)
		  forward++;
	  else if(data[0] == ARROW_LEFT)
		  left++;
	  else if(data[0] == ARROW_RIGHT)
		  right ++;

	  if(receiveTime >= RECOGNISE_TIME){
		  finalDirection = which_is_most(forward, left, right);
//		  finalDirection = ARROW_LEFT;
		  patio2_recognise_finish = 1;
		  move_forward();
		  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	  }

	  data[0] = 0;
	  osDelay(15);

	  stack_debug(6);
  }
  /* USER CODE END patio2ComFcn */
}

/* USER CODE BEGIN Header_patio2MotorCtrlFcn */
/**
* @brief Function implementing the patio2MotorCtrl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2MotorCtrlFcn */
void patio2MotorCtrlFcn(void *argument)
{
  /* USER CODE BEGIN patio2MotorCtrlFcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 等待进程启动
	xTaskNotifyGive(patio2Ultra1AppHandle);
	xTaskNotifyGive(patio2Ultra2AppHandle);
#ifdef MY_DEBUG_START
  	start_info_debug("P2MOT_P");
#endif
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL); // 启动编码器（右轮编码器）
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL); // 启动编码器（左轮编码器）
#ifdef MY_DEBUG_START
  	start_info_debug("Enc_R");
  	start_info_debug("Enc_L");
#endif

	PID pidR; // 初始化用于PID算法的数据等
	pidR.kd=K_D, pidR.ki=K_I,pidR.kp=K_P,pidR.errorN0=REAL_MAX_SPEED,pidR.errorN1=REAL_MAX_SPEED,pidR.errorN2=REAL_MAX_SPEED,pidR.velCurrent=0;
	PID pidL;
	pidL.kd=K_D, pidL.ki=K_I,pidL.kp=K_P,pidL.errorN0=REAL_MAX_SPEED,pidL.errorN1=REAL_MAX_SPEED,pidL.errorN2=REAL_MAX_SPEED,pidL.velCurrent=0;
	int16_t pulseR, pulseL;
	uint16_t encoderR=0, encoderL=0; //
	int16_t pulseChangeR, pulseChangeL;
	TickType_t t = xTaskGetTickCount(); // ms,记录程序运行间隔时间

  for(;;)
  {

	  encoderR = __HAL_TIM_GET_COUNTER(&htim1); //
	  encoderL = __HAL_TIM_GET_COUNTER(&htim4);
	  __HAL_TIM_SET_COUNTER(&htim1, 0); // 获取之后，重置编码器
	  __HAL_TIM_SET_COUNTER(&htim4, 0);
	  t = xTaskGetTickCount() - t>20?xTaskGetTickCount() - t:20;; // 获取距离上一�????????
	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1) && encoderR > 30000)
		  encoderR = 65535 - encoderR;
	  calculate_vel_from_encoder(encoderR, &pidR, t); // 由编码器数据得到小车速度
	  if(__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4) && encoderL > 30000)
		  encoderL = 65535 - encoderL;
	  calculate_vel_from_encoder(encoderL, &pidL, t);
	  t = xTaskGetTickCount();

#ifdef MY_DEBUG_OUT_VELOCITY
	  uint8_t bufferVelR[20]={0}, bufferVelL[20]={0};
	  sprintf((char*)bufferVelR, "realVelR=%1.3f\n", pidR.velCurrent);
	  HAL_UART_Transmit(&huart5, bufferVelR, 16, 20);
	  sprintf((char*)bufferVelL, "realVelL=%1.3f\n", pidL.velCurrent);
	  HAL_UART_Transmit(&huart5, bufferVelL, 16, 20);
#endif

	  if(patio2_need_pid_R() == 1)
	  	  pulseChangeR = pid_algorithm(targetVelR, &pidR, RIGHT_WHEEL); // 根据PID算法算出要改变的pulse
	  else
		  pulseChangeR = 0;
	  if(patio2_need_pid_L() == 1)
	  	  pulseChangeL = pid_algorithm(targetVelL, &pidL, LEFT_WHEEL);
	  else
		  pulseChangeL = 0;

	  pulseChangeR = pulseChangeR<(int16_t)(-MAX_TUNE_PULSE_R)?(int16_t)(-MAX_TUNE_PULSE_R):pulseChangeR;
	  pulseChangeL = pulseChangeL<(int16_t)(-MAX_TUNE_PULSE_L)?(int16_t)(-MAX_TUNE_PULSE_L):pulseChangeL;
	  pulseChangeR = pulseChangeR>(int16_t)(MAX_TUNE_PULSE_R)?(int16_t)(MAX_TUNE_PULSE_R):pulseChangeR;
	  pulseChangeL = pulseChangeL>(int16_t)(MAX_TUNE_PULSE_L)?(int16_t)(MAX_TUNE_PULSE_L):pulseChangeL;

	  pulseR = (int16_t)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_1) + pulseChangeR; // 计算新的pulse
	  pulseL = (int16_t)__HAL_TIM_GET_COMPARE(&htim5, TIM_CHANNEL_2) + pulseChangeL;
	  pulseR = pulseR<1?1:pulseR;
	  pulseL = pulseL<1?1:pulseL;

	  if(pulseR < MOTOR_PWM_PERIOD_PULSE)
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, (uint16_t)pulseR);
	  if(pulseL < MOTOR_PWM_PERIOD_PULSE)
		  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, (uint16_t)pulseL);

	  osDelay(15);

	  stack_debug(7);
  }
  /* USER CODE END patio2MotorCtrlFcn */
}

/* USER CODE BEGIN Header_patio2RangeFcn */
/**
* @brief Function implementing the patio2RangeApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2RangeFcn */
void patio2RangeFcn(void *argument)
{
  /* USER CODE BEGIN patio2RangeFcn */
  /* Infinite loop */
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef MY_DEBUG_START
  	start_info_debug("P2RNG_P");
#endif
	PID pid;
	pid.errorN0=0,pid.errorN1=0,pid.errorN2=0,pid.kd=KD,pid.ki=KI,pid.kp=KP;
	float tmp, accX, accY, gyroZ, accZ, gyroX, gyroY; // 用于记录本次获取到的数据
	float gyroData[GYRO_LENGTH]={0};
	TickType_t t = xTaskGetTickCount(); // 记录程序启动的时
	uint8_t turnOnFlag = 0; // 防止影响IC中断


  for(;;)
  {
	  get_data_from_mpu(&tmp, &accX, &accY, &accZ, &gyroX, &gyroY, &gyroZ); // 获取数据
	  fifo_queue(gyroData, GYRO_LENGTH, gyroZ);
	  t = xTaskGetTickCount() - t>5?xTaskGetTickCount() - t:5;; // 记录距上次计算里程的时间间隔
	  calculate_pos(accX, calculate_weight_average(gyroData, GYRO_LENGTH), t); // 计算并更新里程计数据
	  t = xTaskGetTickCount(); // 记录时间时刻

	  switch(patio2_compare_pos()){
	  case PATIO2_MOVE_CLOSER:
		  move_forward();
		  tune_velocity(0, &pid);
		  break;
	  case PATIO2_RECOGNISE:
		  if(turnOnFlag == 0){
			  xTaskNotifyGive(patio2ComAppHandle);
		  }
		  turnOnFlag = 1;
		  robot_stop();
		  break;
	  case PATIO2_KNOCK_ARROW:
		  move_forward();
		  tune_velocity(0, &pid);
		  break;
	  case PATIO2_RETURN_ORIGIN:
		  move_back();
		  tune_velocity(180, &pid);
		  break;
	  case PATIO2_TURN_CORNER1:
		  right_forward_left_back();
		  break;
	  case PATIO2_FINISH_CORNER1:
		  move_forward();
		  switch(finalDirection){
		  case ARROW_RIGHT:
			  tune_velocity(PATIO2_D_A, &pid);
			  break;
		  case ARROW_FORWARD:
			  tune_velocity(PATIO2_E_A, &pid);
			  break;
		  case ARROW_LEFT:
			  tune_velocity(PATIO2_F_A, &pid);
		  }
		  break;
	  case PATIO2_TURN_OR_GO:
		  switch(finalDirection){
		  case ARROW_RIGHT:
			  right_forward_left_back();
			  break;
		  case ARROW_FORWARD:
			  robot_stop();
			  break;
		  case ARROW_LEFT:
			  left_forward_right_back();
		  }
		  break;

	  case PATIO2_GO:
		  move_forward();
		  tune_velocity(PATIO2_G_A+GYRO_OFFSET, &pid);
		  break;
	  case PATIO2_TURN_CORNER2:
		  left_forward_right_back();
		  break;
	  case PATIO2_FINISH_CORNER2:
		  move_forward();
		  tune_velocity(PATIO2_H_A+GYRO_OFFSET, &pid);
		  break;
	  case PATIO2_TURN_CORNER3:
		  right_forward_left_back();
		  break;
	  case PATIO2_FINISH_CORNER3:
		  move_forward();
		  tune_velocity(PATIO2_I_A+GYRO_OFFSET*2, &pid);
		  break;
	  case PATIO2_TURN_CORNER4:
		  left_forward_right_back();
		  break;
	  case PATIO2_FINISH_CORNER4:
		  move_forward();
		  tune_velocity(PATIO2_J_A+GYRO_OFFSET, &pid);
		  break;
	  case PATIO2_TURN_CORNER5:
		  right_forward_left_back();
		  break;
	  case PATIO2_FINISH_CORNER5:
		  move_forward();
		  tune_velocity(PATIO2_K_A, &pid);
		  break;
	  case PATIO2_ARRIVE_BASKET:
		  robot_stop();
		  xTaskNotifyGive(patio2EngineAppHandle);
		  break;
	  case PATIO2_TURN_CORNER6:
		  right_forward_left_back();
		  break;
	  case PATIO2_FINISH_CORNER6:
		  move_forward();
		  tune_velocity(PATIO2_L_A+GYRO_OFFSET*4, &pid);
		  break;
	  case PATIO2_TRANS_INFO:
		  xTaskNotifyGive(patio2TransAppHandle);
		  robot_stop();
		  break;
	  case PATIO2_FINISH_TRANS:
		  move_forward();
		  tune_velocity(PATIO2_L_A+GYRO_OFFSET*4, &pid);
		  break;
	  case PATIO2_FINISH_TASK:
		  robot_stop();
	  }
	  osDelay(15);

	  stack_debug(8);
  }
  /* USER CODE END patio2RangeFcn */
}

/* USER CODE BEGIN Header_patio2Ultra1Fcn */
/**
* @brief Function implementing the patio2Ultra1App thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2Ultra1Fcn */
void patio2Ultra1Fcn(void *argument)
{
  /* USER CODE BEGIN patio2Ultra1Fcn */
  /* Infinite loop */
  	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //
#ifdef MY_DEBUG_START
  	start_info_debug("P2U1_P");
#endif
//  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); //
  	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
#ifdef MY_DEBUG_START
  	start_info_debug("ul1_cap");
#endif
  	uint32_t pulse=0;
  	float dis=0;
    for(;;)
    {
  	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //
  	  pulse = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
  	  dis = (float)pulse * 0.001*170; //
  	  if(dis > 2){
  		  fifo_queue(ultra1dis, 5, dis);
  	  }

#ifdef MY_DEBUG_OUT_ULTRA_INFO
  	  uint8_t bufferDis[20]={0};
  	  sprintf((char*)bufferDis, "P2U1Dis:%4.3f\n", dis);
  	  HAL_UART_Transmit(&huart5, bufferDis, 18, 20);
#endif

  	  stack_debug(9);
    }
  /* USER CODE END patio2Ultra1Fcn */
}

/* USER CODE BEGIN Header_patio2EngineFcn */
/**
* @brief Function implementing the patio2EngineApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2EngineFcn */
void patio2EngineFcn(void *argument)
{
  /* USER CODE BEGIN patio2EngineFcn */
  /* Infinite loop */

  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef MY_DEBUG_START
	  start_info_debug("P2ENG_P");
#endif
	  patio2_engine_open();

	  osDelay(1000);
	  finishBasketFlag = 1;

	  stack_debug(10);
  }
  /* USER CODE END patio2EngineFcn */
}

/* USER CODE BEGIN Header_patio2TransFcn */
/**
* @brief Function implementing the patio2TransApp thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2TransFcn */
void patio2TransFcn(void *argument)
{
  /* USER CODE BEGIN patio2TransFcn */
  /* Infinite loop */

  for(;;)
  {
	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
#ifdef MY_DEBUG_START
  	start_info_debug("P2RNG_P");
#endif

	  read_time();
	  osDelay(1000);
	  finishTransFlag = 1;

	  stack_debug(11);
  }
  /* USER CODE END patio2TransFcn */
}

/* USER CODE BEGIN Header_patio2Ultra2Fcn */
/**
* @brief Function implementing the patio2Ultra2App thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_patio2Ultra2Fcn */
void patio2Ultra2Fcn(void *argument)
{
  /* USER CODE BEGIN patio2Ultra2Fcn */
  /* Infinite loop */
  	ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //
#ifdef MY_DEBUG_START
  	start_info_debug("P2U2_P");
#endif
//  	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1); //
  	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
#ifdef MY_DEBUG_START
  	start_info_debug("ul2_cap");
#endif
  	uint32_t pulse=0;
  	float dis=0;
    for(;;)
    {
  	  ulTaskNotifyTake(pdTRUE, portMAX_DELAY); //
  	  pulse = __HAL_TIM_GET_COMPARE(&htim8, TIM_CHANNEL_2);
  	  dis = (float)pulse * 0.001*170; //
  	  if(dis > 2){
  		  fifo_queue(ultra2dis, 3, dis);
  	  }

#ifdef MY_DEBUG_OUT_ULTRA_INFO
  	  uint8_t bufferDis[20]={0};
  	  sprintf((char*)bufferDis, "P2U2Dis:%4.3f\n", dis);
  	  HAL_UART_Transmit(&huart5, bufferDis, 18, 20);
#endif
  	  stack_debug(12);
    }

  /* USER CODE END patio2Ultra2Fcn */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void write_to_mpu(uint16_t subadd, uint8_t data){
	HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE_ADDRESS1, subadd, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
	HAL_I2C_Mem_Write(&hi2c1, MPU_WRITE_ADDRESS2, subadd, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

void mpu_ini(){ // using i2c to initialize the mpu
	write_to_mpu(BATT_1_ADDRESS, 0x81);
	HAL_Delay(100); // wait to ensure mpu is reset
	write_to_mpu(BATT_1_ADDRESS, 0x00);
	write_to_mpu(GYRO_CON_ADDRESS, 0x00); // +250 degree range
	write_to_mpu(ACC_CON_ADDRESS, 0x00); // +-2g range
	write_to_mpu(FIFO_ADDRESS, 0x00);
	write_to_mpu(SAMP_FREQ_DIV_ADDRESS, 0x18); // sampling frequency is 40Hz
	write_to_mpu(CON_ADDRESS, 0x04);
	write_to_mpu(INT_EN_ADDRESS, 0x00); // disable the interrupt
//	write_to_mpu(BATT_1_ADDRESS, 0x01);
	write_to_mpu(BATT_2_ADDRESS, 0x00); // use all measure modules
#ifdef MY_DEBUG_START
	start_info_debug("MPU_INI");
#endif
	HAL_Delay(100);
}

void read_from_mpu(uint16_t subadd, uint8_t* buffer1, uint8_t* buffer2, uint16_t length){
	HAL_I2C_Mem_Read(&hi2c1, MPU_WRITE_ADDRESS1, subadd, I2C_MEMADD_SIZE_8BIT, buffer1, length, HAL_MAX_DELAY);
	HAL_I2C_Mem_Read(&hi2c1, MPU_WRITE_ADDRESS2, subadd, I2C_MEMADD_SIZE_8BIT, buffer2, length, HAL_MAX_DELAY);
}

void read_data_from_mpu(uint16_t subadd, int16_t *data1, int16_t* data2){
	uint8_t buffer1[2], buffer2[2];

	read_from_mpu(subadd, buffer1, buffer2, 2);
	*data1 = (uint16_t)(buffer1[0]<<8|buffer1[1]); // bit operation to get another type data
	*data2 = (uint16_t)(buffer2[0]<<8|buffer2[1]);
}

void calculate_error(){
	int16_t acc_x1, acc_y1, acc_z1, gyro_x1, gyro_y1, gyro_z1;
	int16_t acc_x2, acc_y2, acc_z2, gyro_x2, gyro_y2, gyro_z2;

	read_data_from_mpu(ACC_ADDRESS, &acc_x1, &acc_x2);
	read_data_from_mpu(ACC_ADDRESS+2, &acc_y1, &acc_y2);
	read_data_from_mpu(ACC_ADDRESS+4, &acc_z1, &acc_z2);
	read_data_from_mpu(GYRO_ADDRESS, &gyro_x1, &gyro_x2);
	read_data_from_mpu(GYRO_ADDRESS+2, &gyro_y1, &gyro_y2);
	read_data_from_mpu(GYRO_ADDRESS+4, &gyro_z1, &gyro_z2);

	acc_xE1 = (float)acc_x1/16384*G; // relative error
	acc_yE1 = (float)acc_y1/16384*G;
	acc_zE1 = (float)acc_z1/16384*G - G;
	gyro_xE1 = (float)gyro_x1/131.07;
	gyro_yE1 = (float)gyro_y1/131.07;
	gyro_zE1 = (float)gyro_z1/131.07;

	acc_xE2 = (float)-acc_x2/16384*G; // relative error
	acc_yE2 = (float)-acc_y2/16384*G;
	acc_zE2 = (float)-acc_z2/16384*G + G;
	gyro_xE2 = (float)-gyro_x2/131.07;
	gyro_yE2 = (float)-gyro_y2/131.07;
	gyro_zE2 = (float)-gyro_z2/131.07;
#ifdef MY_DEBUG_START
	start_info_debug("MPU_ERROR");
#endif
}

void get_data_from_mpu(float*tmpF, float*acc_xF, float*acc_yF, float*acc_zF, float*gyro_xF, float*gyro_yF, float*gyro_zF){
	int16_t temp1, acc_x1, acc_y1, acc_z1, gyro_x1, gyro_y1, gyro_z1;
	int16_t temp2, acc_x2, acc_y2, acc_z2, gyro_x2, gyro_y2, gyro_z2;

	read_data_from_mpu(TEMP_ADDRESS, &temp1, &temp2);
	read_data_from_mpu(ACC_ADDRESS, &acc_x1, &acc_x2);
	read_data_from_mpu(ACC_ADDRESS+2, &acc_y1, &acc_y2);
	read_data_from_mpu(ACC_ADDRESS+4, &acc_z1, &acc_z2);
	read_data_from_mpu(GYRO_ADDRESS, &gyro_x1, &gyro_x2);
	read_data_from_mpu(GYRO_ADDRESS+2, &gyro_y1, &gyro_y2);
	read_data_from_mpu(GYRO_ADDRESS+4, &gyro_z1, &gyro_z2);

	*tmpF = ((float)temp1/340 + 36.53 - (float)temp2/340 + 36.53)/2;
	*acc_xF = ((float)acc_x1/16384*G-acc_xE1 - (float)acc_x2/16384*G-acc_xE2)/2;
	*acc_yF = ((float)acc_y1/16384*G-acc_yE1 - (float)acc_y2/16384*G-acc_yE2)/2;
	*acc_zF = ((float)acc_z1/16384*G-acc_zE1 - (float)acc_z2/16384*G-acc_zE2)/2;
	*gyro_xF = ((float)gyro_x1/131.07-gyro_xE1 - (float)gyro_x2/131.07-gyro_xE2)/2;
	*gyro_yF = ((float)gyro_y1/131.07-gyro_yE1 - (float)gyro_y2/131.07-gyro_yE2)/2;
	*gyro_zF = ((float)gyro_z1/131.07-gyro_zE1 - (float)gyro_z2/131.07-gyro_zE2)/2;
}

void right_back(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_SET);
}
void left_back(){
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_SET);
}
void move_back(){
	right_back();
	left_back();
}
void right_forward(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
}
void left_forward(){
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
}
void move_forward(){
	right_forward();
	left_forward();
}
void robot_stop(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
}

void right_forward_only(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
}

void left_forward_only(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
}

void left_forward_right_back(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_RESET);
}
void right_forward_left_back(){
	HAL_GPIO_WritePin(IN_3_GPIO_Port, IN_3_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(IN_4_GPIO_Port, IN_4_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_1_GPIO_Port, IN_1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(IN_2_GPIO_Port, IN_2_Pin, GPIO_PIN_SET);
}

int16_t pid_algorithm(float targetVel, PID* pidPtr, uint8_t whichW){
	float deltaV;
	int16_t pulseChange;

	pidPtr->errorN2 = pidPtr->errorN1,pidPtr->errorN1 = pidPtr->errorN0, pidPtr->errorN0 = targetVel-pidPtr->velCurrent;
	deltaV = pidPtr->kp*(pidPtr->errorN0 - pidPtr->errorN1) + pidPtr->ki*pidPtr->errorN0 + pidPtr->kd*(pidPtr->errorN0 - 2*pidPtr->errorN1 + pidPtr->errorN2);

	pulseChange = calculate_pulse_from_vel(deltaV, whichW);

	return pulseChange;
}

int16_t calculate_pulse_from_vel(float vel, uint8_t whichW){
	int16_t pulse = 0;

	float maxSpeed = MAX_SPEED_RIGHT;

	if(whichW == RIGHT_WHEEL)
		maxSpeed = MAX_SPEED_RIGHT;
	else if(whichW == LEFT_WHEEL)
		maxSpeed = MAX_SPEED_LEFT;

	pulse = (int16_t)(vel/maxSpeed*MOTOR_PWM_PERIOD_PULSE);

	return pulse;
}

void calculate_vel_from_encoder(uint16_t encoder_data, PID* pidPtr, TickType_t t){
	float roundPerSecond = 0;
	roundPerSecond = (float)encoder_data/(float)t*1000/1560/VEL_OFFSET; // 1560=4*13*30
	pidPtr->velCurrent = vel_from_round(roundPerSecond);
}

float vel_from_round(float round){
	float vel = 0;
	vel = round*DIS_PER_ROUND;

	return vel;
}

void calculate_pos(float acc_x, float gyro_z, uint32_t t_gap){
	float t=(float)t_gap;
	float zTmp;

	gyro_z = gyro_z<=0?(gyro_z<-MPU_GYRO_ERROR?gyro_z:0):(gyro_z>MPU_GYRO_ERROR?gyro_z:0);
	zTmp = gyro_z*t/1000*ANGLE_PARA;
	zAngle += zTmp;

#ifdef MY_DEBUG_OUT_ANGLE
	uint8_t buffer[20]={0};
	sprintf((char*)buffer, "zAngle=%03.5f\n", zAngle);
	HAL_UART_Transmit(&huart5, buffer, 18, 20);
#endif

	tAll += t_gap;
}

void tune_velocity(float targetAngle, PID* pidPtr){
	float P1vL, P1vR, P2vL, P2vR;
	float ultraV = 0;
	float deltaA = zAngle - targetAngle;
	if(deltaA < -135 || deltaA > 135)
		deltaA = -zAngle;
	float deltaV = pid_algorithm_ang(deltaA, pidPtr);
	float u2Dis = calculate_weight_average(ultra2dis, 3);

	if(which_patio == PATIO_2){
		if(u2Dis < DIS_MARGIN){
			ultraV = tanh(DIS_MARGIN-u2Dis)*MAX_TUNE_SPEED*2;
			ultraV = -ultraV;
		}else{
			u2Dis = DIS_MARGIN;
		}
	}else{
		u2Dis = DIS_MARGIN;
	}

	P2vL = REAL_MAX_SPEED + u2Dis/DIS_MARGIN*deltaV + (DIS_MARGIN-u2Dis)/DIS_MARGIN*ultraV;
	P2vR = REAL_MAX_SPEED - u2Dis/DIS_MARGIN*deltaV - (DIS_MARGIN-u2Dis)/DIS_MARGIN*ultraV;

	P1vL = P1_V+deltaV+P1deltaV;
	P1vR = P1_V-deltaV-P1deltaV;
	if(which_patio == PATIO_2){
		targetVelL = P2vL;
		targetVelR = P2vR;
	}else{
		targetVelL = P1vL;
		targetVelR = P1vR;
	}

//	uint8_t bufferVel[20];
//	sprintf((char*)bufferVel, "deltaV:%1.4f\n", P1deltaV);
//	HAL_UART_Transmit(&huart5, bufferVel, 15, 20);

#ifdef MY_DEBUG_OUT_VELOCITY
	uint8_t bufferVel[20], bufferVelL[20]={0}, bufferVelR[20]={0}, bufferUlV[20]={0};
	sprintf((char*)bufferVel, "deltaV:%1.4f\n", deltaV);
	HAL_UART_Transmit(&huart5, bufferVel, 15, 20);
	sprintf((char*)bufferVelL, "targetVLeft:%1.3f\n", targetVelL);
	HAL_UART_Transmit(&huart5, bufferVelL, 20, 20);
	sprintf((char*)bufferVelR, "targetVRight:%1.3f\n", targetVelR);
	HAL_UART_Transmit(&huart5, bufferVelR, 20, 20);
	sprintf((char*)bufferUlV, "ultraV:%1.3f\n", ultraV);
	HAL_UART_Transmit(&huart5, bufferUlV, 20, 20);
#endif
}

float pid_algorithm_ang(float deltaAng, PID* pidPtr){
	float deltaA = 0;

	pidPtr->errorN1 = pidPtr->errorN0;
	pidPtr->errorN0 = deltaAng;

	deltaA = pidPtr->kp*pidPtr->errorN0 + pidPtr->kd*(pidPtr->errorN0 - pidPtr->errorN1);

	deltaA = deltaA<-60?-60:deltaA;
	deltaA = deltaA>60?60:deltaA;

	float deltaV = sin(PI/180*deltaA);
	if(deltaV >= 0)
		deltaV = exp(deltaV+DELTA_OFFSET)-exp(DELTA_OFFSET);
	else
		deltaV = -exp(-deltaV+DELTA_OFFSET)-exp(DELTA_OFFSET);
	deltaV = tanh(deltaV)*MAX_TUNE_SPEED;

	return deltaV;
}

void patio1_calculate_targetVel(int theta, int rho){
	float deltaV = 0;

	deltaV = WHEEL_GAP*REAL_MAX_SPEED*sin(PI/90*theta)/CAMERA_FRONT;
//	deltaV = deltaV<-MAX_TUNE_SPEED*3?-MAX_TUNE_SPEED*3:deltaV;
//	deltaV = deltaV>MAX_TUNE_SPEED*3?MAX_TUNE_SPEED*3:deltaV;

	P1deltaV = deltaV;

#ifdef MY_DEBUG_OUT_VELOCITY
	uint8_t bufferVel[20], bufferVelL[20], bufferVelR[20];
	sprintf((char*)bufferVel, "deltaV:%1.4f\n", deltaV);
	HAL_UART_Transmit(&huart5, bufferVel, 15, 20);
	sprintf((char*)bufferVelL, "targetVLeft:%1.3f\n", targetVelL);
	HAL_UART_Transmit(&huart5, bufferVelL, 20, 20);
	sprintf((char*)bufferVelR, "targetVRight:%1.3f\n", targetVelR);
	HAL_UART_Transmit(&huart5, bufferVelR, 20, 20);
#endif

	return;
}

uint8_t patio1_compare_pos(){

	switch(patio1Tasks){
	case 0:
		patio1Tasks = 1;
		tAll = 0;
		break;
	case 1:
		if(tAll < P1_1_T){
			return P1_1;
		}else{
			patio1Tasks = 2;
			tAll = 0;
			return P1_2;
		}
	case 2:
		if(tAll < P1_2_T){
			return P1_2;
		}else{
			patio1Tasks = 3;
			tAll = 0;
			return P1_3;
		}
	case 3:
		if(tAll < P1_3_T){
			return P1_3;
		}else{
			patio1Tasks = 4;
			tAll = 0;
			return P1_4;
		}
	case 4:
		if(tAll < P1_4_T){
			return P1_4;
		}else{
			patio1Tasks = 5;
			tAll = 0;
			return P1_5;
		}
	case 5:
		if(tAll < P1_5_T){
			return P1_5;
		}else{
			patio1Tasks = 6;
			tAll = 0;
			return P1_6;
		}
	case 6:
		if(tAll < P1_6_T){
			return P1_6;
		}else{
			patio1Tasks = 7;
			tAll = 0;
			return P1_7;
		}
	case 7:
		if(tAll < P1_7_T){
			return P1_7;
		}else{
			patio1Tasks = 8;
			tAll = 0;
			return P1_8;
		}
	case 8:
		if(tAll < P1_8_T){
			return P1_8;
		}else{
			patio1Tasks = 9;
			tAll = 0;
			return P1_9;
		}
	case 9:
		if(tAll < P1_9_T){
			return P1_9;
		}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
			return P1_9;
		}else{
			patio1Tasks = 10;
			tAll = 0;
			return P1_10;
		}
	case 10:
		if(zAngle > -90+CORNER_MARGIN*1.5){
			return P1_10;
		}else{
			patio1Tasks = 11;
			tAll = 0;
			return P1_11;
		}
	case 11:
		if(tAll < P1_11_T){
			return P1_11;
		}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
			return P1_11;
		}else{
			patio1Tasks = 12;
			tAll = 0;
			return P1_12;
		}
	case 12:
		if(zAngle < 0-CORNER_MARGIN){
			return P1_12;
		}else{
			patio1Tasks = 13;
			tAll = 0;
			return P1_13;
		}
	case 13:
		if(tAll < P1_13_T){
			return P1_13;
		}else{
			patio1Tasks = 14;
			return P1_14;
		}
	case 14:
		return P1_14;
	}
	return PATIO_NOTHING;
}


uint8_t patio1_need_pid_R(){
	if(patio1_compare_pos()==P1_14)
		return 0;
	else
		return 1;
}

uint8_t patio1_need_pid_L(){
	if(patio1_compare_pos()==P1_14)
		return 0;
	else
		return 1;
}

uint8_t patio1_need_com(){
	switch(patio1_compare_pos()){
	case P1_2:
	case P1_4:
	case P1_6:
	case P1_8:
	case P1_10:
	case P1_11:
	case P1_12:
	case P1_14:
		return 0;
	}

	return 1;
}

uint8_t which_is_most(uint8_t forward, uint8_t left, uint8_t right){
	if(forward>=left && forward>=right)
		return ARROW_FORWARD;
	else if(right>=forward && right>=left)
		return ARROW_RIGHT;
	else
		return ARROW_LEFT;
}

uint8_t patio2_compare_pos(){

	switch(patio2Tasks){
	case 0:
		if(tAll < PATIO2_OA_T){
			return PATIO2_MOVE_CLOSER;
		}else{
			patio2Tasks = 1;
#ifdef MY_DEBUG_START
			start_info_debug("P2Recong");
#endif
			tAll = 0;
			return PATIO2_RECOGNISE;
		}
	case 1:
		if(patio2_recognise_finish == 1){
			patio2Tasks = 2;
#ifdef MY_DEBUG_START
			start_info_debug("P2Knock");
#endif
			tAll = 0;
			return PATIO2_KNOCK_ARROW;
		}else{
			return PATIO2_RECOGNISE;
		}
	case 2:
		if(tAll < PATIO2_AC_T){
			return PATIO2_KNOCK_ARROW;
		}else{
			patio2Tasks = 3;
#ifdef MY_DEBUG_START
			start_info_debug("P2Return");
#endif
			tAll = 0;
			return PATIO2_RETURN_ORIGIN;
		}
	case 3:
		if(tAll < PATIO2_CB_T){
			return PATIO2_RETURN_ORIGIN;
		}else{
			patio2Tasks = 4;
#ifdef MY_DEBUG_START
			start_info_debug("P2Turn_B");
#endif
			tAll = 0;
			return PATIO2_TURN_CORNER1;
		}
	case 4:
		switch(finalDirection){
		case ARROW_RIGHT:
			if(zAngle < PATIO2_D_A-ANGLE_MARGIN){
				return PATIO2_TURN_CORNER1;
			}else{
				patio2Tasks = 5;
				tAll = 0;
				return PATIO2_FINISH_CORNER1;
			}
		case ARROW_FORWARD:
			if(zAngle < PATIO2_E_A-ANGLE_MARGIN){
				return PATIO2_TURN_CORNER1;
			}else{
				patio2Tasks = 5;
				tAll = 0;
				return PATIO2_FINISH_CORNER1;
			}
		case ARROW_LEFT:
			if(zAngle < PATIO2_F_A-ANGLE_MARGIN){
				return PATIO2_TURN_CORNER1;
			}else{
				patio2Tasks = 5;
				tAll = 0;
				return PATIO2_FINISH_CORNER1;
			}
		}
		case 5:
			switch(finalDirection){
			case ARROW_RIGHT:
				if(tAll < PATIO2_BD_T){
					return PATIO2_FINISH_CORNER1;
				}else{
					patio2Tasks = 6;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_D");
#endif
					tAll = 0;
					return PATIO2_TURN_OR_GO;
				}
			case ARROW_FORWARD:
				if(tAll < PATIO2_BE_T){
					return PATIO2_FINISH_CORNER1;
				}else{
					patio2Tasks = 6;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_E");
#endif
					tAll = 0;
					return PATIO2_TURN_OR_GO;
				}
			case ARROW_LEFT:
				if(tAll < PATIO2_BF_T){
					return PATIO2_FINISH_CORNER1;
				}else{
					patio2Tasks = 6;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_F");
#endif
					tAll = 0;
					return PATIO2_TURN_OR_GO;
				}
			}
		case 6:
			switch(finalDirection){
			case ARROW_RIGHT:
				if(zAngle < PATIO2_G_A - ANGLE_MARGIN){
					return PATIO2_TURN_OR_GO;
				}else{
					patio2Tasks = 7;
#ifdef MY_DEBUG_START
					start_info_debug("P2F_Turn_D");
#endif
					tAll = 0;
					return PATIO2_GO;
				}
			case ARROW_FORWARD:
				patio2Tasks = 7;
				tAll = 0;
				return PATIO2_GO;
			case ARROW_LEFT:
				if(zAngle > PATIO2_G_A + ANGLE_MARGIN){
					return PATIO2_TURN_OR_GO;
				}else{
					patio2Tasks = 7;
#ifdef MY_DEBUG_START
					start_info_debug("P2F_Turn_F");
#endif
					tAll = 0;
					return PATIO2_GO;
				}
			}
		case 7:
			switch(finalDirection){
			case ARROW_RIGHT:
				if(tAll < PATIO2_DG_T){
					return PATIO2_GO;
				}else{
					patio2Tasks = 8;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_G");
#endif
					tAll = 0;
					return PATIO2_TURN_CORNER2;
				}

			case ARROW_FORWARD:
				if(tAll < PATIO2_EG_T){
					return PATIO2_GO;
				}else{
					patio2Tasks = 8;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_G");
#endif
					tAll = 0;
					return PATIO2_TURN_CORNER2;
				}
			case ARROW_LEFT:
				if(tAll < PATIO2_FG_T){
					return PATIO2_GO;
				}else{
					patio2Tasks = 8;
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_G");
#endif
					tAll = 0;
					return PATIO2_TURN_CORNER2;
				}
			}
		case 8:
			if(zAngle > PATIO2_H_A + ANGLE_MARGIN){
				return PATIO2_TURN_CORNER2;
			}else{
				patio2Tasks = 9;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Turn_G");
#endif
				tAll = 0;
				return PATIO2_FINISH_CORNER2;
			}
		case 9:
			switch(finalDirection){
			case ARROW_RIGHT:
				if(tAll < PAITO2_DH_T){
					return PATIO2_FINISH_CORNER2;
				}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
					return PATIO2_FINISH_CORNER2;
				}else{
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_H");
#endif
					patio2Tasks = 10;
					tAll = 0;
					return PATIO2_TURN_CORNER3;
				}
			case ARROW_FORWARD:
				if(tAll < PAITO2_EH_T){
					return PATIO2_FINISH_CORNER2;
				}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
					return PATIO2_FINISH_CORNER2;
				}else{
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_H");
#endif
					patio2Tasks = 10;
					tAll = 0;
					return PATIO2_TURN_CORNER3;
				}
			case ARROW_LEFT:
				if(tAll < PAITO2_FH_T){
					return PATIO2_FINISH_CORNER2;
				}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
					return PATIO2_FINISH_CORNER2;
				}else{
#ifdef MY_DEBUG_START
					start_info_debug("P2Turn_H");
#endif
					patio2Tasks = 10;
					tAll = 0;
					return PATIO2_TURN_CORNER3;
				}
			}
		case 10:
			if(zAngle < PATIO2_I_A - ANGLE_MARGIN){
				return PATIO2_TURN_CORNER3;
			}else{
				patio2Tasks = 11;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Turn_H");
#endif
				tAll = 0;
				return PATIO2_FINISH_CORNER3;
			}
		case 11:
			if(tAll < PATIO2_HI_T){
				return PATIO2_FINISH_CORNER3;
			}else if(calculate_weight_average(ultra2dis, 3)<DIS_MARGIN*5){
				tAll  =0;
				return PATIO2_FINISH_CORNER3;
			}else if(tAll < TIME_TIME){
				return PATIO2_FINISH_CORNER3;
			}else{
				patio2Tasks = 12;
#ifdef MY_DEBUG_START
				start_info_debug("P2Turn_I");
#endif
				tAll = 0;
				return PATIO2_TURN_CORNER4;
			}
		case 12:
			if(zAngle > PATIO2_J_A + ANGLE_MARGIN){
				return PATIO2_TURN_CORNER4;
			}else{
				patio2Tasks = 13;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Turn_I");
#endif
				tAll = 0;
				return PATIO2_FINISH_CORNER4;
			}
		case 13:
			if(tAll < PATIO2_IJ_T){
				return PATIO2_FINISH_CORNER4;
			}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
				return PATIO2_FINISH_CORNER4;
			}else{
				patio2Tasks = 14;
#ifdef MY_DEBUG_START
				start_info_debug("P2Turn_J");
#endif
				tAll = 0;
//				HAL_TIM_IC_Stop_IT(&htim8, TIM_CHANNEL_2);
				return PATIO2_TURN_CORNER5;
			}
		case 14:
			if(zAngle < PATIO2_K_A - ANGLE_MARGIN){
				return PATIO2_TURN_CORNER5;
			}else{
				patio2Tasks = 15;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Turn_J");
#endif
				tAll = 0;
				return PATIO2_FINISH_CORNER5;
			}
		case 15:
			if(tAll < PATIO2_JK_T){
				return PATIO2_FINISH_CORNER5;
			}else{
				patio2Tasks = 16;
#ifdef MY_DEBUG_START
				start_info_debug("P2Arr_Bas");
#endif
				tAll = 0;
				return PATIO2_ARRIVE_BASKET;
			}
		case 16:
			if(finishBasketFlag == 0){
				return PATIO2_ARRIVE_BASKET;
			}else{
				patio2Tasks = 17;
#ifdef MY_DEBUG_START
				start_info_debug("P2Throw");
#endif
				tAll = 0;
				return PATIO2_FINISH_BASKET;
			}
		case 17:
			if(zAngle < PATIO2_L_A - ANGLE_MARGIN){
				return PATIO2_TURN_CORNER6;
			}else{
				patio2Tasks = 18;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Turn_K");
#endif
				tAll = 0;
//				HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
				return PATIO2_FINISH_CORNER6;
			}
		case 18:
			if(tAll < PATIO2_KL_T){
				return PATIO2_FINISH_CORNER6;
			}else{
				patio2Tasks = 19;
#ifdef MY_DEBUG_START
				start_info_debug("P2Trans");
#endif
				tAll = 0;
				return PATIO2_TRANS_INFO;
			}
		case 19:
			if(finishTransFlag == 0){
				return PATIO2_TRANS_INFO;
			}else{
				patio2Tasks = 20;
#ifdef MY_DEBUG_START
				start_info_debug("P2F_Trans");
#endif
				tAll = 0;
				return PATIO2_FINISH_TRANS;
			}
		case 20:
			if(tAll < PATIO2_LM_T){
				return PATIO2_FINISH_TRANS;
			}else if(calculate_weight_average(ultra1dis, 5)>DIS_MARGIN){
				return PATIO2_FINISH_TRANS;
			}
			else{
				patio2Tasks = 21;
#ifdef MY_DEBUG_START
				start_info_debug("P2Finish");
#endif
				tAll = 0;
				return PATIO2_FINISH_TASK;
			}
		case 21:
			return PATIO2_FINISH_TASK;
	}

	return PATIO_NOTHING;
}

uint8_t patio2_need_pid_R(){
	switch(patio2_compare_pos()){
	case PATIO2_RECOGNISE:
	case PATIO2_ARRIVE_BASKET:
	case PATIO2_TRANS_INFO:
	case PATIO2_FINISH_TASK:
		return 0;
	}

	return 1;
}

uint8_t patio2_need_pid_L(){
	switch(patio2_compare_pos()){
	case PATIO2_RECOGNISE:
	case PATIO2_ARRIVE_BASKET:
	case PATIO2_TRANS_INFO:
	case PATIO2_FINISH_TASK:
		return 0;
	}

	return 1;
}

void patio2_engine_open(){
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ENGINE_OPEN_PULSE);
}

void patio2_engine_close(){
	__HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, ENGINE_CLOSE_PULSE);
}

float calculate_weight_average(float *buffer, uint8_t length){
	float average = 0;
//	float weight = 2/(float)((length+1)*length);
	for(int i=0;i<length;i++)
		average += buffer[i];
	return average/length;
}

void fifo_queue(float* array, uint8_t length, float data){
	for(uint8_t i=0; i<length-1; i++){
		array[i] = array[i+1];
	}
	array[length-1] = data;
}

void write_time(){
	uint8_t data[] = {0x00, 0x12, 0x21, 0x04, 0x13, 0x04, 0x23};
	HAL_I2C_Mem_Write(&hi2c2, ADDRESS_CLK, 0x00, I2C_MEMADD_SIZE_8BIT, data, sizeof(data), 100);
}

void read_time(){
	uint8_t buffer[60]={0};
	char teamName[] = "CosmicCrusaders";
	char weekday[][10] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
	char memberName[] = "XiaChi\nZhanYuwei\nShiZongxin\nWangZijun\nGuBorui\nYuYanqi\nHuangJi\nSongJiabei\nLiuXinyu\nXiaYixuan\n\n";
	HAL_I2C_Mem_Read(&hi2c2, ADDRESS_CLK, 0x00, I2C_MEMADD_SIZE_8BIT, buffer, 7, 100);
	int s, m, h, wd, d, mo, y;
	s = (buffer[0]>>4)*10 + (buffer[0]&0x0f);
	m = (buffer[1]>>4)*10 + (buffer[1]&0x0f);
	h = ((buffer[2]>>5)&0x01)*20 + ((buffer[2]>>4)&0x01)*10 + (buffer[2]&0x0f);
	wd = buffer[3];
	d = (buffer[4]>>4)*10 + (buffer[4]&0x0f);
	mo = (buffer[5]<<1>>5)*10 + (buffer[5]&0x0f);
	y = (buffer[6]>>4)*10 + (buffer[6]&0x0f);

	sprintf((char*)buffer, "%s\n20%02d-%02d-%02d\n%s\n%02d:%02d:%02d\n\n", teamName, y, mo, d, weekday[wd], h, m, s);
	HAL_UART_Transmit(&huart5, buffer, 45, HAL_MAX_DELAY);
	HAL_UART_Transmit(&huart5, (uint8_t*)memberName, 93, HAL_MAX_DELAY);
}

void stack_debug(uint32_t leave){
#ifdef MY_DEBUG_STACK
	uint8_t buffer[20] = {0};
	sprintf((char*)buffer, "stack%02lu:%3lu\n\n", leave, uxTaskGetStackHighWaterMark(NULL));
	HAL_UART_Transmit(&huart5, buffer, 20, 20);
#endif
	return;
}

void start_info_debug(const char* buffer){
	char info[50]={0};

	sprintf(info, "%s has been started\n\n", buffer);
	HAL_UART_Transmit(&huart5, (uint8_t*)info, 50, 50);
}

void decode_from_mv_p1(uint8_t buffer[], int* rho, int* theta){
	uint8_t my_index = 0;
	for(my_index=0; my_index<7; my_index++){
		if(buffer[my_index]==0xaa && buffer[my_index+1]==0xbb && buffer[my_index+2]==0xcc && buffer[my_index+3]==0xdd){
			if(buffer[my_index+15]==0xaa && buffer[my_index+14]==0xbb && buffer[my_index+13]==0xcc && buffer[my_index+12]==0xdd){
				*rho = *(int*)&buffer[my_index+4];
				*theta = *(int*)&buffer[my_index+8];
				return;
			}
		}
	}
}

void decode_from_mv_p2(uint8_t* buffer, uint8_t* data){
	uint8_t my_index = 0;
	for(my_index=0; my_index<10; my_index++){
		if(buffer[my_index]==0xaa){
			*data = buffer[my_index+1];
			return;
		}
	}
}

//callback functions following
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	BaseType_t taskWoken = pdFALSE;
	if(htim == &htim2){
		if(which_patio == PATIO_1)
			vTaskNotifyGiveFromISR(patio1Ultra1AppHandle, &taskWoken);
		else if(which_patio == PATIO_2)
			vTaskNotifyGiveFromISR(patio2Ultra1AppHandle, &taskWoken);
	}else if(htim == &htim8){
		if(which_patio == PATIO_1)
			vTaskNotifyGiveFromISR(patio1Ultra2AppHandle, &taskWoken);
		else if(which_patio == PATIO_2)
			vTaskNotifyGiveFromISR(patio2Ultra2AppHandle, &taskWoken);
	}
	portYIELD_FROM_ISR(taskWoken);
}
/* USER CODE END Application */

