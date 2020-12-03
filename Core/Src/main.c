/* USER CODE BEGIN Header */
//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define angular_velocity_control
#define Enable_DOB
#define Enable_DFOB
// #define Enable_D_Controller_av
// #define Enable_Vehicle_Velocity_control
// #define Enable_Driving_force_FB
// #define Enable_Driving_Force_Control
#define Enable_Identification
//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */


/* Direct printf to output somewhere */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#ifndef __UUID_H
#define __UUID_H
//#define STM32_UUID ((uint32_t *)0x1FF0F420)
#define STM32_UUID ((uint32_t *)UID_BASE)
#endif //__UUID_H
/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}


float t = 0.0;
const float dt = 0.001;// Control sampling time [sec]
const float pi = 3.14159265358979;
uint16_t loop = 0;
uint8_t mode = 0;
uint8_t divide = 0;// Remainder when mode is divided by 3.

// * Encoder
uint16_t cnt1 = 0;
uint16_t cnt2 = 0;
uint16_t cnt3 = 0;
uint16_t cnt4 = 0;

int16_t cnt_offset = 30000;

uint16_t cnt1_pre = 0;
uint16_t cnt2_pre = 0;
uint16_t cnt3_pre = 0;
uint16_t cnt4_pre = 0;

int8_t digit1 = 0;// Digit Overflow
int8_t digit2 = 0;
int8_t digit3 = 0;
int8_t digit4 = 0;

#define Gear 64.0f
float rsl = 12.0;// Encoder resolution

int8_t direc1 = 0;// Wheel rotation direction
int8_t direc2 = 0;
int8_t direc3 = 0;
int8_t direc4 = 0;
// * Encoder


// * Motor Variables
float theta1_res = 0.0;// [rad]
float theta2_res = 0.0;
float theta3_res = 0.0;
float theta4_res = 0.0;

float theta1_res_pre = 0.0;// [rad]
float theta2_res_pre = 0.0;
float theta3_res_pre = 0.0;
float theta4_res_pre = 0.0;

float dtheta1_res = 0.0;// [rad/sec]
float dtheta2_res = 0.0;
float dtheta3_res = 0.0;
float dtheta4_res = 0.0;

float dtheta1_res_pre = 0.0;// [rad/sec]
float dtheta2_res_pre = 0.0;
float dtheta3_res_pre = 0.0;
float dtheta4_res_pre = 0.0;

#define G_LPF 20.0f // [rad/sec] : Up to half of sampling frequency 200.0 50.0

float dtheta1_res_raw = 0.0;// [rad/sec]
float dtheta2_res_raw = 0.0;
float dtheta3_res_raw = 0.0;
float dtheta4_res_raw = 0.0;

float ddtheta1_ref = 0.0;
float ddtheta2_ref = 0.0;
float ddtheta3_ref = 0.0;
float ddtheta4_ref = 0.0;

float dtheta1_cmd = 0.0;
float dtheta2_cmd = 0.0;
float dtheta3_cmd = 0.0;
float dtheta4_cmd = 0.0;

float i1_ref = 0.0;
float i2_ref = 0.0;
float i3_ref = 0.0;
float i4_ref = 0.0;

float ia1_ref = 0.0;
float ia2_ref = 0.0;
float ia3_ref = 0.0;
float ia4_ref = 0.0;

float ia1_ref_pre = 0.0;
float ia2_ref_pre = 0.0;
float ia3_ref_pre = 0.0;
float ia4_ref_pre = 0.0;

uint16_t PWM1 = 0;
uint16_t PWM2 = 0;
uint16_t PWM3 = 0;
uint16_t PWM4 = 0;

uint16_t PWM_constant = 0;
// * Motor Variables


// * Robot Variables
float vx_cmd = 0.0;
float vy_cmd = 0.0;
float dphi_cmd = 0.0;

float vx_res = 0.0;// [m/sec]
float vy_res = 0.0;
float dphi_res = 0.0;// [rad/sec]

float x_res = 0.0;// [m]
float y_res = 0.0;
float phi_res = 1.570796326794895;// [rad]
// * Robot Variables


// * Robot Parameters
#define W    0.15f // [m]
#define L    0.15f
#define Rw   0.05f // [m]
#define Mass 5.54f // [kg] Weight of MARK42
#define Jz   0.089f // [kg*m^2] 0.22cm, 0.38cm
// * Robot Parameters


// * Motor Parameters
// const float Ktn = 0.0134;// [Nm/A]
#define Ktn 0.0134f // [Nm/A]
#define i_max 1.4f // [A] : This is set in ESCON Studio 0.15
#define PWM_rsl 4000.0f // PWM resolution : This is set in STM32CubeIDE
#define a  Mass * Rw * Rw / 8.0f
#define b  Jz * Rw * Rw/(16.0*(L+W)*(L+W))
#define J1 5.7 / 10000000.0f
#define J2 5.7 / 10000000.0f
#define J3 5.7 / 10000000.0f
#define J4 5.7 / 10000000.0f

// These are the values on the side of wheels
// const float F1_plus  =  0.00002;// Coulomb friction torque [Nm]
// const float F1_minus = -0.00002;
// const float D1_plus  =  0.0011; // Viscous frcition coefficient [Nm*sec/rad]
// const float D1_minus = -0.0011;

// const float F2_plus  =  0.00002;
// const float F2_minus = -0.00002;
// const float D2_plus  =  0.0011;
// const float D2_minus = -0.0011;

// const float F3_plus  =  0.00002;
// const float F3_minus = -0.00002;
// const float D3_plus  =  0.0011;
// const float D3_minus = -0.0011;

// const float F4_plus  =  0.00002;
// const float F4_minus = -0.00002;
// const float D4_plus  =  0.0011;
// const float D4_minus = -0.0011;

// const float F1_plus  =  0.00002;// Coulomb friction torque [Nm]
// const float F1_minus = -0.00002;

// const float F2_plus  =  0.00002;
// const float F2_minus = -0.00002;

// const float F3_plus  =  0.00002;
// const float F3_minus = -0.00002;

// const float F4_plus  =  0.00002;
// const float F4_minus = -0.00002;

// * 2020/11/14
const float D1_plus  =  0.0004; // Viscous frcition coefficient [Nm*sec/rad]
const float D1_minus = 0.0003;

const float D2_plus  =  -0.0006;
const float D2_minus = -0.0003;

const float D3_plus  =  0.0042;
const float D3_minus = 0.0027;

const float D4_plus  =  -0.0005;
const float D4_minus = -0.000004;

// const float F1_plus  =  0.0324;// Coulomb friction torque [Nm]
// const float F1_minus = -0.0312;

// const float F2_plus  =  0.0385;
// const float F2_minus = -0.0357;

// const float F3_plus  =  0.0352;
// const float F3_minus = -0.0355;

// const float F4_plus  =  0.0582;
// const float F4_minus = -0.0525;


const float F1_plus  =  0.0;// Coulomb friction torque [Nm]
const float F1_minus = 0.0;

const float F2_plus  =  0.0;
const float F2_minus = 0.0;

const float F3_plus  =  0.0;
const float F3_minus = 0.0;

const float F4_plus  =  0.0;
const float F4_minus = 0.0;

// const float F1_plus  =  0.0158;// Coulomb friction torque [Nm]
// const float F1_minus = -0.0163;
// const float D1_plus  =  -0.002; // Viscous frcition coefficient [Nm*sec/rad]
// const float D1_minus = -0.002;

// const float F2_plus  =  0.0173;
// const float F2_minus = -0.017;
// const float D2_plus  =  -0.002;
// const float D2_minus = -0.0021;

// const float F3_plus  =  0.0211;
// const float F3_minus = -0.0186;
// const float D3_plus  =  -0.0005;
// const float D3_minus = -0.0003;

// const float F4_plus  =  0.0286;// 0.00002;
// const float F4_minus = -0.026;// -0.00002;
// const float D4_plus  =  -0.00211;// 0.0011;
// const float D4_minus = -0.0022;// -0.0011;

const float M11 = a + b + Gear * Gear * J1;// 0.00422
const float M12 = b;// 0.00015
const float M13 = a - b;// 0.00158
const float M14 = -b;// -0.00015

const float M21 = b;
const float M22 = a + b + Gear * Gear * J2;
const float M23 = -b;
const float M24 = a-b;

const float M31 = a-b;
const float M32 = -b;
const float M33 = a + b + Gear * Gear * J3;
const float M34 = b;

const float M41 = -b;
const float M42 = a-b;
const float M43 = b;
const float M44 = a + b + Gear * Gear * J4;
// * Motor Parameters


// * Control Gains etc.
#define Kp_av    20.0f// Gain for angular velocity control 100.0
#define Kp_av_df 10.0f// Gain for Driving Force    control 10.0
#define Kp_av_4 5.0f
float Kd_av = 2.0 * sqrt(Kp_av);
float G_LPF_D_av = 5.0;// D controller of angular velocity control

float D_controller1_av = 0.0;
float D_controller2_av = 0.0;
float D_controller3_av = 0.0;
float D_controller4_av = 0.0;
float D_controller1_av_pre = 0.0;
float D_controller2_av_pre = 0.0;
float D_controller3_av_pre = 0.0;
float D_controller4_av_pre = 0.0;

float delta_dtheta1 = 0.0;
float delta_dtheta2 = 0.0;
float delta_dtheta3 = 0.0;
float delta_dtheta4 = 0.0;

float delta_dtheta1_pre = 0.0;
float delta_dtheta2_pre = 0.0;
float delta_dtheta3_pre = 0.0;
float delta_dtheta4_pre = 0.0;

float Kp_vv_x   = 5.0;// Gain for vehicle velocity control(Based on encoder) 10.0
float Kp_vv_y   = 5.0;
float Kp_vv_phi = 5.0;

float ddx_ref = 0.0;
float ddy_ref = 0.0;
float ddphi_ref = 0.0;

#define Kp_df_x 100.0f//0.1f 0.5 10.0 50.0
#define Kp_df_y 100.0f//0.1f 0.5
#define Kp_df_phi 10000.0f//0.1f 5.0 100.0(1115-36)

#define Kp_df 1.2f//0.2f
#define Ki_df 0.01f // Ki Gain for driving force control 10.0 0.1 1.0 0.1 0.018
float fx_ref = 0.0;
float fy_ref = 0.0;
float Mz_ref = 0.0;

float fd1_ref = 0.0;
float fd2_ref = 0.0;
float fd3_ref = 0.0;
float fd4_ref = 0.0;

float Ki_df_integral1 = 0.0;
float Ki_df_integral2 = 0.0;
float Ki_df_integral3 = 0.0;
float Ki_df_integral4 = 0.0;

// Made a mistake
// float vx_ref_new = 0.0;
// float vy_ref_new = 0.0;
// float dphi_ref_new = 0.0;

float vel1_ref_new = 0.0; 
float vel2_ref_new = 0.0; 
float vel3_ref_new = 0.0; 
float vel4_ref_new = 0.0; 
// * Control Gains etc.


// * DOB
#define G_DOB 50.0f // [rad/sec]
float tau_dob1 = 0.0;
float tau_dob2 = 0.0;
float tau_dob3 = 0.0;
float tau_dob4 = 0.0;

float tau_dob1_pre = 0.0;
float tau_dob2_pre = 0.0;
float tau_dob3_pre = 0.0;
float tau_dob4_pre = 0.0;

float tau_dis1_raw = 0.0;
float tau_dis2_raw = 0.0;
float tau_dis3_raw = 0.0;
float tau_dis4_raw = 0.0;

float tau_dis1_raw_pre = 0.0;
float tau_dis2_raw_pre = 0.0;
float tau_dis3_raw_pre = 0.0;
float tau_dis4_raw_pre = 0.0;

float i1_comp = 0.0;
float i2_comp = 0.0;
float i3_comp = 0.0;
float i4_comp = 0.0;
// * DOB


// * DFOB
#define G_DFOB 20.0f // [rad/sec] 50.0 30.0
float tau_dfob1 = 0.0;
float tau_dfob2 = 0.0;
float tau_dfob3 = 0.0;
float tau_dfob4 = 0.0;

float tau_dfob1_pre = 0.0;
float tau_dfob2_pre = 0.0;
float tau_dfob3_pre = 0.0;
float tau_dfob4_pre = 0.0;

float tau_dfob1_raw = 0.0;// Almost the same as tau_dis1_raw. Just withdraw F+D*dtheta_res
float tau_dfob2_raw = 0.0;
float tau_dfob3_raw = 0.0;
float tau_dfob4_raw = 0.0;

float tau_dfob1_raw_pre = 0.0;
float tau_dfob2_raw_pre = 0.0;
float tau_dfob3_raw_pre = 0.0;
float tau_dfob4_raw_pre = 0.0;

float integral_tau_dfob1 = 0.0;
float integral_tau_dfob2 = 0.0;
float integral_tau_dfob3 = 0.0;
float integral_tau_dfob4 = 0.0;

float fd_hat1 = 0.0;
float fd_hat2 = 0.0;
float fd_hat3 = 0.0;
float fd_hat4 = 0.0;

float fx_hat = 0.0;
float fy_hat = 0.0;
float Mz_hat = 0.0;
// * DFOB


// * Save variables in SRAM
#define N_SRAM 1500 // Sampling Number of variables in SRAM (Number of array) // 3000 // About 50 variables : Up to 2500 sampling -> Set 2200 for safety

int i_save = 0;  // For "for sentences"
int i_output = 0;// For displaying datas after experiment

float t_SRAM[N_SRAM] = {};

float dtheta1_res_SRAM[N_SRAM] = {};// [rad/sec]
float dtheta2_res_SRAM[N_SRAM] = {};
float dtheta3_res_SRAM[N_SRAM] = {};
float dtheta4_res_SRAM[N_SRAM] = {};

float theta1_res_SRAM[N_SRAM] = {};// [rad]
float theta2_res_SRAM[N_SRAM] = {};
float theta3_res_SRAM[N_SRAM] = {};
float theta4_res_SRAM[N_SRAM] = {};

float ddtheta1_ref_SRAM[N_SRAM] = {};
float ddtheta2_ref_SRAM[N_SRAM] = {};
float ddtheta3_ref_SRAM[N_SRAM] = {};
float ddtheta4_ref_SRAM[N_SRAM] = {};

float i1_ref_SRAM[N_SRAM] = {};
float i2_ref_SRAM[N_SRAM] = {};
float i3_ref_SRAM[N_SRAM] = {};
float i4_ref_SRAM[N_SRAM] = {};

float ia1_ref_SRAM[N_SRAM] = {};
float ia2_ref_SRAM[N_SRAM] = {};
float ia3_ref_SRAM[N_SRAM] = {};
float ia4_ref_SRAM[N_SRAM] = {};

uint16_t PWM1_SRAM[N_SRAM] = {};
uint16_t PWM2_SRAM[N_SRAM] = {};
uint16_t PWM3_SRAM[N_SRAM] = {};
uint16_t PWM4_SRAM[N_SRAM] = {};

// float tau_dis1_raw_SRAM[N_SRAM] = {};
// float tau_dis2_raw_SRAM[N_SRAM] = {};
// float tau_dis3_raw_SRAM[N_SRAM] = {};
// float tau_dis4_raw_SRAM[N_SRAM] = {};

float fd1_ref_SRAM[N_SRAM] = {};
float fd2_ref_SRAM[N_SRAM] = {};
float fd3_ref_SRAM[N_SRAM] = {};
float fd4_ref_SRAM[N_SRAM] = {};

float Ki_df_integral1_SRAM[N_SRAM] = {};
float Ki_df_integral2_SRAM[N_SRAM] = {};
float Ki_df_integral3_SRAM[N_SRAM] = {};
float Ki_df_integral4_SRAM[N_SRAM] = {};

float tau_dob1_SRAM[N_SRAM] = {};
float tau_dob2_SRAM[N_SRAM] = {};
float tau_dob3_SRAM[N_SRAM] = {};
float tau_dob4_SRAM[N_SRAM] = {};

// float i1_comp_SRAM[N_SRAM] = {};
// float i2_comp_SRAM[N_SRAM] = {};
// float i3_comp_SRAM[N_SRAM] = {};
// float i4_comp_SRAM[N_SRAM] = {};

float tau_dfob1_SRAM[N_SRAM] = {};
float tau_dfob2_SRAM[N_SRAM] = {};
float tau_dfob3_SRAM[N_SRAM] = {};
float tau_dfob4_SRAM[N_SRAM] = {};

// float fd_hat1_SRAM[N_SRAM] = {};
// float fd_hat2_SRAM[N_SRAM] = {};
// float fd_hat3_SRAM[N_SRAM] = {};
// float fd_hat4_SRAM[N_SRAM] = {};

// float vx_res_SRAM[N_SRAM] = {};
// float vy_res_SRAM[N_SRAM] = {};
// float dphi_res_SRAM[N_SRAM] = {};

// float x_res_SRAM[N_SRAM] = {};
// float y_res_SRAM[N_SRAM] = {};
// float phi_res_SRAM[N_SRAM] = {};
// * Save variables in SRAM


// * fprintf
FILE *outputfile;
// * fprintf



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM9_Init(void);
/* USER CODE BEGIN PFP */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance==TIM2){
		// TIM2 task
    switch(mode){
      case 0:
		    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET); // Green

        PWM1 = 0.5*PWM_rsl;
        PWM2 = 0.5*PWM_rsl;
        PWM3 = 0.5*PWM_rsl;
        PWM4 = 0.5*PWM_rsl;

        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM1);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PWM2);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM3);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM4);
       
        break;
      case 1:
        cnt1 = TIM1->CNT;
        cnt2 = TIM3->CNT;
        cnt3 = TIM8->CNT;
        cnt4 = TIM4->CNT;

        if     (cnt1 - cnt1_pre > 0x10000/2) digit1--;
        else if(cnt1_pre - cnt1 > 0x10000/2) digit1++;
        if     (cnt2 - cnt2_pre > 0x10000/2) digit2--;
        else if(cnt2_pre - cnt2 > 0x10000/2) digit2++;
        if     (cnt3 - cnt3_pre > 0x10000/2) digit3--;
        else if(cnt3_pre - cnt3 > 0x10000/2) digit3++;
        if     (cnt4 - cnt4_pre > 0x10000/2) digit4--;
        else if(cnt4_pre - cnt4 > 0x10000/2) digit4++;

        theta1_res = (cnt1 - cnt_offset + digit1 * 0x10000) / (4.0 * rsl * Gear)*2.0*pi;// [rad]
        theta2_res = (cnt2 - cnt_offset + digit2 * 0x10000) / (4.0 * rsl * Gear)*2.0*pi;
        theta3_res = (cnt3 - cnt_offset + digit3 * 0x10000) / (4.0 * rsl * Gear)*2.0*pi;
        theta4_res = (cnt4 - cnt_offset + digit4 * 0x10000) / (4.0 * rsl * Gear)*2.0*pi;

        dtheta1_res_raw = ( theta1_res - theta1_res_pre )/dt;
        dtheta2_res_raw = ( theta2_res - theta2_res_pre )/dt;
        dtheta3_res_raw = ( theta3_res - theta3_res_pre )/dt;
        dtheta4_res_raw = ( theta4_res - theta4_res_pre )/dt;

        dtheta1_res = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta1_res_pre + 2.0 * G_LPF * (theta1_res - theta1_res_pre) );
        dtheta2_res = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta2_res_pre + 2.0 * G_LPF * (theta2_res - theta2_res_pre) );
        dtheta3_res = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta3_res_pre + 2.0 * G_LPF * (theta3_res - theta3_res_pre) );
        dtheta4_res = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta4_res_pre + 2.0 * G_LPF * (theta4_res - theta4_res_pre) );

        vx_res = (Rw / 4.0) * (dtheta1_res - dtheta2_res + dtheta3_res - dtheta4_res);// [m/sec]
        vy_res = (Rw / 4.0) * (dtheta1_res + dtheta2_res + dtheta3_res + dtheta4_res);
        dphi_res = (Rw / 4.0) / (W + L) * ( - dtheta1_res - dtheta2_res + dtheta3_res + dtheta4_res);// [rad/sec]

        x_res   += vx_res   * dt;// [m]
        y_res   += vy_res   * dt;
        phi_res += dphi_res * dt;// [rad]


        direc1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
        direc2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
        direc3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8);
        direc4 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);

        #ifdef Enable_Driving_Force_Control
        if(t < 25.0){
        vy_cmd = 0.3;// 0.4
        // vx_cmd = 0.3;
        // dphi_cmd = 5.0 / 3.0 * pi / 3.0;// [rad/sec]
        }else if(t >= 25.0){
          vx_cmd = 0.0;
          vy_cmd = 0.0;
          dphi_cmd = 0.0;
        }
        ddx_ref   = Kp_df_x   * (vx_cmd   -   vx_res);
        ddy_ref   = Kp_df_y   * (vy_cmd   -   vy_res);
        ddphi_ref = Kp_df_phi * (dphi_cmd - dphi_res);

        fx_ref = Mass * ddx_ref;
        fy_ref = Mass * ddy_ref;
        Mz_ref = Jz * ddphi_ref;

        // * Jacobi Matrix (T^T)^+ --> Future Work : Weighted Jacobi Matrix
        fd1_ref = 1.0 / 4.0 * (   fx_ref + fy_ref - ( L + W ) * Mz_ref );// Cancel Rw term
        fd2_ref = 1.0 / 4.0 * ( - fx_ref + fy_ref - ( L + W ) * Mz_ref );
        fd3_ref = 1.0 / 4.0 * (   fx_ref + fy_ref + ( L + W ) * Mz_ref );
        fd4_ref = 1.0 / 4.0 * ( - fx_ref + fy_ref + ( L + W ) * Mz_ref );

        // * I
        // Ki_df_integral1 += Ki_df * dt * ( fd1_ref - fd_hat1 );
        // Ki_df_integral2 += Ki_df * dt * ( fd2_ref - fd_hat2 );
        // Ki_df_integral3 += Ki_df * dt * ( fd3_ref - fd_hat3 );
        // Ki_df_integral4 += Ki_df * dt * ( fd4_ref - fd_hat4 );

        Ki_df_integral1 = Ki_df_integral1 + Ki_df * dt * ( fd1_ref - fd_hat1 );
        Ki_df_integral2 = Ki_df_integral2 + Ki_df * dt * ( fd2_ref - fd_hat2 );
        Ki_df_integral3 = Ki_df_integral3 + Ki_df * dt * ( fd3_ref - fd_hat3 );
        Ki_df_integral4 = Ki_df_integral4 + Ki_df * dt * ( fd4_ref - fd_hat4 );

        // * P / PI
        // vel1_ref_new = Kp_df * ( fd1_ref - fd_hat1 ) + Ki_df_integral1;
        // vel2_ref_new = Kp_df * ( fd2_ref - fd_hat2 ) + Ki_df_integral2;
        // vel3_ref_new = Kp_df * ( fd3_ref - fd_hat3 ) + Ki_df_integral3;
        // vel4_ref_new = Kp_df * ( fd4_ref - fd_hat4 ) + Ki_df_integral4;

        // * I
        vel1_ref_new = Ki_df_integral1;
        vel2_ref_new = Ki_df_integral2;
        vel3_ref_new = Ki_df_integral3;
        vel4_ref_new = Ki_df_integral4;

        dtheta1_cmd = vel1_ref_new / Rw;
        dtheta2_cmd = vel2_ref_new / Rw;
        dtheta3_cmd = vel3_ref_new / Rw;
        dtheta4_cmd = vel4_ref_new / Rw;
        #endif

        #ifdef Enable_Driving_force_FB
        if( t < 3.0 ){
          fd_hat1 = 0.0;
          fd_hat2 = 0.0;
          fd_hat3 = 0.0;
          fd_hat4 = 0.0;
        }else if(t >= 3.0 && t < 10.0){
          vy_cmd = 0.5;// [m/sec]
          // vx_cmd = 0.5;
        }else if(t >= 10.0){
          vx_cmd = 0.0;
          vy_cmd = 0.0;
          dphi_cmd = 0.0;
        }

        ddx_ref   = Kp_df_x   * (vx_cmd   -   vx_res);
        ddy_ref   = Kp_df_y   * (vy_cmd   -   vy_res);
        ddphi_ref = Kp_df_phi * (dphi_cmd - dphi_res);

        fx_ref = Mass * ddx_ref;
        fy_ref = Mass * ddy_ref;
        Mz_ref = Jz * ddphi_ref;

        fd1_ref =   fx_ref + fy_ref - ( L + W ) * Mz_ref;// Cancel Rw term
        fd2_ref = - fx_ref + fy_ref - ( L + W ) * Mz_ref;
        fd3_ref =   fx_ref + fy_ref + ( L + W ) * Mz_ref;
        fd4_ref = - fx_ref + fy_ref + ( L + W ) * Mz_ref;

        fd1_ref = 0.0;
        fd2_ref = 0.0;
        fd3_ref = 0.0;
        fd4_ref = 0.0;

        Ki_df_integral1 += Ki_df * dt * ( fd1_ref - fd_hat1 );
        Ki_df_integral2 += Ki_df * dt * ( fd2_ref - fd_hat2 );
        Ki_df_integral3 += Ki_df * dt * ( fd3_ref - fd_hat3 );
        Ki_df_integral4 += Ki_df * dt * ( fd4_ref - fd_hat4 );

        ddtheta1_ref = Kp_df * ( fd1_ref - fd_hat1 ) + Ki_df_integral1;
        ddtheta2_ref = Kp_df * ( fd2_ref - fd_hat2 ) + Ki_df_integral2;
        ddtheta3_ref = Kp_df * ( fd3_ref - fd_hat3 ) + Ki_df_integral3;
        ddtheta4_ref = Kp_df * ( fd4_ref - fd_hat4 ) + Ki_df_integral4;
        #endif

        #ifdef Enable_Vehicle_Velocity_control
        // if(t < 25.0){
        //   vx_cmd = 0.3;
        //   vy_cmd = 0.3;// [m/sec]
        //   dphi_cmd = pi / 3.0;// [rad/sec]

        // if(t > 3.0){
        // if(t > 3.0 && t < 10.0){
        //   dphi_cmd = pi / 6.0;// [rad/sec]
        //   vx_cmd = 0.5;
        // }else if(t >= 10.0){
        // }else if(t >= 25.0){
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        //   dphi_cmd = 0.0;
        // }

        vy_cmd = 0.5;

        ddx_ref   = Kp_vv_x   * (vx_cmd   -   vx_res);
        ddy_ref   = Kp_vv_y   * (vy_cmd   -   vy_res);
        ddphi_ref = Kp_vv_phi * (dphi_cmd - dphi_res);

        ddtheta1_ref =  20.0 * ddx_ref + 20.0 * ddy_ref - 6.0 * ddphi_ref;// [rad/sec^2]
        ddtheta2_ref = -20.0 * ddx_ref + 20.0 * ddy_ref - 6.0 * ddphi_ref;
        ddtheta3_ref =  20.0 * ddx_ref + 20.0 * ddy_ref + 6.0 * ddphi_ref;
        ddtheta4_ref = -20.0 * ddx_ref + 20.0 * ddy_ref + 6.0 * ddphi_ref;
        #endif

        #ifdef angular_velocity_control
        // Convert Local to Joint space
        // Jacobi T matirix (including "Rw")

        // if(t > 3.0){
        //   vy_cmd = -0.3;// [m/sec]
        // }

        // vx_cmd = 0.0;
        vy_cmd = 0.3;
        // dphi_cmd = 0.0;

        dtheta1_cmd =  20.0 * vx_cmd + 20.0 * vy_cmd - 6.0 * dphi_cmd;// [rad/sec]
        dtheta2_cmd = -20.0 * vx_cmd + 20.0 * vy_cmd - 6.0 * dphi_cmd;
        dtheta3_cmd =  20.0 * vx_cmd + 20.0 * vy_cmd + 6.0 * dphi_cmd;
        dtheta4_cmd = -20.0 * vx_cmd + 20.0 * vy_cmd + 6.0 * dphi_cmd;

        ddtheta1_ref = Kp_av * (dtheta1_cmd - dtheta1_res);
        ddtheta2_ref = Kp_av * (dtheta2_cmd - dtheta2_res);
        ddtheta3_ref = Kp_av * (dtheta3_cmd - dtheta3_res);
        ddtheta4_ref = Kp_av * (dtheta4_cmd - dtheta4_res);
        // ddtheta4_ref = Kp_av_4 * (dtheta4_cmd - dtheta4_res);
        #endif

        #ifdef Enable_Driving_Force_Control
        ddtheta1_ref = Kp_av_df * (dtheta1_cmd - dtheta1_res);
        ddtheta2_ref = Kp_av_df * (dtheta2_cmd - dtheta2_res);
        ddtheta3_ref = Kp_av_df * (dtheta3_cmd - dtheta3_res);
        ddtheta4_ref = Kp_av_df * (dtheta4_cmd - dtheta4_res);
        #endif

        i1_ref = (M11*ddtheta1_ref + M12*ddtheta2_ref + M13*ddtheta3_ref + M14*ddtheta4_ref)/( Gear * Ktn );
        i2_ref = (M21*ddtheta1_ref + M22*ddtheta2_ref + M23*ddtheta3_ref + M24*ddtheta4_ref)/( Gear * Ktn );
        i3_ref = (M31*ddtheta1_ref + M32*ddtheta2_ref + M33*ddtheta3_ref + M34*ddtheta4_ref)/( Gear * Ktn );
        i4_ref = (M41*ddtheta1_ref + M42*ddtheta2_ref + M43*ddtheta3_ref + M44*ddtheta4_ref)/( Gear * Ktn );

        #ifdef Enable_Identification
        // * When identifying F and D
        i1_ref = Gear * Gear * J1 * ddtheta1_ref / ( Gear * Ktn );
        i2_ref = Gear * Gear * J2 * ddtheta2_ref / ( Gear * Ktn );
        i3_ref = Gear * Gear * J3 * ddtheta3_ref / ( Gear * Ktn );
        i4_ref = Gear * Gear * J4 * ddtheta4_ref / ( Gear * Ktn );
        // * When identifying F and D
        #endif

        #ifdef Enable_DOB
        // tau_dis1_raw = Gear * Ktn * i1_ref - M11 * dtheta1_res;
        // tau_dis2_raw = Gear * Ktn * i2_ref - M22 * dtheta2_res;
        // tau_dis3_raw = Gear * Ktn * i3_ref - M33 * dtheta3_res;
        // tau_dis4_raw = Gear * Ktn * i4_ref - M44 * dtheta4_res;

        // tau_dob1 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob1_pre + G_DOB * dt * (tau_dis1_raw + tau_dis1_raw_pre) );
        // tau_dob2 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob2_pre + G_DOB * dt * (tau_dis2_raw + tau_dis2_raw_pre) );
        // tau_dob3 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob3_pre + G_DOB * dt * (tau_dis3_raw + tau_dis3_raw_pre) );
        // tau_dob4 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob4_pre + G_DOB * dt * (tau_dis4_raw + tau_dis4_raw_pre) );

        // tau_dis1_raw = Gear * Ktn * i1_ref + G_DOB * M11 * dtheta1_res;
        // tau_dis2_raw = Gear * Ktn * i2_ref + G_DOB * M22 * dtheta2_res;
        // tau_dis3_raw = Gear * Ktn * i3_ref + G_DOB * M33 * dtheta3_res;
        // tau_dis4_raw = Gear * Ktn * i4_ref + G_DOB * M44 * dtheta4_res;

        // tau_dob1 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob1_pre + G_DOB * dt * (tau_dis1_raw + tau_dis1_raw_pre) ) - G_DOB * M11 * dtheta1_res;
        // tau_dob2 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob2_pre + G_DOB * dt * (tau_dis2_raw + tau_dis2_raw_pre) ) - G_DOB * M22 * dtheta2_res;
        // tau_dob3 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob3_pre + G_DOB * dt * (tau_dis3_raw + tau_dis3_raw_pre) ) - G_DOB * M33 * dtheta3_res;
        // tau_dob4 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob4_pre + G_DOB * dt * (tau_dis4_raw + tau_dis4_raw_pre) ) - G_DOB * M44 * dtheta4_res;

        // * Bilinear Transform / Tustin Transform
        // tau_dob1 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob1_pre + G_DOB * dt * Gear * Ktn * ( ia1_ref + ia1_ref_pre ) - 2.0 * G_DOB * M11 * ( dtheta1_res - dtheta1_res_pre ) );
        // tau_dob2 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob2_pre + G_DOB * dt * Gear * Ktn * ( ia2_ref + ia2_ref_pre ) - 2.0 * G_DOB * M22 * ( dtheta2_res - dtheta2_res_pre ) );
        // tau_dob3 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob3_pre + G_DOB * dt * Gear * Ktn * ( ia3_ref + ia3_ref_pre ) - 2.0 * G_DOB * M33 * ( dtheta3_res - dtheta3_res_pre ) );
        // tau_dob4 = 1.0 / (2.0 + G_DOB * dt) * ( (2.0 - G_DOB * dt)*tau_dob4_pre + G_DOB * dt * Gear * Ktn * ( ia4_ref + ia4_ref_pre ) - 2.0 * G_DOB * M44 * ( dtheta4_res - dtheta4_res_pre ) );
        // * Bilinear Transform / Tustin Transform
        
        // * Backward Difference
        tau_dob1 = 1.0 / (1.0 + G_DOB * dt) * ( tau_dob1_pre + G_DOB * dt * Gear * Ktn * ia1_ref - G_DOB * M11 * ( dtheta1_res - dtheta1_res_pre ) );
        tau_dob2 = 1.0 / (1.0 + G_DOB * dt) * ( tau_dob2_pre + G_DOB * dt * Gear * Ktn * ia2_ref - G_DOB * M22 * ( dtheta2_res - dtheta2_res_pre ) );
        tau_dob3 = 1.0 / (1.0 + G_DOB * dt) * ( tau_dob3_pre + G_DOB * dt * Gear * Ktn * ia3_ref - G_DOB * M33 * ( dtheta3_res - dtheta3_res_pre ) );
        tau_dob4 = 1.0 / (1.0 + G_DOB * dt) * ( tau_dob4_pre + G_DOB * dt * Gear * Ktn * ia4_ref - G_DOB * M44 * ( dtheta4_res - dtheta4_res_pre ) );
        // * Backward Difference


          // * Save previous values
          tau_dob1_pre = tau_dob1;
          tau_dob2_pre = tau_dob2;
          tau_dob3_pre = tau_dob3;
          tau_dob4_pre = tau_dob4;

          // tau_dis1_raw_pre = tau_dis1_raw;
          // tau_dis2_raw_pre = tau_dis2_raw;
          // tau_dis3_raw_pre = tau_dis3_raw;
          // tau_dis4_raw_pre = tau_dis4_raw;

          ia1_ref_pre = ia1_ref;
          ia2_ref_pre = ia2_ref;
          ia3_ref_pre = ia3_ref;
          ia4_ref_pre = ia4_ref;
          // * Save previous values
        
        i1_comp = tau_dob1 / ( Gear*Ktn );
        i2_comp = tau_dob2 / ( Gear*Ktn );
        i3_comp = tau_dob3 / ( Gear*Ktn );
        i4_comp = tau_dob4 / ( Gear*Ktn );
        #endif

        #ifdef Enable_DFOB
        // if(direc1==0)
        tau_dfob1 = integral_tau_dfob1 - M11 * G_DFOB * dtheta1_res;// * Continuous
        tau_dfob2 = integral_tau_dfob2 - M22 * G_DFOB * dtheta2_res;// * Continuous
        tau_dfob3 = integral_tau_dfob3 - M33 * G_DFOB * dtheta3_res;// * Continuous
        tau_dfob4 = integral_tau_dfob4 - M44 * G_DFOB * dtheta4_res;// * Continuous

        switch(direc1){
          case 0:
            // tau_dfob1_raw = Gear * Ktn * i1_ref - M11 * dtheta1_res - F1_plus  - D1_plus  * dtheta1_res;
            // tau_dfob1_raw = Gear * Ktn * i1_ref + G_DFOB * M11 * dtheta1_res - F1_plus  - D1_plus  * dtheta1_res;
            // tau_dfob1 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob1_pre + G_DFOB * dt * ( Gear * Ktn * ia1_ref - F1_plus  - D1_plus  * dtheta1_res ) - G_DFOB*M11*(dtheta1_res - dtheta1_res_pre));// * Backward Difference
            integral_tau_dfob1 = integral_tau_dfob1 + ( Gear * Ktn * ia1_ref + M11 * G_DFOB * dtheta1_res - F1_plus  - D1_plus  * dtheta1_res - integral_tau_dfob1) * G_DFOB * dt;// * Continuous
            break;
          case 1:
            // tau_dfob1_raw = Gear * Ktn * i1_ref - M11 * dtheta1_res - F1_minus - D1_minus * dtheta1_res;
            // tau_dfob1_raw = Gear * Ktn * i1_ref + G_DFOB * M11 * dtheta1_res - F1_minus - D1_minus * dtheta1_res;
            // tau_dfob1 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob1_pre + G_DFOB * dt * ( Gear * Ktn * ia1_ref - F1_minus - D1_minus * dtheta1_res ) - G_DFOB*M11*(dtheta1_res - dtheta1_res_pre));
            integral_tau_dfob1 = integral_tau_dfob1 + ( Gear * Ktn * ia1_ref + M11 * G_DFOB * dtheta1_res - F1_minus  - D1_minus  * dtheta1_res - integral_tau_dfob1) * G_DFOB * dt;// * Continuous
            break;
        }
        switch(direc2){
          case 0:
            // tau_dfob2_raw = Gear * Ktn * i2_ref - M22 * dtheta2_res - F2_plus  - D2_plus  * dtheta2_res;
            // tau_dfob2_raw = Gear * Ktn * i2_ref + G_DFOB * M22 * dtheta2_res - F2_plus  - D2_plus  * dtheta2_res;
            // tau_dfob2 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob2_pre + G_DFOB * dt * ( Gear * Ktn * ia2_ref - F2_plus  - D2_plus  * dtheta2_res ) - G_DFOB*M22*(dtheta2_res - dtheta2_res_pre));
            integral_tau_dfob2 = integral_tau_dfob2 + ( Gear * Ktn * ia2_ref + M22 * G_DFOB * dtheta2_res - F2_plus  - D2_plus  * dtheta2_res - integral_tau_dfob2) * G_DFOB * dt;// * Continuous
            break;
          case 1:
            // tau_dfob2_raw = Gear * Ktn * i2_ref - M22 * dtheta2_res - F2_minus - D2_minus * dtheta2_res;
            // tau_dfob2_raw = Gear * Ktn * i2_ref + G_DFOB * M22 * dtheta2_res - F2_minus - D2_minus * dtheta2_res;
            // tau_dfob2 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob2_pre + G_DFOB * dt * ( Gear * Ktn * ia2_ref - F2_minus - D2_minus * dtheta2_res ) - G_DFOB*M22*(dtheta2_res - dtheta2_res_pre));
            integral_tau_dfob2 = integral_tau_dfob2 + ( Gear * Ktn * ia2_ref + M22 * G_DFOB * dtheta2_res - F2_minus  - D2_minus  * dtheta2_res - integral_tau_dfob2) * G_DFOB * dt;// * Continuous
            break;
        }
        switch(direc3){
          case 0:
            // tau_dfob3_raw = Gear * Ktn * i3_ref - M33 * dtheta3_res - F3_plus  - D3_plus  * dtheta3_res;
            // tau_dfob3_raw = Gear * Ktn * i3_ref + G_DFOB * M33 * dtheta3_res - F3_plus  - D3_plus  * dtheta3_res;
            // tau_dfob3 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob3_pre + G_DFOB * dt * ( Gear * Ktn * ia3_ref - F3_plus  - D3_plus  * dtheta3_res ) - G_DFOB*M33*(dtheta3_res - dtheta3_res_pre));
            integral_tau_dfob3 = integral_tau_dfob3 + ( Gear * Ktn * ia3_ref + M33 * G_DFOB * dtheta3_res - F3_plus  - D3_plus  * dtheta3_res - integral_tau_dfob3) * G_DFOB * dt;// * Continuous
            break;
          case 1:
            // tau_dfob3_raw = Gear * Ktn * i3_ref - M33 * dtheta3_res - F3_minus - D3_minus * dtheta3_res;
            // tau_dfob3_raw = Gear * Ktn * i3_ref + G_DFOB * M33 * dtheta3_res - F3_minus - D3_minus * dtheta3_res;
            // tau_dfob3 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob3_pre + G_DFOB * dt * ( Gear * Ktn * ia3_ref - F3_minus - D3_minus * dtheta3_res ) - G_DFOB*M33*(dtheta3_res - dtheta3_res_pre));
            integral_tau_dfob3 = integral_tau_dfob3 + ( Gear * Ktn * ia3_ref + M33 * G_DFOB * dtheta3_res - F3_minus  - D3_minus  * dtheta3_res - integral_tau_dfob3) * G_DFOB * dt;// * Continuous
            break;
        }
        switch(direc4){
          case 0:
            // tau_dfob4_raw = Gear * Ktn * i4_ref - M44 * dtheta4_res - F4_plus  - D4_plus  * dtheta4_res;
            // tau_dfob4_raw = Gear * Ktn * i4_ref + G_DFOB * M44 * dtheta4_res - F4_plus  - D4_plus  * dtheta4_res;
            // tau_dfob4 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob4_pre + G_DFOB * dt * ( Gear * Ktn * ia4_ref - F4_plus  - D4_plus  * dtheta4_res ) - G_DFOB*M44*(dtheta4_res - dtheta4_res_pre));
            integral_tau_dfob4 = integral_tau_dfob4 + ( Gear * Ktn * ia4_ref + M44 * G_DFOB * dtheta4_res - F4_plus  - D4_plus  * dtheta4_res - integral_tau_dfob4) * G_DFOB * dt;// * Continuous
            break;
          case 1:
            // tau_dfob4_raw = Gear * Ktn * i4_ref - M44 * dtheta4_res - F4_minus - D4_minus * dtheta4_res;
            // tau_dfob4_raw = Gear * Ktn * i4_ref + G_DFOB * M44 * dtheta4_res - F4_minus - D4_minus * dtheta4_res;
            // tau_dfob4 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob4_pre + G_DFOB * dt * ( Gear * Ktn * ia4_ref - F4_minus - D4_minus * dtheta4_res ) - G_DFOB*M44*(dtheta4_res - dtheta4_res_pre));
            integral_tau_dfob4 = integral_tau_dfob4 + ( Gear * Ktn * ia4_ref + M44 * G_DFOB * dtheta4_res - F4_minus  - D4_minus  * dtheta4_res - integral_tau_dfob4) * G_DFOB * dt;// * Continuous
            break;
        }

        // tau_dfob1 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob1_pre + G_DFOB * dt * (tau_dfob1_raw + tau_dfob1_raw_pre) );
        // tau_dfob2 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob2_pre + G_DFOB * dt * (tau_dfob2_raw + tau_dfob2_raw_pre) );
        // tau_dfob3 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob3_pre + G_DFOB * dt * (tau_dfob3_raw + tau_dfob3_raw_pre) );
        // tau_dfob4 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob4_pre + G_DFOB * dt * (tau_dfob4_raw + tau_dfob4_raw_pre) );

        // tau_dfob1 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob1_pre + G_DFOB * dt * (tau_dfob1_raw + tau_dfob1_raw_pre) ) - G_DFOB * M11 * dtheta1_res;
        // tau_dfob2 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob2_pre + G_DFOB * dt * (tau_dfob2_raw + tau_dfob2_raw_pre) ) - G_DFOB * M22 * dtheta2_res;
        // tau_dfob3 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob3_pre + G_DFOB * dt * (tau_dfob3_raw + tau_dfob3_raw_pre) ) - G_DFOB * M33 * dtheta3_res;
        // tau_dfob4 = 1.0 / (2.0 + G_DFOB * dt) * ( (2.0 - G_DFOB * dt)*tau_dfob4_pre + G_DFOB * dt * (tau_dfob4_raw + tau_dfob4_raw_pre) ) - G_DFOB * M44 * dtheta4_res;

          // * Save previous values
          tau_dfob1_pre = tau_dfob1;
          tau_dfob2_pre = tau_dfob2;
          tau_dfob3_pre = tau_dfob3;
          tau_dfob4_pre = tau_dfob4;

          // tau_dfob1_raw_pre = tau_dfob1_raw;
          // tau_dfob2_raw_pre = tau_dfob2_raw;
          // tau_dfob3_raw_pre = tau_dfob3_raw;
          // tau_dfob4_raw_pre = tau_dfob4_raw;
          // * Save previous values
        
        fd_hat1 = tau_dfob1 / Rw;// [N] Element of fd's wheel rotation direction
        fd_hat2 = tau_dfob2 / Rw;
        fd_hat3 = tau_dfob3 / Rw;
        fd_hat4 = tau_dfob4 / Rw;

        fx_hat = 1.0 / Rw             * (   tau_dfob1 - tau_dfob2 + tau_dfob3 - tau_dfob4 );
        fy_hat = 1.0 / Rw             * (   tau_dfob1 + tau_dfob2 + tau_dfob3 + tau_dfob4 );
        Mz_hat = 1.0 / Rw * ( L + W ) * ( - tau_dfob1 - tau_dfob2 + tau_dfob3 + tau_dfob4 );

        #endif

        ia1_ref = i1_ref + i1_comp;
        ia2_ref = i2_ref + i2_comp;
        ia3_ref = i3_ref + i3_comp;
        ia4_ref = i4_ref + i4_comp;

        if      (ia1_ref > i_max) ia1_ref =  i_max;
        else if(ia1_ref < -i_max) ia1_ref = -i_max;
        if      (ia2_ref > i_max) ia2_ref =  i_max;
        else if(ia2_ref < -i_max) ia2_ref = -i_max;
        if      (ia3_ref > i_max) ia3_ref =  i_max;
        else if(ia3_ref < -i_max) ia3_ref = -i_max;
        if      (ia4_ref > i_max) ia4_ref =  i_max;
        else if(ia4_ref < -i_max) ia4_ref = -i_max;

        PWM1 = (90.0 - 50.0)/100.0*PWM_rsl / i_max * ia1_ref + PWM_rsl * 0.5;
        PWM2 = (90.0 - 50.0)/100.0*PWM_rsl / i_max * ia2_ref + PWM_rsl * 0.5;
        PWM3 = (90.0 - 50.0)/100.0*PWM_rsl / i_max * ia3_ref + PWM_rsl * 0.5;
        PWM4 = (90.0 - 50.0)/100.0*PWM_rsl / i_max * ia4_ref + PWM_rsl * 0.5;

        // if(PWM1 >= PWM_rsl * 0.9){
        //   PWM1 = PWM_rsl * 0.87;
        // }
        // if(PWM2 >= PWM_rsl * 0.9){
        //   PWM2 = PWM_rsl * 0.87;
        // }
        // if(PWM3 >= PWM_rsl * 0.9){
        //   PWM3 = PWM_rsl * 0.87;
        // }
        // if(PWM4 >= PWM_rsl * 0.9){
        //   PWM4 = PWM_rsl * 0.87;
        // }

    		// PWM_constant = 0.1* PWM_rsl;
    
    		// PWM1 = PWM_constant;
    		// PWM2 = PWM_constant;
    		// PWM3 = PWM_constant;
    		// PWM4 = PWM_constant;

        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM1);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PWM2);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM3);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM4);

        // * Save previous values
        theta1_res_pre = theta1_res;
        theta2_res_pre = theta2_res;
        theta3_res_pre = theta3_res;
        theta4_res_pre = theta4_res;

        dtheta1_res_pre = dtheta1_res;
        dtheta2_res_pre = dtheta2_res;
        dtheta3_res_pre = dtheta3_res;
        dtheta4_res_pre = dtheta4_res;

        cnt1_pre = cnt1;
        cnt2_pre = cnt2;
        cnt3_pre = cnt3;
        cnt4_pre = cnt4;
        // * Save previous values

        // if(loop % 1000 == 0){
        // printf("%.3f, ", t);

    //		printf("%d, ", cnt1);
    //		printf("%d, ", cnt2);
    //		printf("%d, ", cnt3);
    //		printf("%d, ", cnt4);

    //		printf("%d, ", digit1);
    //		printf("%d, ", digit2);
    //		printf("%d, ", digit3);
    //		printf("%d, ", digit4);

    //		printf("%f, ", theta1_res);
    //		printf("%f, ", theta2_res);
    //		printf("%f, ", theta3_res);
    //		printf("%f, ", theta4_res);

    //		printf("%f, ", a);
    //		printf("%f, ", b);

    //		printf("%.8f, ", J1);
    //		printf("%f, ", M11);
    //		printf("%f, ", M12);
    //		printf("%f, ", M13);
    //		printf("%f, ", M14);

        // printf("%.3f, ", theta1_res);
        // printf("%.3f, ", theta2_res);
        // printf("%.3f, ", theta3_res);
        // printf("%.3f, ", theta4_res);

    //		printf("%.3f, ", dtheta1_res_raw);
    //		printf("%.3f, ", dtheta2_res_raw);
    //		printf("%.3f, ", dtheta3_res_raw);
    //		printf("%.3f, ", dtheta4_res_raw);

        // printf("%.3f, ", dtheta1_res);
        // printf("%.3f, ", dtheta2_res);
        // printf("%.3f, ", dtheta3_res);
        // printf("%.3f, ", dtheta4_res);

        // printf("%.3f, ", i1_ref);
        // printf("%.3f, ", i2_ref);
        // printf("%.3f, ", i3_ref);
        // printf("%.3f, ", i4_ref);

        // printf("%d, ", PWM1);
        // printf("%d, ", PWM2);
        // printf("%d, ", PWM3);
        // printf("%d, ", PWM4);
        // printf("\r\n");
        // }

        if(loop % 10 == 0 && i_save < N_SRAM){
          // if(loop % 1000 == 0){
          //   printf("save");
          //   printf("\r\n");
          // }
          t_SRAM[i_save] = t;

          dtheta1_res_SRAM[i_save] = dtheta1_res;
          dtheta2_res_SRAM[i_save] = dtheta2_res;
          dtheta3_res_SRAM[i_save] = dtheta3_res;
          dtheta4_res_SRAM[i_save] = dtheta4_res;

          theta1_res_SRAM[i_save] = theta1_res;
          theta2_res_SRAM[i_save] = theta2_res;
          theta3_res_SRAM[i_save] = theta3_res;
          theta4_res_SRAM[i_save] = theta4_res;

          ddtheta1_ref_SRAM[i_save] = ddtheta1_ref;
          ddtheta2_ref_SRAM[i_save] = ddtheta2_ref;
          ddtheta3_ref_SRAM[i_save] = ddtheta3_ref;
          ddtheta4_ref_SRAM[i_save] = ddtheta4_ref;

          i1_ref_SRAM[i_save] = i1_ref;
          i2_ref_SRAM[i_save] = i2_ref;
          i3_ref_SRAM[i_save] = i3_ref;
          i4_ref_SRAM[i_save] = i4_ref;

          ia1_ref_SRAM[i_save] = ia1_ref;
          ia2_ref_SRAM[i_save] = ia2_ref;
          ia3_ref_SRAM[i_save] = ia3_ref;
          ia4_ref_SRAM[i_save] = ia4_ref;

          PWM1_SRAM[i_save] = PWM1;
          PWM2_SRAM[i_save] = PWM2;
          PWM3_SRAM[i_save] = PWM3;
          PWM4_SRAM[i_save] = PWM4;

          // tau_dis1_raw_SRAM[i_save] = tau_dis1_raw;
          // tau_dis2_raw_SRAM[i_save] = tau_dis2_raw;
          // tau_dis3_raw_SRAM[i_save] = tau_dis3_raw;
          // tau_dis4_raw_SRAM[i_save] = tau_dis4_raw;

          fd1_ref_SRAM[i_save] = fd1_ref;
          fd2_ref_SRAM[i_save] = fd2_ref;
          fd3_ref_SRAM[i_save] = fd3_ref;
          fd4_ref_SRAM[i_save] = fd4_ref;

          Ki_df_integral1_SRAM[i_save] = Ki_df_integral1;
          Ki_df_integral2_SRAM[i_save] = Ki_df_integral2;
          Ki_df_integral3_SRAM[i_save] = Ki_df_integral3;
          Ki_df_integral4_SRAM[i_save] = Ki_df_integral4;

          tau_dob1_SRAM[i_save] = tau_dob1;
          tau_dob2_SRAM[i_save] = tau_dob2;
          tau_dob3_SRAM[i_save] = tau_dob3;
          tau_dob4_SRAM[i_save] = tau_dob4;

          // i1_comp_SRAM[i_save] = i1_comp;
          // i2_comp_SRAM[i_save] = i2_comp;
          // i3_comp_SRAM[i_save] = i3_comp;
          // i4_comp_SRAM[i_save] = i4_comp;

          tau_dfob1_SRAM[i_save] = tau_dfob1;
          tau_dfob2_SRAM[i_save] = tau_dfob2;
          tau_dfob3_SRAM[i_save] = tau_dfob3;
          tau_dfob4_SRAM[i_save] = tau_dfob4;

          i_save++;
        }

        loop = loop + 1;
        t = t + dt;
       
        break;
      case 2:

        break;
    }

		}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == GPIO_PIN_8){
		// if(mode == 0){
		// 	printf("EXTI Interrupt");
		// 	printf("%d, ", mode);
		// 	printf("\r\n");
		// }

		mode++;
		printf("%d, ", mode);
		printf("\r\n");

    divide = mode % 3;

    switch(divide){
      case 0:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET); // Green
        break;
      case 1:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_SET); // Blue
        break;
      case 2:
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7,  GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);// Red
        break;
    }

    switch(mode){
      case 0:
        // printf("0, %d, ", mode);
        // printf("\r\n");
        break;
      case 1:
        // printf("1, %d, ", mode);
        // printf("\r\n");

        tau_dfob1 = 0.0;
        tau_dfob2 = 0.0;
        tau_dfob3 = 0.0;
        tau_dfob4 = 0.0;

        tau_dfob1_pre = 0.0;
        tau_dfob1_pre = 0.0;
        tau_dfob1_pre = 0.0;
        tau_dfob1_pre = 0.0;

        break;
      case 2:
        // printf("2, %d, ", mode);
        // printf("\r\n");

        // outputfile = fopen("C:\\Users\\TATSUMI\\STM32CubeIDE\\workspace_1.4.0\\1128_4.txt", "w+");

        // if( outputfile == NULL ){
        //   printf("fail \r\n");
        //   exit(1);
        // }
        // outputfile = fopen("C:\\Users\\TATSUMI\\STM32CubeIDE\\workspace_1.4.0\\1109_4.txt", "w");
        // fprintf(outputfile, "abc");
        // fclose(outputfile);

        PWM1 = 0.5*PWM_rsl;// Stop motor
        PWM2 = 0.5*PWM_rsl;
        PWM3 = 0.5*PWM_rsl;
        PWM4 = 0.5*PWM_rsl;

        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, PWM1);
        __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, PWM2);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, PWM3);
        __HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_2, PWM4);

        for( i_output=0; i_output<N_SRAM; i_output++ ){
          printf("%d, ", i_output);

          printf("%f, ", t_SRAM[i_output]);

          printf("%f, ", dtheta1_res_SRAM[i_output]);
          printf("%f, ", dtheta2_res_SRAM[i_output]);
          printf("%f, ", dtheta3_res_SRAM[i_output]);
          printf("%f, ", dtheta4_res_SRAM[i_output]);

          printf("%f, ", theta1_res_SRAM[i_output]);
          printf("%f, ", theta2_res_SRAM[i_output]);
          printf("%f, ", theta3_res_SRAM[i_output]);
          printf("%f, ", theta4_res_SRAM[i_output]);

          printf("%f, ", ddtheta1_ref_SRAM[i_output]);
          printf("%f, ", ddtheta2_ref_SRAM[i_output]);
          printf("%f, ", ddtheta3_ref_SRAM[i_output]);
          printf("%f, ", ddtheta4_ref_SRAM[i_output]);

          printf("%f, ", i1_ref_SRAM[i_output]);
          printf("%f, ", i2_ref_SRAM[i_output]);
          printf("%f, ", i3_ref_SRAM[i_output]);
          printf("%f, ", i4_ref_SRAM[i_output]);

          printf("%f, ", ia1_ref_SRAM[i_output]);
          printf("%f, ", ia2_ref_SRAM[i_output]);
          printf("%f, ", ia3_ref_SRAM[i_output]);
          printf("%f, ", ia4_ref_SRAM[i_output]);

          printf("%d, ", PWM1_SRAM[i_output]);
          printf("%d, ", PWM2_SRAM[i_output]);
          printf("%d, ", PWM3_SRAM[i_output]);
          printf("%d, ", PWM4_SRAM[i_output]);

          // printf("%f, ", tau_dis1_raw_SRAM[i_output]);
          // printf("%f, ", tau_dis2_raw_SRAM[i_output]);
          // printf("%f, ", tau_dis3_raw_SRAM[i_output]);
          // printf("%f, ", tau_dis4_raw_SRAM[i_output]);

          printf("%f, ", fd1_ref_SRAM[i_output]);
          printf("%f, ", fd2_ref_SRAM[i_output]);
          printf("%f, ", fd3_ref_SRAM[i_output]);
          printf("%f, ", fd4_ref_SRAM[i_output]);
          
          printf("%f, ", Ki_df_integral1_SRAM[i_output]);
          printf("%f, ", Ki_df_integral2_SRAM[i_output]);
          printf("%f, ", Ki_df_integral3_SRAM[i_output]);
          printf("%f, ", Ki_df_integral4_SRAM[i_output]);
          
          printf("%f, ", tau_dob1_SRAM[i_output]);
          printf("%f, ", tau_dob2_SRAM[i_output]);
          printf("%f, ", tau_dob3_SRAM[i_output]);
          printf("%f, ", tau_dob4_SRAM[i_output]);

          printf("%f, ", tau_dfob1_SRAM[i_output]);
          printf("%f, ", tau_dfob2_SRAM[i_output]);
          printf("%f, ", tau_dfob3_SRAM[i_output]);
          printf("%f, ", tau_dfob4_SRAM[i_output]);

          printf("\r\n");
        }
        break;
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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM5_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  TIM1->CNT = cnt_offset;
  TIM3->CNT = cnt_offset;
  TIM4->CNT = cnt_offset;
  TIM8->CNT = cnt_offset;

  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET); // Green

//  printf("\r\n initialized Success!!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 96-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000-1;
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
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 1-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim8, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 1-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 4000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_MDC_Pin RMII_RXD0_Pin RMII_RXD1_Pin */
  GPIO_InitStruct.Pin = RMII_MDC_Pin|RMII_RXD0_Pin|RMII_RXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_REF_CLK_Pin RMII_MDIO_Pin RMII_CRS_DV_Pin */
  GPIO_InitStruct.Pin = RMII_REF_CLK_Pin|RMII_MDIO_Pin|RMII_CRS_DV_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RMII_TXD1_Pin */
  GPIO_InitStruct.Pin = RMII_TXD1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(RMII_TXD1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PG6 */
  GPIO_InitStruct.Pin = GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : RMII_TX_EN_Pin RMII_TXD0_Pin */
  GPIO_InitStruct.Pin = RMII_TX_EN_Pin|RMII_TXD0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
