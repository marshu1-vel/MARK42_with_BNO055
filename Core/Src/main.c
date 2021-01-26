/* USER CODE BEGIN Header */
//! ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// #define angular_velocity_control
#define Enable_DOB
#define Enable_DFOB
#define Enable_WOB
// #define Enable_WOB_FB
#define Disable_WOB_FB
// #define Enable_PD_controller_av
// #define Enable_Vehicle_Velocity_control
// #define Enable_Driving_force_FB
#define Enable_Driving_Force_Control_Jointspace_Part // This part is common to Driving Force Control and Driving Force Distribution Control
// #define Enable_Driving_Force_Control
#define Enable_Driving_Force_Distribution_Control
#define Enable_I2C
// #define Enable_Inertia_Identification
#define Enable_Inertia_Mass_Matrix_by_Lagrange
// #define Enable_Slip_Ratio_Observer
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
#include "bno055_stm32.h"
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

I2C_HandleTypeDef hi2c1;

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
// const float pi = 3.14159265358979;
#define pi 3.14159265358979f
uint16_t loop = 0;
uint8_t mode = 0;
uint8_t divide = 0;// Remainder when mode is divided by 3.
uint8_t isFirst = 0;

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

#define G_LPF      50.0f // [rad/sec] : Up to half of sampling frequency 200.0 50.0 20.0
#define G_LPF_ddth 30.0f // [rad/sec] : For pseudo derivative when calculating angular acceleration

float ddtheta1_res = 0.0;
float ddtheta2_res = 0.0;
float ddtheta3_res = 0.0;
float ddtheta4_res = 0.0;

float ddtheta1_res_pre = 0.0;
float ddtheta2_res_pre = 0.0;
float ddtheta3_res_pre = 0.0;
float ddtheta4_res_pre = 0.0;

// float dtheta1_res_raw = 0.0;// [rad/sec]
// float dtheta2_res_raw = 0.0;
// float dtheta3_res_raw = 0.0;
// float dtheta4_res_raw = 0.0;

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
// const float D1_plus  =  0.0011; // Viscous friction coefficient [Nm*sec/rad]
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
// const float D1_plus  =  0.0004; // Viscous friction coefficient [Nm*sec/rad]
// const float D1_minus = 0.0003;

// const float D2_plus  =  -0.0006;
// const float D2_minus = -0.0003;

// const float D3_plus  =  0.0042;
// const float D3_minus = 0.0027;

// const float D4_plus  =  -0.0005;
// const float D4_minus = -0.000004;

// const float F1_plus  =  0.0324;// Coulomb friction torque [Nm]
// const float F1_minus = -0.0312;

// const float F2_plus  =  0.0385;
// const float F2_minus = -0.0357;

// const float F3_plus  =  0.0352;
// const float F3_minus = -0.0355;

// const float F4_plus  =  0.0582;
// const float F4_minus = -0.0525;


// const float F1_plus  =  0.0;// Coulomb friction torque [Nm]
// const float F1_minus = 0.0;

// const float F2_plus  =  0.0;
// const float F2_minus = 0.0;

// const float F3_plus  =  0.0;
// const float F3_minus = 0.0;

// const float F4_plus  =  0.0;
// const float F4_minus = 0.0;

// const float F1_plus  =  0.0158;// Coulomb friction torque [Nm]
// const float F1_minus = -0.0163;
// const float D1_plus  =  -0.002; // Viscous friction coefficient [Nm*sec/rad]
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
// * 2020/11/14


// * 2020/12/11 : First, before acceleration experiment
// const float D1_plus  = -0.00003; // Viscous friction coefficient [Nm*sec/rad]
// const float D1_minus =  0.0003;

// const float D2_plus  = -0.0002;
// const float D2_minus =  0.00003;

// const float D3_plus  = 0.0001;
// const float D3_minus = 0.0015;

// const float D4_plus  = 0.0011;
// const float D4_minus = 0.001;

// const float F1_plus  =  0.01794;// Coulomb friction torque [Nm]
// const float F1_minus = -0.0448;

// const float F2_plus  =  0.0278;
// const float F2_minus = -0.05695;

// const float F3_plus  =  0.0366;
// const float F3_minus = -0.0695;

// const float F4_plus  =  0.0263;
// const float F4_minus = -0.0655;
// * 2020/12/11


// * 2020/12/12 : Second, utilize inertia( Gear * Gear * J ) (not utilize inertia by acceleration experiment)
const float D1_plus  = 0.0005; // Viscous friction coefficient [Nm*sec/rad]
const float D1_minus = 0.0006;

const float D2_plus  = 0.0004;
const float D2_minus = 0.000007;

const float D3_plus  = 0.0011;
const float D3_minus = 0.0021;

const float D4_plus  = 0.0012;
const float D4_minus = 0.001;

const float F1_plus  =  0.0173;// Coulomb friction torque [Nm]
const float F1_minus = -0.0424;

const float F2_plus  =  0.0206;
const float F2_minus = -0.052518;

const float F3_plus  =  0.0391;
const float F3_minus = -0.0633;

const float F4_plus  =  0.0276;
const float F4_minus = -0.0664;
// * 2020/12/12

#ifdef Enable_Inertia_Mass_Matrix_by_Lagrange
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
#endif

#ifdef Enable_Inertia_Identification
const float M11 = Gear * Gear * J1;// 0.00233472
const float M22 = Gear * Gear * J2;
const float M33 = Gear * Gear * J3;
const float M44 = Gear * Gear * J4;

// * First identification
// const float M11 = 0.002797896;// 0.00233472
// const float M22 = 0.00284852;
// const float M33 = 0.003269981;
// const float M44 = 0.002736898;

// * Second
// const float M11 = 0.00348393;// 0.00233472
// const float M22 = 0.003551489;
// const float M33 = 0.003543495;
// const float M44 = 0.003475158;
#endif


// * Motor Parameters


// * Control Gains etc.
#define Kp_av    20.0f// Gain for angular velocity control 100.0
#define Kp_av_df 10.0f//10.0f// Gain for Driving Force    control 10.0
#define Kp_av_4 5.0f
float Kd_av = 0.08;//0.05;//2.0 * sqrt(Kp_av);
#define G_LPF_D_av 50.0f// D controller of angular velocity control

float PD_controller1_av = 0.0;
float PD_controller2_av = 0.0;
float PD_controller3_av = 0.0;
float PD_controller4_av = 0.0;
float PD_controller1_av_pre = 0.0;
float PD_controller2_av_pre = 0.0;
float PD_controller3_av_pre = 0.0;
float PD_controller4_av_pre = 0.0;

float delta_dtheta1 = 0.0;
float delta_dtheta2 = 0.0;
float delta_dtheta3 = 0.0;
float delta_dtheta4 = 0.0;

float delta_dtheta1_pre = 0.0;
float delta_dtheta2_pre = 0.0;
float delta_dtheta3_pre = 0.0;
float delta_dtheta4_pre = 0.0;

// float Kp_vv_x   = 5.0;
// float Kp_vv_y   = 5.0;
// float Kp_vv_phi = 5.0;
#define Kp_vv_x 1.75f//5.0f // Gain for vehicle velocity control(Based on encoder) 10.0
#define Kp_vv_y 1.75f//5.0f
#define Kp_vv_phi 2.3f//1.75f//5.0f as of 2021/01/17 1.75 -> 2.3

float ddx_ref = 0.0;
float ddy_ref = 0.0;
float ddphi_ref = 0.0;

#define Kp_df_x 100.0f//0.1f 0.5 10.0 50.0
#define Kp_df_y 100.0f//0.1f 0.5
#define Kp_df_phi 900.0f//10000.0f//0.1f 5.0 100.0(1115-36) : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)

#define Kp_df 0.005f//0.005f//5000.0f//1.2f//0.2f
#define Ki_df 0.01f // Ki Gain for driving force control 10.0 0.1 1.0 0.1 0.018
float fx_ref = 0.0;
float fy_ref = 0.0;
float Mz_ref = 0.0;

float fd1_ref = 0.0;
float fd2_ref = 0.0;
float fd3_ref = 0.0;
float fd4_ref = 0.0;

float fd1_ref_normal = 0.0;
float fd2_ref_normal = 0.0;
float fd3_ref_normal = 0.0;
float fd4_ref_normal = 0.0;

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

// float tau_dis1_raw = 0.0;// Raw data (Without LPF)
// float tau_dis2_raw = 0.0;
// float tau_dis3_raw = 0.0;
// float tau_dis4_raw = 0.0;

// float tau_dis1_raw_pre = 0.0;
// float tau_dis2_raw_pre = 0.0;
// float tau_dis3_raw_pre = 0.0;
// float tau_dis4_raw_pre = 0.0;

float i1_comp = 0.0;
float i2_comp = 0.0;
float i3_comp = 0.0;
float i4_comp = 0.0;
// * DOB


// * DFOB
#define G_DFOB 50.0f//20.0f // [rad/sec] 50.0 30.0
float tau_dfob1 = 0.0;
float tau_dfob2 = 0.0;
float tau_dfob3 = 0.0;
float tau_dfob4 = 0.0;

float tau_dfob1_pre = 0.0;
float tau_dfob2_pre = 0.0;
float tau_dfob3_pre = 0.0;
float tau_dfob4_pre = 0.0;

// float tau_dfob1_raw = 0.0;// Raw data (Without LPF)
// float tau_dfob2_raw = 0.0;
// float tau_dfob3_raw = 0.0;
// float tau_dfob4_raw = 0.0;

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

float tau_fric1 = 0.0;
float tau_fric2 = 0.0;
float tau_fric3 = 0.0;
float tau_fric4 = 0.0;
// * DFOB


// * For Driving Force Distribution Control
float w1 = 0.0;
float w2 = 0.0;
float w3 = 0.0;
float w4 = 0.0;

#define epsilon 0.01f//0.1f//0.01f// 0.001f

float alpha_1 = 0.0;
float alpha_2 = 0.0;
float alpha_3 = 0.0;
float alpha_4 = 0.0;

float v1_x = 0.0;
float v2_x = 0.0;
float v3_x = 0.0;
float v4_x = 0.0;

float v1_y = 0.0;
float v2_y = 0.0;
float v3_y = 0.0;
float v4_y = 0.0;
// * For Driving Force Distribution Control


// * Fixed Trace Algorithm, FTA
float alpha_1_hat = 0.0;
float alpha_2_hat = 0.0;
float alpha_3_hat = 0.0;
float alpha_4_hat = 0.0;

// float alpha_1_hat_pre = 0.0;
// float alpha_2_hat_pre = 0.0;
// float alpha_3_hat_pre = 0.0;
// float alpha_4_hat_pre = 0.0;

float tan_alpha_1_hat = 0.0;
float tan_alpha_2_hat = 0.0;
float tan_alpha_3_hat = 0.0;
float tan_alpha_4_hat = 0.0;

float tan_beta_1_hat = 0.0;
float tan_beta_2_hat = 0.0;
float tan_beta_3_hat = 0.0;
float tan_beta_4_hat = 0.0;

float tan_beta_1_hat_pre = 0.0;
float tan_beta_2_hat_pre = 0.0;
float tan_beta_3_hat_pre = 0.0;
float tan_beta_4_hat_pre = 0.0;

// #define Covariance_initial 100000.0f//50000.0f//10000.0f//5000.0f//1000.0f
#define Gamma 0.1f//0.3f//1000.0f//0.1f// 10000.0f

float P1_k = Gamma;//Covariance_initial;// Covariance matrix at k
float P2_k = Gamma;//Covariance_initial;
float P3_k = Gamma;//Covariance_initial;
float P4_k = Gamma;//Covariance_initial;

float P1_k_1 = Gamma;//Covariance_initial;// Covariance matrix at k - 1
float P2_k_1 = Gamma;//Covariance_initial;
float P3_k_1 = Gamma;//Covariance_initial;
float P4_k_1 = Gamma;//Covariance_initial;

float Kappa_1 = 0.0;
float Kappa_2 = 0.0;
float Kappa_3 = 0.0;
float Kappa_4 = 0.0;
// * Fixed Trace Algorithm, FTA


// * IMU
bno055_vector_t Euler;
bno055_vector_t Gyro;
bno055_vector_t Acc;       // Without Fusion
bno055_vector_t Acc_Linear;// With Fusion

float yaw = 0.0;
float roll = 0.0;
float pitch = 0.0;

float yaw_initial = pi / 2.0;
float yaw_pre = 0.0;
float yaw_digit = 0.0;

float yaw_rate = 0.0;// [rad/sec]
float roll_rate = 0.0;
float pitch_rate = 0.0;

float yaw_rate_pre  = 0.0;
// float yaw_rate_pre2 = 0.0;

// float yaw_rate_notch      = 0.0;
// float yaw_rate_notch_pre  = 0.0;
// float yaw_rate_notch_pre2 = 0.0;

float d_yawrate     = 0.0;// [rad/sec^2] ddphi_res : angular acceleration
float d_yawrate_pre = 0.0;

// #define zeta1 1.0f // Inverse of Q value
// #define N_roller 9.0f // Number of free roller
// float G_notch1 = 90.0; // [rad/sec]

float Acc_x = 0.0;// Sensor coordinate system
float Acc_y = 0.0;
float Acc_z = 0.0;

float Acc_x_correct = 0.0;// Space(Absolute) coordinate system
float Acc_y_correct = 0.0;
float Acc_z_correct = 0.0;

float Acc_x_correct_pre = 0.0;// Space(Absolute) coordinate system
float Acc_y_correct_pre = 0.0;
float Acc_z_correct_pre = 0.0;

#define G_LPF_acc  300.0f // [rad/sec]
#define G_LPF_gyro 50.0f //150.0f // [rad/sec]
#define G_HPF_acc  100.0f // [rad/sec]

float Acc_x_LPF = 0.0;
float Acc_y_LPF = 0.0;
float Acc_z_LPF = 0.0;

float Acc_x_LPF_pre = 0.0;
float Acc_y_LPF_pre = 0.0;
float Acc_z_LPF_pre = 0.0;
// * IMU


// * Slip Ratio
#ifdef Enable_Slip_Ratio_Observer
#define epsilon 0.05f//0.01f//0.001f // Prevent division by 0
#define epsilon_acc 0.1f//0.01f//0.001f // Prevent division by 0

float lambda_1_hat = 0.0;// Estimated slip ratio when just encoder is utilized
float lambda_2_hat = 0.0;
float lambda_3_hat = 0.0;
float lambda_4_hat = 0.0;

float d_lambda_1_hat = 0.0;
float d_lambda_2_hat = 0.0;
float d_lambda_3_hat = 0.0;
float d_lambda_4_hat = 0.0;

float dv_1 = 0.0;
float dv_2 = 0.0;
float dv_3 = 0.0;
float dv_4 = 0.0;

float dv_1_pre = 0.0;
float dv_2_pre = 0.0;
float dv_3_pre = 0.0;
float dv_4_pre = 0.0;

float v1_hat = 0.0;
float v2_hat = 0.0;
float v3_hat = 0.0;
float v4_hat = 0.0;

#define G_LPF_dv 100.0f // [rad/sec]

float dv_1_LPF = 0.0;
float dv_2_LPF = 0.0;
float dv_3_LPF = 0.0;
float dv_4_LPF = 0.0;

float dv_1_LPF_pre = 0.0;
float dv_2_LPF_pre = 0.0;
float dv_3_LPF_pre = 0.0;
float dv_4_LPF_pre = 0.0;

float delta_dv = 0.0;

float lambda_1_hat_acc = 0.0;// Estimated slip ratio when encoder and acceleration are utilized
float lambda_2_hat_acc = 0.0;
float lambda_3_hat_acc = 0.0;
float lambda_4_hat_acc = 0.0;

float d_lambda_1_hat_acc = 0.0;
float d_lambda_2_hat_acc = 0.0;
float d_lambda_3_hat_acc = 0.0;
float d_lambda_4_hat_acc = 0.0;

float dv_1_acc = 0.0;
float dv_2_acc = 0.0;
float dv_3_acc = 0.0;
float dv_4_acc = 0.0;

float v1_hat_acc = 0.0;
float v2_hat_acc = 0.0;
float v3_hat_acc = 0.0;
float v4_hat_acc = 0.0;
#endif
// * Slip Ratio

// * WOB
#define G_WOB 50.0f//1.0f//0.01f // [rad/sec]

#ifdef Enable_WOB_FB
#define WOB_FB 1.0f // 1.0 : With feedback
#endif

#ifdef Disable_WOB_FB
#define WOB_FB 0.0f // 0 : No feedback
#endif
float WOB_x_input = 0.0;
float WOB_y_input = 0.0;
// float WOB_phi_input = 0.0;

// float WOB_x_input_pre = 0.0;
// float WOB_y_input_pre = 0.0;
// float WOB_phi_input_pre = 0.0;

float Fx_dis = 0.0;// [N]
float Fy_dis = 0.0;
float ddphi_dis = 0.0;// [rad/sec^2] For WOB Acceleration Dimension
float Mz_dis = 0.0;// [Nm] For WOB Force Dimension

float Fx_dis_pre = 0.0;
float Fy_dis_pre = 0.0;
float ddphi_dis_pre = 0.0;
// float Mz_dis_pre = 0.0;
// * WOB


// * YMO ( Yaw Moment Observer )
#define G_YMO 50.0f // [rad/sec]
float M_YMO = 0.0;
float M_YMO_pre = 0.0;
// * YMO ( Yaw Moment Observer )


// * Save variables in SRAM
#define N_SRAM 1500 // Sampling Number of variables in SRAM (Number of array) // 3000 // About 50 variables : Up to 2500 sampling -> Set 2200 for safety
// #define N_SRAM 1100
// float t_experiment = N_SRAM / 100.0;
#define t_experiment N_SRAM / 100.0f

int i_save = 0;  // For "for sentences"
int i_output = 0;// For displaying data after experiment

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

float ddtheta1_res_SRAM[N_SRAM] = {};
float ddtheta2_res_SRAM[N_SRAM] = {};
float ddtheta3_res_SRAM[N_SRAM] = {};
float ddtheta4_res_SRAM[N_SRAM] = {};

// float i1_ref_SRAM[N_SRAM] = {};
// float i2_ref_SRAM[N_SRAM] = {};
// float i3_ref_SRAM[N_SRAM] = {};
// float i4_ref_SRAM[N_SRAM] = {};

float ia1_ref_SRAM[N_SRAM] = {};
float ia2_ref_SRAM[N_SRAM] = {};
float ia3_ref_SRAM[N_SRAM] = {};
float ia4_ref_SRAM[N_SRAM] = {};

uint16_t PWM1_SRAM[N_SRAM] = {};
uint16_t PWM2_SRAM[N_SRAM] = {};
uint16_t PWM3_SRAM[N_SRAM] = {};
uint16_t PWM4_SRAM[N_SRAM] = {};

float fd1_ref_normal_SRAM[N_SRAM] = {};// Added : as of 2021/01/16
float fd1_ref_SRAM[N_SRAM] = {};

float fd2_ref_normal_SRAM[N_SRAM] = {};
float fd2_ref_SRAM[N_SRAM] = {};

float fd3_ref_normal_SRAM[N_SRAM] = {};
float fd3_ref_SRAM[N_SRAM] = {};

float fd4_ref_normal_SRAM[N_SRAM] = {};
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

float yaw_SRAM[N_SRAM] = {};
float roll_SRAM[N_SRAM] = {};
float pitch_SRAM[N_SRAM] = {};

float yaw_rate_SRAM[N_SRAM] = {};
float roll_rate_SRAM[N_SRAM] = {};
float pitch_rate_SRAM[N_SRAM] = {};

// float yaw_rate_notch_SRAM[N_SRAM] = {};

float Acc_x_SRAM[N_SRAM] = {};
float Acc_y_SRAM[N_SRAM] = {};
float Acc_z_SRAM[N_SRAM] = {};

float Acc_x_correct_SRAM[N_SRAM] = {};
float Acc_y_correct_SRAM[N_SRAM] = {};
float Acc_z_correct_SRAM[N_SRAM] = {};

float Acc_x_LPF_SRAM[N_SRAM] = {};
float Acc_y_LPF_SRAM[N_SRAM] = {};
// float Acc_z_LPF_SRAM[N_SRAM] = {};
float d_yawrate_SRAM[N_SRAM] = {};

#ifdef Enable_Slip_Ratio_Observer
float lambda_1_hat_SRAM[N_SRAM] = {};
float lambda_2_hat_SRAM[N_SRAM] = {};
float lambda_3_hat_SRAM[N_SRAM] = {};
float lambda_4_hat_SRAM[N_SRAM] = {};

float lambda_1_hat_acc_SRAM[N_SRAM] = {};
float lambda_2_hat_acc_SRAM[N_SRAM] = {};
float lambda_3_hat_acc_SRAM[N_SRAM] = {};
float lambda_4_hat_acc_SRAM[N_SRAM] = {};

float d_lambda_1_hat_SRAM[N_SRAM] = {};
float d_lambda_2_hat_SRAM[N_SRAM] = {};
float d_lambda_3_hat_SRAM[N_SRAM] = {};
float d_lambda_4_hat_SRAM[N_SRAM] = {};

float d_lambda_1_hat_acc_SRAM[N_SRAM] = {};
float d_lambda_2_hat_acc_SRAM[N_SRAM] = {};
float d_lambda_3_hat_acc_SRAM[N_SRAM] = {};
float d_lambda_4_hat_acc_SRAM[N_SRAM] = {};

float v1_hat_SRAM[N_SRAM] = {};
float v2_hat_SRAM[N_SRAM] = {};
float v3_hat_SRAM[N_SRAM] = {};
float v4_hat_SRAM[N_SRAM] = {};

float v1_hat_acc_SRAM[N_SRAM] = {};
float v2_hat_acc_SRAM[N_SRAM] = {};
float v3_hat_acc_SRAM[N_SRAM] = {};
float v4_hat_acc_SRAM[N_SRAM] = {};
#endif

float Fx_dis_SRAM[N_SRAM] = {};
float Fy_dis_SRAM[N_SRAM] = {};
float Mz_dis_SRAM[N_SRAM] = {};

float M_YMO_SRAM[N_SRAM] = {};

float alpha_1_SRAM[N_SRAM] = {};
float alpha_2_SRAM[N_SRAM] = {};
float alpha_3_SRAM[N_SRAM] = {};
float alpha_4_SRAM[N_SRAM] = {};

float alpha_1_hat_SRAM[N_SRAM] = {};
float alpha_2_hat_SRAM[N_SRAM] = {};
float alpha_3_hat_SRAM[N_SRAM] = {};
float alpha_4_hat_SRAM[N_SRAM] = {};

float tan_beta_1_hat_SRAM[N_SRAM] = {};
float tan_beta_2_hat_SRAM[N_SRAM] = {};
float tan_beta_3_hat_SRAM[N_SRAM] = {};
float tan_beta_4_hat_SRAM[N_SRAM] = {};

float Kappa_1_SRAM[N_SRAM] = {};
float Kappa_2_SRAM[N_SRAM] = {};
float Kappa_3_SRAM[N_SRAM] = {};
float Kappa_4_SRAM[N_SRAM] = {};
// * Save variables in SRAM

// * Command
float omega = 0.0;// [rad/sec] 
float r = 0.0;// [m] : Turning radius of steady circle turning
float A = 0.0;// [m] : Amplitude of sin wave movement
// * Command

// * RLS with forgetting factor (Estimate Jacobi matrix T_hat)
// float T_hat_11 = 0.0;
// float T_hat_12 = 0.0;

// float T_hat_11_Z1 = 0.0;
// float T_hat_12_Z1 = 0.0;

// float P_11    = 0.0;
// float P_11_Z1 = 0.0;
// * RLS with forgetting factor (Estimate Jacobi matrix T_hat)


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
static void MX_I2C1_Init(void);
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
        
        // * IMU
        yaw   = Euler.x;// [degree]
        roll  = Euler.y;
        pitch = Euler.z;

        yaw_rate   = Gyro.z;// [dps : degree/sec]
        roll_rate  = Gyro.y;//Gyro.x;
        pitch_rate = Gyro.x;//Gyro.y;

        Acc_x = -Acc.x;// Adjust these values to Vehicle coordinate system of modeling
        Acc_y = -Acc.y;// Direction is opposite, due to inertial force
        Acc_z = -Acc.z;// Add minus - : as of 2021/01/09

        yaw   = -yaw   * 2.0 * pi / 360.0;// [rad] Convert degree to rad
        roll  = -roll  * 2.0 * pi / 360.0;
        pitch = -pitch * 2.0 * pi / 360.0;

        yaw_rate   = yaw_rate   * 2.0 * pi / 360.0;// [rad/sec] // ! Direction is not confirmed yet.
        roll_rate  = roll_rate  * 2.0 * pi / 360.0;
        pitch_rate = pitch_rate * 2.0 * pi / 360.0;

        // Acc_x_correct = cos(roll)*cos(pitch)                                *Acc_x + sin(roll)*cos(pitch)                                *Acc_y - sin(pitch)         *Acc_z;
        // Acc_y_correct = ( cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) )*Acc_x + ( sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) )*Acc_y + cos(pitch)*sin(yaw)*Acc_z;
        // Acc_z_correct = ( cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) )*Acc_x + ( sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) )*Acc_y + cos(pitch)*cos(yaw)*Acc_z;

        // Acc_y_correct = cos(roll)*cos(pitch)                                *Acc_y + sin(roll)*cos(pitch)                                *(-Acc_x) - sin(pitch)         *Acc_z;
        // Acc_x_correct = ( cos(roll)*sin(pitch)*sin(yaw)-sin(roll)*cos(yaw) )*Acc_y + ( sin(roll)*sin(pitch)*sin(yaw)+cos(roll)*cos(yaw) )*(-Acc_x) + cos(pitch)*sin(yaw)*Acc_z;
        // Acc_x_correct = - Acc_x_correct;
        // Acc_z_correct = ( cos(roll)*sin(pitch)*cos(yaw)+sin(roll)*sin(yaw) )*Acc_y + ( sin(roll)*sin(pitch)*cos(yaw)-cos(roll)*sin(yaw) )*(-Acc_x) + cos(pitch)*cos(yaw)*Acc_z;

        // Acc_y_correct = cos(roll)*cos(pitch)                                *Acc_y + sin(roll)*cos(pitch)                                *(-Acc_x) - sin(pitch)         *Acc_z;
        // Acc_x_correct = ( cos(roll)*sin(pitch)*sin(0.0)-sin(roll)*cos(0.0) )*Acc_y + ( sin(roll)*sin(pitch)*sin(0.0)+cos(roll)*cos(0.0) )*(-Acc_x) + cos(pitch)*sin(0.0)*Acc_z;
        // Acc_x_correct = - Acc_x_correct;
        // Acc_z_correct = ( cos(roll)*sin(pitch)*cos(0.0)+sin(roll)*sin(0.0) )*Acc_y + ( sin(roll)*sin(pitch)*cos(0.0)-cos(roll)*sin(0.0) )*(-Acc_x) + cos(pitch)*cos(0.0)*Acc_z;

        Acc_y_correct = cos(roll)*cos(pitch)                                *Acc_y + sin(roll)*cos(pitch)                                *(-Acc_x) - sin(pitch)         *Acc_z;
        Acc_x_correct = -sin(roll)*Acc_y +  cos(roll)*(-Acc_x);
        Acc_x_correct = - Acc_x_correct;
        Acc_z_correct = cos(roll)*sin(pitch)*Acc_y + sin(roll)*sin(pitch)*(-Acc_x) + cos(pitch)*Acc_z;

        Acc_x_LPF = 1.0 / (2.0 + G_LPF_acc * dt) * ( (2.0 - G_LPF_acc * dt) * Acc_x_LPF_pre + G_LPF_acc * dt * ( Acc_x_correct + Acc_x_correct_pre ) );// LPF
        Acc_y_LPF = 1.0 / (2.0 + G_LPF_acc * dt) * ( (2.0 - G_LPF_acc * dt) * Acc_y_LPF_pre + G_LPF_acc * dt * ( Acc_y_correct + Acc_y_correct_pre ) );// LPF

        // Acc_x_LPF = 1.0 / (2.0 + G_HPF_acc * dt) * ( (2.0 - G_HPF_acc * dt) * Acc_x_LPF_pre + 2.0 * ( Acc_x_correct - Acc_x_correct_pre ) );// HPF
        // Acc_y_LPF = 1.0 / (2.0 + G_HPF_acc * dt) * ( (2.0 - G_HPF_acc * dt) * Acc_y_LPF_pre + 2.0 * ( Acc_y_correct - Acc_y_correct_pre ) );// HPF

        d_yawrate = 1.0 / (2.0 + G_LPF_gyro * dt) * ( (2.0 - G_LPF_gyro * dt) * d_yawrate_pre + 2.0 * G_LPF_gyro * (yaw_rate - yaw_rate_pre) );// Pseudo Derivative : ddphi_res

        yaw   = yaw   * 2.0 * pi / 360.0 + yaw_initial;// [rad] Convert this value to definition in modeling 


        // if( vy_cmd > 0.0 ){
        //   G_notch1 = N_roller * ( dtheta1_cmd + dtheta2_cmd + dtheta3_cmd + dtheta4_cmd ) / 4.0;
        // }else if( vx_cmd > 0.0 ){
        //   G_notch1 = N_roller * ( dtheta1_cmd - dtheta2_cmd + dtheta3_cmd - dtheta4_cmd ) / 4.0;
        // }

        // G_notch1 = 90.0;

        // printf("%f", G_notch1);
        // printf("\r\n");

        // * Notch Filter ( Band-stop Filter ) : Back Difference
        // yaw_rate_notch = 1.0 / ( 1.0 + 2.0*zeta1*G_notch1*dt + G_notch1*G_notch1*dt*dt ) * ( 2.0*(1.0+zeta1*G_notch1*dt)*yaw_rate_notch_pre - yaw_rate_notch_pre2 + (1.0 + G_notch1*G_notch1*dt*dt)*yaw_rate - 2.0*yaw_rate_pre + yaw_rate_pre2 );
        // * Notch Filter ( Band-stop Filter ) : Back Difference

        if     ( yaw - yaw_pre > 2.0*pi/2.0 ) yaw_digit--;
        else if( yaw_pre - yaw > 2.0*pi/2.0 ) yaw_digit++;
        yaw_pre = yaw;

        yaw = (yaw + yaw_digit * 2.0 * pi);// :* 2.0 * pi / 360.0;

        // dtheta1_res_raw = ( theta1_res - theta1_res_pre )/dt;
        // dtheta2_res_raw = ( theta2_res - theta2_res_pre )/dt;
        // dtheta3_res_raw = ( theta3_res - theta3_res_pre )/dt;
        // dtheta4_res_raw = ( theta4_res - theta4_res_pre )/dt;

        dtheta1_res  = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta1_res_pre + 2.0 * G_LPF * (theta1_res - theta1_res_pre) );
        dtheta2_res  = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta2_res_pre + 2.0 * G_LPF * (theta2_res - theta2_res_pre) );
        dtheta3_res  = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta3_res_pre + 2.0 * G_LPF * (theta3_res - theta3_res_pre) );
        dtheta4_res  = 1.0 / (2.0 + G_LPF * dt) * ( (2.0 - G_LPF * dt)*dtheta4_res_pre + 2.0 * G_LPF * (theta4_res - theta4_res_pre) );

        ddtheta1_res = 1.0 / (2.0 + G_LPF_ddth * dt) * ( (2.0 - G_LPF_ddth * dt)*ddtheta1_res_pre + 2.0 * G_LPF_ddth * (dtheta1_res - dtheta1_res_pre) );
        ddtheta2_res = 1.0 / (2.0 + G_LPF_ddth * dt) * ( (2.0 - G_LPF_ddth * dt)*ddtheta2_res_pre + 2.0 * G_LPF_ddth * (dtheta2_res - dtheta2_res_pre) );
        ddtheta3_res = 1.0 / (2.0 + G_LPF_ddth * dt) * ( (2.0 - G_LPF_ddth * dt)*ddtheta3_res_pre + 2.0 * G_LPF_ddth * (dtheta3_res - dtheta3_res_pre) );
        ddtheta4_res = 1.0 / (2.0 + G_LPF_ddth * dt) * ( (2.0 - G_LPF_ddth * dt)*ddtheta4_res_pre + 2.0 * G_LPF_ddth * (dtheta4_res - dtheta4_res_pre) );

          // * Save previous values
          ddtheta1_res_pre = ddtheta1_res;
          ddtheta2_res_pre = ddtheta2_res;
          ddtheta3_res_pre = ddtheta3_res;
          ddtheta4_res_pre = ddtheta4_res;
          // * Save previous values

        // * Slip Ratio Observer : SRO, SRE
        #ifdef Enable_Slip_Ratio_Observer
        delta_dv = ( L + W )*( L + W ) / Jz * ( - fd_hat1 - fd_hat2 + fd_hat3 + fd_hat4 );// 1.0112 * fd1,2,3,4

        // dv_1 = sqrt(2.0) / Mass * ( fd_hat1 + fd_hat3 ) - delta_dv - ( Fx_dis + Fy_dis ) / Mass + ( L + W ) / Jz * Mz_dis;// Just Encoder
        // dv_2 = sqrt(2.0) / Mass * ( fd_hat2 + fd_hat4 ) - delta_dv + ( Fx_dis - Fy_dis ) / Mass + ( L + W ) / Jz * Mz_dis;
        // dv_3 = sqrt(2.0) / Mass * ( fd_hat1 + fd_hat3 ) + delta_dv - ( Fx_dis + Fy_dis ) / Mass - ( L + W ) / Jz * Mz_dis;
        // dv_4 = sqrt(2.0) / Mass * ( fd_hat2 + fd_hat4 ) + delta_dv + ( Fx_dis - Fy_dis ) / Mass - ( L + W ) / Jz * Mz_dis;

        dv_1 = sqrt(2.0) / Mass * ( fd_hat1 + fd_hat3 ) - ( Fx_dis + Fy_dis ) / Mass - ( L + W ) * d_yawrate;// Encoder + Gyro
        dv_2 = sqrt(2.0) / Mass * ( fd_hat2 + fd_hat4 ) + ( Fx_dis - Fy_dis ) / Mass - ( L + W ) * d_yawrate;
        dv_3 = sqrt(2.0) / Mass * ( fd_hat1 + fd_hat3 ) - ( Fx_dis + Fy_dis ) / Mass + ( L + W ) * d_yawrate;
        dv_4 = sqrt(2.0) / Mass * ( fd_hat2 + fd_hat4 ) + ( Fx_dis - Fy_dis ) / Mass + ( L + W ) * d_yawrate;

        // dv_1_LPF = 1.0 / (2.0 + G_LPF_dv * dt) * ( (2.0 - G_LPF_dv * dt) * dv_1_LPF_pre + G_LPF_dv * dt * ( dv_1 + dv_1_pre ) );// LPF
        // dv_2_LPF = 1.0 / (2.0 + G_LPF_dv * dt) * ( (2.0 - G_LPF_dv * dt) * dv_2_LPF_pre + G_LPF_dv * dt * ( dv_2 + dv_2_pre ) );// LPF
        // dv_3_LPF = 1.0 / (2.0 + G_LPF_dv * dt) * ( (2.0 - G_LPF_dv * dt) * dv_3_LPF_pre + G_LPF_dv * dt * ( dv_3 + dv_3_pre ) );// LPF
        // dv_4_LPF = 1.0 / (2.0 + G_LPF_dv * dt) * ( (2.0 - G_LPF_dv * dt) * dv_4_LPF_pre + G_LPF_dv * dt * ( dv_4 + dv_4_pre ) );// LPF

        dv_1_LPF = dv_1;
        dv_2_LPF = dv_2;
        dv_3_LPF = dv_3;
        dv_4_LPF = dv_4;

          // * Save previous values
          dv_1_pre = dv_1;
          dv_2_pre = dv_2;
          dv_3_pre = dv_3;
          dv_4_pre = dv_4;

          dv_1_LPF_pre = dv_1_LPF;
          dv_2_LPF_pre = dv_2_LPF;
          dv_3_LPF_pre = dv_3_LPF;
          dv_4_LPF_pre = dv_4_LPF;
          // * Save previous values

        // d_yawrate = 0.0;

        dv_1_acc =   Acc_x_LPF + Acc_y_LPF - ( L + W ) * d_yawrate;
        dv_2_acc = - Acc_x_LPF + Acc_y_LPF - ( L + W ) * d_yawrate;
        dv_3_acc =   Acc_x_LPF + Acc_y_LPF + ( L + W ) * d_yawrate;
        dv_4_acc = - Acc_x_LPF + Acc_y_LPF + ( L + W ) * d_yawrate;
        
        // if( dtheta1_res > epsilon || dtheta1_res < - epsilon ){
        //   if( dv_1 >= 0.0 ){// Acceleration
        //     d_lambda_1_hat = ddtheta1_res / dtheta1_res * ( 1.0 - lambda_1_hat ) - dv_1 / ( Rw * dtheta1_res );
        //     // printf("1 \r\n");
        //   }else{// Deceleration
        //     d_lambda_1_hat = ddtheta1_res / dtheta1_res * ( 1.0 + lambda_1_hat ) - ( 1.0 + lambda_1_hat )*( 1.0 + lambda_1_hat ) * dv_1 / ( Rw * dtheta1_res );
        //     // printf("2 \r\n");
        //   }
        // }else{
        //     // d_lambda_1_hat = ( Rw * ddtheta1_res - dv_1 ) / epsilon;
        //     d_lambda_1_hat = 0.0;
        //     lambda_1_hat = 0.0;
        //     // printf("3 \r\n");
        // }

        // if( dtheta1_res > epsilon || dtheta1_res < - epsilon ){
        //   if( Rw * fabsf(dtheta1_res) > fabsf(v1_hat) ){// Acceleration
        //     d_lambda_1_hat = ddtheta1_ref / dtheta1_res * ( 1.0 - lambda_1_hat ) - dv_1 / ( Rw * dtheta1_res );
        //     lambda_1_hat  += d_lambda_1_hat * dt;
        //     v1_hat = ( 1.0 - lambda_1_hat ) * Rw * dtheta1_res;
        //   }else{// Deceleration
        //     d_lambda_1_hat = ddtheta1_ref / dtheta1_res * ( 1.0 + lambda_1_hat ) - ( 1.0 + lambda_1_hat )*( 1.0 + lambda_1_hat ) * dv_1 / ( Rw * dtheta1_res );
        //     lambda_1_hat  += d_lambda_1_hat * dt;
        //     v1_hat = Rw * dtheta1_res / ( 1.0 + lambda_1_hat );
        //   }
        // }else{
        //   if( dtheta1_res != 0.0 ){
        //     lambda_1_hat = ( Rw * dtheta1_res - v1_hat ) / epsilon;
        //   }else{
        //     d_lambda_1_hat = 0.0;
        //     lambda_1_hat = 0.0;
        //   }        
        // }

        if( dtheta1_res > epsilon / Rw || dtheta1_res < - epsilon / Rw ){
          // ! Acceleration Definition
          d_lambda_1_hat = ddtheta1_ref / dtheta1_res * ( 1.0 - lambda_1_hat ) - dv_1 / ( Rw * dtheta1_res );
          // ? Deceleration Definition
          // d_lambda_1_hat = ddtheta1_res / dtheta1_res * ( 1.0 + lambda_1_hat ) - ( 1.0 + lambda_1_hat )*( 1.0 + lambda_1_hat ) * dv_1 / ( Rw * dtheta1_res );
        }else{
          if( dtheta1_res != 0.0 ){
            lambda_1_hat = ( Rw * dtheta1_res - v1_hat ) / epsilon;
          }else{
            d_lambda_1_hat = 0.0;
            lambda_1_hat = 0.0;
          }    
        }

        if( dtheta2_res > epsilon / Rw || dtheta2_res < - epsilon / Rw ){
          // ! Acceleration Definition
          d_lambda_2_hat = ddtheta2_ref / dtheta2_res * ( 1.0 - lambda_2_hat ) - dv_2 / ( Rw * dtheta2_res );
          // ? Deceleration Definition
          // d_lambda_2_hat = ddtheta2_res / dtheta2_res * ( 1.0 + lambda_2_hat ) - ( 1.0 + lambda_2_hat )*( 1.0 + lambda_2_hat ) * dv_2 / ( Rw * dtheta2_res );
        }else{
          if( dtheta2_res != 0.0 ){
            lambda_2_hat = ( Rw * dtheta2_res - v2_hat ) / epsilon;
          }else{
            d_lambda_2_hat = 0.0;
            lambda_2_hat = 0.0;
          }    
        }

        if( dtheta3_res > epsilon / Rw || dtheta3_res < - epsilon / Rw ){
          // ! Acceleration Definition
          d_lambda_3_hat = ddtheta3_ref / dtheta3_res * ( 1.0 - lambda_3_hat ) - dv_3 / ( Rw * dtheta3_res );
          // ? Deceleration Definition
          // d_lambda_3_hat = ddtheta3_res / dtheta3_res * ( 1.0 + lambda_3_hat ) - ( 1.0 + lambda_3_hat )*( 1.0 + lambda_3_hat ) * dv_3 / ( Rw * dtheta3_res );
        }else{
          if( dtheta3_res != 0.0 ){
            lambda_3_hat = ( Rw * dtheta3_res - v3_hat ) / epsilon;
          }else{
            d_lambda_3_hat = 0.0;
            lambda_3_hat = 0.0;
          }    
        }

        if( dtheta4_res > epsilon / Rw || dtheta4_res < - epsilon / Rw ){
          // ! Acceleration Definition
          d_lambda_4_hat = ddtheta4_ref / dtheta4_res * ( 1.0 - lambda_4_hat ) - dv_4 / ( Rw * dtheta4_res );
          // ? Deceleration Definition
          // d_lambda_4_hat = ddtheta4_res / dtheta4_res * ( 1.0 + lambda_4_hat ) - ( 1.0 + lambda_4_hat )*( 1.0 + lambda_4_hat ) * dv_4 / ( Rw * dtheta4_res );
        }else{
          if( dtheta4_res != 0.0 ){
            lambda_4_hat = ( Rw * dtheta4_res - v4_hat ) / epsilon;
          }else{
            d_lambda_4_hat = 0.0;
            lambda_4_hat = 0.0;
          }    
        }

        if( dtheta1_res > epsilon_acc / Rw || dtheta1_res < - epsilon_acc / Rw ){
          // ! Acceleration Definition
          d_lambda_1_hat_acc = ddtheta1_ref / dtheta1_res * ( 1.0 - lambda_1_hat_acc ) - dv_1_acc / ( Rw * dtheta1_res );
          // ? Deceleration Definition
          // d_lambda_1_hat_acc = ddtheta1_res / dtheta1_res * ( 1.0 + lambda_1_hat_acc ) - ( 1.0 + lambda_1_hat_acc )*( 1.0 + lambda_1_hat_acc ) * dv_1_acc / ( Rw * dtheta1_res );
        }else{
          if( dtheta1_res != 0.0 ){
            lambda_1_hat_acc = ( Rw * dtheta1_res - v1_hat_acc ) / epsilon_acc;
          }else{
            d_lambda_1_hat_acc = 0.0;
            lambda_1_hat_acc = 0.0;
          }    
        }

        if( dtheta2_res > epsilon_acc / Rw || dtheta2_res < - epsilon_acc / Rw ){
          // ! Acceleration Definition
          d_lambda_2_hat_acc = ddtheta2_ref / dtheta2_res * ( 1.0 - lambda_2_hat_acc ) - dv_2_acc / ( Rw * dtheta2_res );
          // ? Deceleration Definition
          // d_lambda_2_hat_acc = ddtheta2_res / dtheta2_res * ( 1.0 + lambda_2_hat_acc ) - ( 1.0 + lambda_2_hat_acc )*( 1.0 + lambda_2_hat_acc ) * dv_2_acc / ( Rw * dtheta2_res );
        }else{
          if( dtheta2_res != 0.0 ){
            lambda_2_hat_acc = ( Rw * dtheta2_res - v2_hat_acc ) / epsilon_acc;
          }else{
            d_lambda_2_hat_acc = 0.0;
            lambda_2_hat_acc = 0.0;
          }    
        }

        if( dtheta3_res > epsilon_acc / Rw || dtheta3_res < - epsilon_acc / Rw ){
          // ! Acceleration Definition
          d_lambda_3_hat_acc = ddtheta3_ref / dtheta3_res * ( 1.0 - lambda_3_hat_acc ) - dv_3_acc / ( Rw * dtheta3_res );
          // ? Deceleration Definition
          // d_lambda_3_hat_acc = ddtheta3_res / dtheta3_res * ( 1.0 + lambda_3_hat_acc ) - ( 1.0 + lambda_3_hat_acc )*( 1.0 + lambda_3_hat_acc ) * dv_3_acc / ( Rw * dtheta3_res );
        }else{
          if( dtheta3_res != 0.0 ){
            lambda_3_hat_acc = ( Rw * dtheta3_res - v3_hat_acc ) / epsilon_acc;
          }else{
            d_lambda_3_hat_acc = 0.0;
            lambda_3_hat_acc = 0.0;
          }    
        }

        if( dtheta4_res > epsilon_acc / Rw || dtheta4_res < - epsilon_acc / Rw ){
          // ! Acceleration Definition
          d_lambda_4_hat_acc = ddtheta4_ref / dtheta4_res * ( 1.0 - lambda_4_hat_acc ) - dv_4_acc / ( Rw * dtheta4_res );
          // ? Deceleration Definition
          // d_lambda_4_hat_acc = ddtheta4_res / dtheta4_res * ( 1.0 + lambda_4_hat_acc ) - ( 1.0 + lambda_4_hat_acc )*( 1.0 + lambda_4_hat_acc ) * dv_4_acc / ( Rw * dtheta4_res );
        }else{
          if( dtheta4_res != 0.0 ){
            lambda_4_hat_acc = ( Rw * dtheta4_res - v4_hat_acc ) / epsilon_acc;
          }else{
            d_lambda_4_hat_acc = 0.0;
            lambda_4_hat_acc = 0.0;
          }    
        }

        lambda_1_hat     += d_lambda_1_hat * dt;
        lambda_2_hat     += d_lambda_2_hat * dt;
        lambda_3_hat     += d_lambda_3_hat * dt;
        lambda_4_hat     += d_lambda_4_hat * dt;

        lambda_1_hat_acc += d_lambda_1_hat_acc * dt;
        lambda_2_hat_acc += d_lambda_2_hat_acc * dt;
        lambda_3_hat_acc += d_lambda_3_hat_acc * dt;
        lambda_4_hat_acc += d_lambda_4_hat_acc * dt;

        v1_hat = Rw * dtheta1_res / ( 1.0 + lambda_1_hat );
        v2_hat = Rw * dtheta2_res / ( 1.0 + lambda_2_hat );
        v3_hat = Rw * dtheta3_res / ( 1.0 + lambda_3_hat );
        v4_hat = Rw * dtheta4_res / ( 1.0 + lambda_4_hat );

        v1_hat_acc = Rw * dtheta1_res / ( 1.0 + lambda_1_hat_acc );
        v2_hat_acc = Rw * dtheta2_res / ( 1.0 + lambda_2_hat_acc );
        v3_hat_acc = Rw * dtheta3_res / ( 1.0 + lambda_3_hat_acc );
        v4_hat_acc = Rw * dtheta4_res / ( 1.0 + lambda_4_hat_acc );
        #endif
        // * Slip Ratio Observer : SRO, SRE

        // * RLS with forgetting factor (Estimate Jacobi matrix T_hat)



        
        // * RLS with forgetting factor (Estimate Jacobi matrix T_hat)

        vx_res = (Rw / 4.0) * (dtheta1_res - dtheta2_res + dtheta3_res - dtheta4_res);// [m/sec]
        vy_res = (Rw / 4.0) * (dtheta1_res + dtheta2_res + dtheta3_res + dtheta4_res);
        // dphi_res = (Rw / 4.0) / (W + L) * ( - dtheta1_res - dtheta2_res + dtheta3_res + dtheta4_res);// [rad/sec]

        dphi_res = yaw_rate;
        // dphi_res = yaw_rate_notch;

        x_res   += vx_res   * dt;// [m]
        y_res   += vy_res   * dt;
        phi_res += dphi_res * dt;// [rad]

        direc1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
        direc2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
        direc3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim8);
        direc4 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);

        // * Command

        // if( t < t_experiment - 4.0 ){
        //   // vx_cmd = 0.5;
        //   // vy_cmd = 0.5;
        //   dphi_cmd = 1.0;
        // }else{
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        //   dphi_cmd = 0.0;
        // }

        // if( t < 3.0 ){
        //   vy_cmd = 0.3;
        // }else if( t < 5.0 ){
        //   vx_cmd = -0.3;
        //   vy_cmd = 0.0;
        // }else if( t < 10.0 ){
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        //   dphi_cmd = 0.5;
        // // }
        // // else if( t < 12.0 ){
        // //   vx_cmd = 0.0;
        // //   vy_cmd = 0.0;
        // //   dphi_cmd = 0.0;
        // }else{
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        //   dphi_cmd = 0.0;
        // }

        // if( t < 3.0 ){
        //   vy_cmd = 0.3;
        // }else if( t < 6.0 ){
        //   vx_cmd = 0.3;
        //   vy_cmd = 0.0;
        // }else if( t < 9.0 ){
        //   vx_cmd = 0.0;
        //   vy_cmd = -0.3;
        // }else if( t < 12.0 ){
        //   vx_cmd = -0.3;
        //   vy_cmd = 0.0;
        // }else{
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        // }
        
        // ! --1-- : Steady circle turning without changing posture of vehicle
        // omega = 0.5;// Period T is 2pi / omega
        // r     = 0.8;

        // if(t < t_experiment -3.0){
        //   vx_cmd   = - r * omega * sin(omega * t);
        //   vy_cmd   =   r * omega * cos(omega * t);
        //   dphi_cmd = 0.0;
        // }

        // ! --2-- : Steady circle turning while pointing the head toward center of trajectory
        // omega = 0.5;// Period T is 2pi / omega
        // r     = 0.8;
        
        // if(t < t_experiment -3.0){
        //   vx_cmd   = - r * omega;
        //   vy_cmd   = 0.0;
        //   dphi_cmd = omega;
        // }

        // ! --3-- : Steady circle turning while pointing the side toward center of trajectory
        // omega = 0.5;// Period T is 2pi / omega
        // r     = 0.75;
        // omega = 0.3;
        // r     = 0.45;
        // omega = 0.5;
        // r     = 0.3;// * Internal Singular Point
        // omega = 0.5;
        // r     = 0.6;//0.5;
        omega = 0.8;
        r = 0.6;//0.4;

        // if( t < 3.0 ){
        //   omega = 0.1;
        // }else if( t < 6.0 ){
        //   omega = 0.5;
        // }else if( t < 9.0 ){
        //   omega = 0.1;
        // }else if( t < 12.0 ){
        //   omega = 0.5;
        // }else{
        //   omega = 0.0;
        // }

        if( t < 6.0 ){
          omega = 0.1;
        }else if( t < 12.0 ){
          omega = 0.5;
        }else{
          omega = 0.0;
        }
        
        // if( t < 3.0 ){
        //   r = 0.2;
        // }else if( t < 6.0 ){
        //   r = 0.75;
        // }else if( t < 9.0 ){
        //   r = 0.1;
        // }else if( t < 12.0 ){
        //   r = 0.75;
        // }else{
        //   r = 0.0;
        // }
        if(t < t_experiment - 3.0){
          vx_cmd   = 0.0;
          vy_cmd   = r * omega;
          dphi_cmd = omega;
        }else{
          vx_cmd   = 0.0;
          vy_cmd   = 0.0;
          dphi_cmd = 0.0;
        }

        // ! --4-- : Sin wave movement without changing posture of vehicle
        // omega = 0.3;// Period T is 2pi / omega
        // A = 0.8;
        
        // if(t < t_experiment -3.0){
        //   vx_cmd   = omega;
        //   vy_cmd   = A * omega * cos(omega * t);
        //   dphi_cmd = 0.0;
        // }
        
        // ! --5-- : Sin wave movement while changing posture of vehicle : Not work well
        // ! Not work well
        // omega = 0.2;// Period T is 2pi / omega
        // A = 0.6;

        // if(t < t_experiment -3.0){
        //   vx_cmd   = omega * sin(omega*t + pi / 4.0) - A*omega*cos(omega*t)*cos(omega*t + pi / 4.0);
        //   vy_cmd   = omega * cos(omega*t + pi / 4.0) + A*omega*cos(omega*t)*sin(omega*t + pi / 4.0);
        //   dphi_cmd = - pi / 4.0 * sin(omega*t);
        // }

        // ! --6-- : Diagonal movement

        // if( t < t_experiment - 3.0 ){
        //   vx_cmd = 0.2;
        //   vy_cmd = vx_cmd;
        // }else{
        //   vx_cmd = 0.0;
        //   vy_cmd = 0.0;
        // }

        // ! --7-- : Acceleration & rotation movement
        
        // if( t < t_experiment - 3.0 ){
        //   if( t < 3.0 ){
        //     omega = 0.0;
        //     r     = 0.0;
        //   }else if( t < t_experiment ){
        //     omega = 0.35;
        //     r     = 0.1 * (t - 3.0);
        //   }
        // }

        // if(t < 3.0){
        //   vx_cmd   = 0.0;
        //   vy_cmd   = 0.2;
        //   dphi_cmd = 0.0;
        // }else{
        //   vx_cmd   = 0.0;
        //   vy_cmd   = 0.2 + r * omega;
        //   dphi_cmd = omega;
        // }

        // if( t > t_experiment - 3.0 ){
        //   vx_cmd   = 0.0;
        //   vy_cmd   = 0.0;
        //   dphi_cmd = 0.0;
        // }

        // * Command

        v1_x = vx_res - L * yaw_rate;
        v2_x = vx_res + L * yaw_rate;
        v3_x = vx_res + L * yaw_rate;
        v4_x = vx_res - L * yaw_rate;

        v1_y = vy_res - W * yaw_rate;
        v2_y = vy_res - W * yaw_rate;
        v3_y = vy_res + W * yaw_rate;
        v4_y = vy_res + W * yaw_rate;

        // if( v1_x < epsilon && v1_x > - epsilon ) v1_x = epsilon;// fabsf( v1_x ) * epsilon;
        // if( v2_x < epsilon && v2_x > - epsilon ) v2_x = epsilon;// fabsf( v2_x ) * epsilon;
        // if( v3_x < epsilon && v3_x > - epsilon ) v3_x = epsilon;// fabsf( v3_x ) * epsilon;
        // if( v4_x < epsilon && v4_x > - epsilon ) v4_x = epsilon;// fabsf( v4_x ) * epsilon;

        // alpha_1 = atanf( v1_y / v1_x );
        // alpha_2 = atanf( v2_y / v2_x );
        // alpha_3 = atanf( v3_y / v3_x );
        // alpha_4 = atanf( v4_y / v4_x );

        alpha_1 = atan2f( v1_y , v1_x );
        alpha_2 = atan2f( v2_y , v2_x );
        alpha_3 = atan2f( v3_y , v3_x );
        alpha_4 = atan2f( v4_y , v4_x );

        // w1 = cos( pi / 4.0 + alpha_1 ) * cos( pi / 4.0 + alpha_1 );
        // w2 = cos( pi / 4.0 - alpha_2 ) * cos( pi / 4.0 - alpha_2 );
        // w3 = cos( pi / 4.0 + alpha_3 ) * cos( pi / 4.0 + alpha_3 );
        // w4 = cos( pi / 4.0 - alpha_4 ) * cos( pi / 4.0 - alpha_4 );

        // * With FTA
        w1 = cos( pi / 4.0 + alpha_1_hat ) * cos( pi / 4.0 + alpha_1_hat );
        w2 = cos( pi / 4.0 - alpha_2_hat ) * cos( pi / 4.0 - alpha_2_hat );
        w3 = cos( pi / 4.0 + alpha_3_hat ) * cos( pi / 4.0 + alpha_3_hat );
        w4 = cos( pi / 4.0 - alpha_4_hat ) * cos( pi / 4.0 - alpha_4_hat );
        // * With FTA

        // w1 = 0.5;
        // w2 = 0.5;
        // w3 = 0.5;
        // w4 = 0.5;

        // * Fixed Trace Algorithm, FTA
        // alpha_1_hat = alpha_1_hat_pre - P1_k_1 / ( 1.0 + P1_k_1 ) * ( alpha_1_hat_pre - alpha_1 );
        // alpha_2_hat = alpha_2_hat_pre - P2_k_1 / ( 1.0 + P2_k_1 ) * ( alpha_2_hat_pre - alpha_2 );
        // alpha_3_hat = alpha_3_hat_pre - P3_k_1 / ( 1.0 + P3_k_1 ) * ( alpha_3_hat_pre - alpha_3 );
        // alpha_4_hat = alpha_4_hat_pre - P4_k_1 / ( 1.0 + P4_k_1 ) * ( alpha_4_hat_pre - alpha_4 );

        tan_beta_1_hat = tan_beta_1_hat_pre - P1_k_1 * v1_y / ( 1.0 + v1_y * P1_k_1 * v1_y ) * ( tan_beta_1_hat_pre * v1_y - v1_x );
        tan_beta_2_hat = tan_beta_2_hat_pre - P2_k_1 * v2_y / ( 1.0 + v2_y * P2_k_1 * v2_y ) * ( tan_beta_2_hat_pre * v2_y - v2_x );
        tan_beta_3_hat = tan_beta_3_hat_pre - P3_k_1 * v3_y / ( 1.0 + v3_y * P3_k_1 * v3_y ) * ( tan_beta_3_hat_pre * v3_y - v3_x );
        tan_beta_4_hat = tan_beta_4_hat_pre - P4_k_1 * v4_y / ( 1.0 + v4_y * P4_k_1 * v4_y ) * ( tan_beta_4_hat_pre * v4_y - v4_x );

        // if( vx_cmd == 0.0 ){
        //   tan_beta_1_hat = fabsf( tan_beta_1_hat );
        //   tan_beta_2_hat = fabsf( tan_beta_2_hat );
        //   tan_beta_3_hat = fabsf( tan_beta_3_hat );
        //   tan_beta_4_hat = fabsf( tan_beta_4_hat );
        // }

        Kappa_1 = 1.0 / ( 1.0 + Gamma * ( v1_y * v1_y ) );
        Kappa_2 = 1.0 / ( 1.0 + Gamma * ( v2_y * v2_y ) );
        Kappa_3 = 1.0 / ( 1.0 + Gamma * ( v3_y * v3_y ) );
        Kappa_4 = 1.0 / ( 1.0 + Gamma * ( v4_y * v4_y ) );

        P1_k = 1.0 / Kappa_1 * ( P1_k_1 - P1_k_1 * v1_y * v1_y * P1_k_1 / ( 1.0 + v1_y * P1_k_1 * v1_y ) );// ! Be careful of zero-division by Kappa's initial value!
        P2_k = 1.0 / Kappa_2 * ( P2_k_1 - P2_k_1 * v2_y * v2_y * P2_k_1 / ( 1.0 + v2_y * P2_k_1 * v2_y ) );
        P3_k = 1.0 / Kappa_3 * ( P3_k_1 - P3_k_1 * v3_y * v3_y * P3_k_1 / ( 1.0 + v3_y * P3_k_1 * v3_y ) );
        P4_k = 1.0 / Kappa_4 * ( P4_k_1 - P4_k_1 * v4_y * v4_y * P4_k_1 / ( 1.0 + v4_y * P4_k_1 * v4_y ) );

        // if ( tan_beta_1_hat == 0.0 ) tan_beta_1_hat = epsilon;
        // if ( tan_beta_2_hat == 0.0 ) tan_beta_2_hat = epsilon;
        // if ( tan_beta_3_hat == 0.0 ) tan_beta_3_hat = epsilon;
        // if ( tan_beta_4_hat == 0.0 ) tan_beta_4_hat = epsilon;

        tan_alpha_1_hat = 1.0 / tan_beta_1_hat;
        tan_alpha_2_hat = 1.0 / tan_beta_2_hat;
        tan_alpha_3_hat = 1.0 / tan_beta_3_hat;
        tan_alpha_4_hat = 1.0 / tan_beta_4_hat;

        alpha_1_hat = atan2f( tan_alpha_1_hat, 1.0 );// Argument Order : ( y, x )
        alpha_2_hat = atan2f( tan_alpha_2_hat, 1.0 );
        alpha_3_hat = atan2f( tan_alpha_3_hat, 1.0 );
        alpha_4_hat = atan2f( tan_alpha_4_hat, 1.0 );

        // if(isnan(tan_beta_1_hat)) printf("tan_beta_1_hat");
        // if(isnan(P1_k)) printf("P1_k");
        // if(isnan(Kappa_1)) printf("Kappa_1");
        // if(isnan(tan_alpha_1_hat)) printf("tan_alpha_1_hat");
        // if(isnan(alpha_1_hat)) printf("alpha_1_hat");
        // printf("\r\n");

        // printf("P1_k:%f", P1_k);
        // printf("\r\n");

          // * Save previous values
          tan_beta_1_hat_pre = tan_beta_1_hat;
          tan_beta_2_hat_pre = tan_beta_2_hat;
          tan_beta_3_hat_pre = tan_beta_3_hat;
          tan_beta_4_hat_pre = tan_beta_4_hat;

          P1_k_1 = P1_k;
          P2_k_1 = P2_k;
          P3_k_1 = P3_k;
          P4_k_1 = P4_k;
          // * Save previous values
        // * Fixed Trace Algorithm, FTA


        #ifdef Enable_Driving_Force_Distribution_Control
        ddx_ref   = Kp_df_x   * (vx_cmd   -   vx_res);
        ddy_ref   = Kp_df_y   * (vy_cmd   -   vy_res);
        ddphi_ref = Kp_df_phi * (dphi_cmd - dphi_res);

        fx_ref = Mass * ddx_ref + WOB_FB * Fx_dis;
        fy_ref = Mass * ddy_ref + WOB_FB * Fy_dis;
        Mz_ref = Jz * ddphi_ref + WOB_FB * Mz_dis;

        // v1_x = vx_res - L * yaw_rate;
        // v2_x = vx_res + L * yaw_rate;
        // v3_x = vx_res + L * yaw_rate;
        // v4_x = vx_res - L * yaw_rate;

        // v1_y = vy_res - W * yaw_rate;
        // v2_y = vy_res - W * yaw_rate;
        // v3_y = vy_res + W * yaw_rate;
        // v4_y = vy_res + W * yaw_rate;

        // if( v1_x < epsilon && v1_x > - epsilon ) v1_x = epsilon;// fabsf( v1_x ) * epsilon;
        // if( v2_x < epsilon && v2_x > - epsilon ) v2_x = epsilon;// fabsf( v2_x ) * epsilon;
        // if( v3_x < epsilon && v3_x > - epsilon ) v3_x = epsilon;// fabsf( v3_x ) * epsilon;
        // if( v4_x < epsilon && v4_x > - epsilon ) v4_x = epsilon;// fabsf( v4_x ) * epsilon;

        // // alpha_1 = atanf( v1_y / v1_x );
        // // alpha_2 = atanf( v2_y / v2_x );
        // // alpha_3 = atanf( v3_y / v3_x );
        // // alpha_4 = atanf( v4_y / v4_x );

        // alpha_1 = atan2f( v1_y , v1_x );
        // alpha_2 = atan2f( v2_y , v2_x );
        // alpha_3 = atan2f( v3_y , v3_x );
        // alpha_4 = atan2f( v4_y , v4_x );

        // w1 = cos( pi / 4.0 + alpha_1 ) * cos( pi / 4.0 + alpha_1 );
        // w2 = cos( pi / 4.0 - alpha_2 ) * cos( pi / 4.0 - alpha_2 );
        // w3 = cos( pi / 4.0 + alpha_3 ) * cos( pi / 4.0 + alpha_3 );
        // w4 = cos( pi / 4.0 - alpha_4 ) * cos( pi / 4.0 - alpha_4 );

        // w1 = 0.5;
        // w2 = 0.5;
        // w3 = 0.5;
        // w4 = 0.5;

        // * Jacobi Matrix (T^T)^+ --> Future Work : Weighted Jacobi Matrix
        // fd1_ref_normal = sqrt(2.0) * 1.0 / 4.0 * (   fx_ref + fy_ref - 1.0 / ( L + W ) * Mz_ref );// Cancel Rw term
        // fd2_ref_normal = sqrt(2.0) * 1.0 / 4.0 * ( - fx_ref + fy_ref - 1.0 / ( L + W ) * Mz_ref );// Add sqrt(2.0) : as of 2021/01/08
        // fd3_ref_normal = sqrt(2.0) * 1.0 / 4.0 * (   fx_ref + fy_ref + 1.0 / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)
        // fd4_ref_normal = sqrt(2.0) * 1.0 / 4.0 * ( - fx_ref + fy_ref + 1.0 / ( L + W ) * Mz_ref );// ! For comparing fd before distributing process : as of 2021/01/16

        fd1_ref_normal = 1.0 / ( sqrtf(2.0) * 4.0 ) * ( sqrtf(2.0) * (   fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)
        fd2_ref_normal = 1.0 / ( sqrtf(2.0) * 4.0 ) * ( sqrtf(2.0) * ( - fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// ! For comparing fd before distributing process : as of 2021/01/16
        fd3_ref_normal = 1.0 / ( sqrtf(2.0) * 4.0 ) * ( sqrtf(2.0) * (   fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );// Change place of sqrt(2.0) : as of 2021/01/17
        fd4_ref_normal = 1.0 / ( sqrtf(2.0) * 4.0 ) * ( sqrtf(2.0) * ( - fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );

        // fd1_ref = Rw / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w3 + w4 ) * fx_ref + ( w2 + w3 ) * fy_ref - ( w2 + w4 ) / ( L + W ) * Mz_ref );
        // fd2_ref = Rw / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w3 + w4 ) * fx_ref + ( w1 + w4 ) * fy_ref - ( w1 + w3 ) / ( L + W ) * Mz_ref );
        // fd3_ref = Rw / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w1 + w2 ) * fx_ref + ( w1 + w4 ) * fy_ref + ( w2 + w4 ) / ( L + W ) * Mz_ref );
        // fd4_ref = Rw / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w1 + w2 ) * fx_ref + ( w2 + w3 ) * fy_ref + ( w1 + w3 ) / ( L + W ) * Mz_ref );

        // fd1_ref = sqrt(2.0) / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w3 + w4 ) * fx_ref + ( w2 + w3 ) * fy_ref - ( w2 + w4 ) / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W !! Check!!
        // fd2_ref = sqrt(2.0) / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w3 + w4 ) * fx_ref + ( w1 + w4 ) * fy_ref - ( w1 + w3 ) / ( L + W ) * Mz_ref );// as of 2021/01/17
        // fd3_ref = sqrt(2.0) / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w1 + w2 ) * fx_ref + ( w1 + w4 ) * fy_ref + ( w2 + w4 ) / ( L + W ) * Mz_ref );
        // fd4_ref = sqrt(2.0) / ( 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w1 + w2 ) * fx_ref + ( w2 + w3 ) * fy_ref + ( w1 + w3 ) / ( L + W ) * Mz_ref );

        fd1_ref = 1.0 / ( sqrtf(2.0) * 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w3 + w4 ) * sqrtf(2.0) * fx_ref + ( w2 + w3 ) * sqrtf(2.0) * fy_ref - ( w2 + w4 ) / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W !! Check!!
        fd2_ref = 1.0 / ( sqrtf(2.0) * 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w3 + w4 ) * sqrtf(2.0) * fx_ref + ( w1 + w4 ) * sqrtf(2.0) * fy_ref - ( w1 + w3 ) / ( L + W ) * Mz_ref );// as of 2021/01/17
        fd3_ref = 1.0 / ( sqrtf(2.0) * 2.0 * ( w1 + w2 + w3 + w4 ) ) * (   ( w1 + w2 ) * sqrtf(2.0) * fx_ref + ( w1 + w4 ) * sqrtf(2.0) * fy_ref + ( w2 + w4 ) / ( L + W ) * Mz_ref );
        fd4_ref = 1.0 / ( sqrtf(2.0) * 2.0 * ( w1 + w2 + w3 + w4 ) ) * ( - ( w1 + w2 ) * sqrtf(2.0) * fx_ref + ( w2 + w3 ) * sqrtf(2.0) * fy_ref + ( w1 + w3 ) / ( L + W ) * Mz_ref );
        #endif

        #ifdef Enable_Driving_Force_Control
        ddx_ref   = Kp_df_x   * (vx_cmd   -   vx_res);
        ddy_ref   = Kp_df_y   * (vy_cmd   -   vy_res);
        ddphi_ref = Kp_df_phi * (dphi_cmd - dphi_res);

        fx_ref = Mass * ddx_ref + WOB_FB * Fx_dis;
        fy_ref = Mass * ddy_ref + WOB_FB * Fy_dis;
        Mz_ref = Jz * ddphi_ref + WOB_FB * Mz_dis;

        // * Jacobi Matrix (T^T)^+ --> Future Work : Weighted Jacobi Matrix
        // fd1_ref = sqrt(2.0) * 1.0 / 4.0 * (   fx_ref + fy_ref - 1.0 / ( L + W ) * Mz_ref );// Cancel Rw term
        // fd2_ref = sqrt(2.0) * 1.0 / 4.0 * ( - fx_ref + fy_ref - 1.0 / ( L + W ) * Mz_ref );// Add sqrt(2.0) : as of 2021/01/08
        // fd3_ref = sqrt(2.0) * 1.0 / 4.0 * (   fx_ref + fy_ref + 1.0 / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)
        // fd4_ref = sqrt(2.0) * 1.0 / 4.0 * ( - fx_ref + fy_ref + 1.0 / ( L + W ) * Mz_ref );

        // fd1_ref = 1.0 / 4.0 * ( sqrt(2.0) * (   fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// Cancel Rw term
        // fd2_ref = 1.0 / 4.0 * ( sqrt(2.0) * ( - fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// Add sqrt(2.0) : as of 2021/01/08
        // fd3_ref = 1.0 / 4.0 * ( sqrt(2.0) * (   fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)
        // fd4_ref = 1.0 / 4.0 * ( sqrt(2.0) * ( - fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );// Change place of sqrt(2.0) : as of 2021/01/17

        fd1_ref = 1.0 / ( sqrt(2.0) * 4.0 ) * ( sqrt(2.0) * (   fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// Cancel Rw term
        fd2_ref = 1.0 / ( sqrt(2.0) * 4.0 ) * ( sqrt(2.0) * ( - fx_ref + fy_ref ) - 1.0 / ( L + W ) * Mz_ref );// Add sqrt(2.0) : as of 2021/01/08
        fd3_ref = 1.0 / ( sqrt(2.0) * 4.0 ) * ( sqrt(2.0) * (   fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );// ! Coefficient *, / of L + W : as of 2021/01/16, related to Weighted Jacobi Matrix ( *, / L + W)
        fd4_ref = 1.0 / ( sqrt(2.0) * 4.0 ) * ( sqrt(2.0) * ( - fx_ref + fy_ref ) + 1.0 / ( L + W ) * Mz_ref );// Change place of sqrt(2.0) : as of 2021/01/17
        #endif

        #ifdef Enable_Driving_Force_Control_Jointspace_Part
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
        vel1_ref_new = Kp_df * ( fd1_ref - fd_hat1 ) + Ki_df_integral1;
        vel2_ref_new = Kp_df * ( fd2_ref - fd_hat2 ) + Ki_df_integral2;
        vel3_ref_new = Kp_df * ( fd3_ref - fd_hat3 ) + Ki_df_integral3;
        vel4_ref_new = Kp_df * ( fd4_ref - fd_hat4 ) + Ki_df_integral4;

        // * I
        // vel1_ref_new = Ki_df_integral1;
        // vel2_ref_new = Ki_df_integral2;
        // vel3_ref_new = Ki_df_integral3;
        // vel4_ref_new = Ki_df_integral4;

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

        ddx_ref   = Kp_vv_x   * (vx_cmd   -   vx_res) + WOB_FB * Fx_dis / Mass;
        ddy_ref   = Kp_vv_y   * (vy_cmd   -   vy_res) + WOB_FB * Fy_dis / Mass;
        ddphi_ref = Kp_vv_phi * (dphi_cmd - dphi_res) + WOB_FB * Mz_dis / Jz;

        ddtheta1_ref =  20.0 * ddx_ref + 20.0 * ddy_ref - 6.0 * ddphi_ref;// [rad/sec^2]
        ddtheta2_ref = -20.0 * ddx_ref + 20.0 * ddy_ref - 6.0 * ddphi_ref;
        ddtheta3_ref =  20.0 * ddx_ref + 20.0 * ddy_ref + 6.0 * ddphi_ref;
        ddtheta4_ref = -20.0 * ddx_ref + 20.0 * ddy_ref + 6.0 * ddphi_ref;
        #endif

        #ifdef angular_velocity_control
        // Convert Local to Joint space
        // Jacobi T matrix (including "Rw")

        // * For angular acceleration experiment
        // if(t < 5.0){
        //   vy_cmd = 0.3;// [m/sec]
        // }else if(t < 10.0){
        //   vy_cmd = 0.5;
        // }else{
        //   vy_cmd = 0.3;
        // }

        // if(t < 5.0){
        //   vy_cmd = 0.3;// [m/sec]
        // }else{
        //   vy_cmd = 0.5;
        // }
        // * For angular acceleration experiment

        // vx_cmd = 0.3;
        vy_cmd = 0.3;
        // dphi_cmd = 0.5;

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

        #ifdef Enable_PD_controller_av
        delta_dtheta1 = dtheta1_cmd - dtheta1_res;
        delta_dtheta2 = dtheta2_cmd - dtheta2_res;
        delta_dtheta3 = dtheta3_cmd - dtheta3_res;
        delta_dtheta4 = dtheta4_cmd - dtheta4_res;

        // PD_controller1_av = Kd_av * 1.0 / (2.0 + G_LPF_D_av * dt) * ((2.0 - G_LPF_D_av * dt) * PD_controller1_av_pre + 2.0 * G_LPF_D_av * (delta_dtheta1 - delta_dtheta1_pre));
        // PD_controller2_av = Kd_av * 1.0 / (2.0 + G_LPF_D_av * dt) * ((2.0 - G_LPF_D_av * dt) * PD_controller2_av_pre + 2.0 * G_LPF_D_av * (delta_dtheta2 - delta_dtheta2_pre));
        // PD_controller3_av = Kd_av * 1.0 / (2.0 + G_LPF_D_av * dt) * ((2.0 - G_LPF_D_av * dt) * PD_controller3_av_pre + 2.0 * G_LPF_D_av * (delta_dtheta3 - delta_dtheta3_pre));
        // PD_controller4_av = Kd_av * 1.0 / (2.0 + G_LPF_D_av * dt) * ((2.0 - G_LPF_D_av * dt) * PD_controller4_av_pre + 2.0 * G_LPF_D_av * (delta_dtheta4 - delta_dtheta4_pre));

        // * Backward Difference
        PD_controller1_av = 1.0 / (1.0 + G_LPF_D_av * dt) * ( PD_controller1_av_pre + ( Kp_av * ( 1.0 + G_LPF_D_av * dt ) + Kd_av * G_LPF_D_av )*delta_dtheta1 - (Kp_av + Kd_av * G_LPF_D_av) * delta_dtheta1_pre );
        PD_controller2_av = 1.0 / (1.0 + G_LPF_D_av * dt) * ( PD_controller2_av_pre + ( Kp_av * ( 1.0 + G_LPF_D_av * dt ) + Kd_av * G_LPF_D_av )*delta_dtheta2 - (Kp_av + Kd_av * G_LPF_D_av) * delta_dtheta2_pre );
        PD_controller3_av = 1.0 / (1.0 + G_LPF_D_av * dt) * ( PD_controller3_av_pre + ( Kp_av * ( 1.0 + G_LPF_D_av * dt ) + Kd_av * G_LPF_D_av )*delta_dtheta3 - (Kp_av + Kd_av * G_LPF_D_av) * delta_dtheta3_pre );
        PD_controller4_av = 1.0 / (1.0 + G_LPF_D_av * dt) * ( PD_controller4_av_pre + ( Kp_av * ( 1.0 + G_LPF_D_av * dt ) + Kd_av * G_LPF_D_av )*delta_dtheta4 - (Kp_av + Kd_av * G_LPF_D_av) * delta_dtheta4_pre );

        // * Mr. Yokokura's Method ( Bilinear Transform / Tustin Transform )
        // PD_controller1_av = 1.0 / (2.0 + G_LPF_D_av * dt) * ( PD_controller1_av_pre*(2.0-G_LPF_D_av*dt) + Kp_av*( 2.0*(delta_dtheta1-delta_dtheta1_pre) + G_LPF_D_av*dt*(delta_dtheta1+delta_dtheta1_pre) ) + 2.0*Kd_av*G_LPF_D_av*(delta_dtheta1-delta_dtheta1_pre) );
        // PD_controller2_av = 1.0 / (2.0 + G_LPF_D_av * dt) * ( PD_controller2_av_pre*(2.0-G_LPF_D_av*dt) + Kp_av*( 2.0*(delta_dtheta2-delta_dtheta2_pre) + G_LPF_D_av*dt*(delta_dtheta2+delta_dtheta2_pre) ) + 2.0*Kd_av*G_LPF_D_av*(delta_dtheta2-delta_dtheta2_pre) );
        // PD_controller3_av = 1.0 / (2.0 + G_LPF_D_av * dt) * ( PD_controller3_av_pre*(2.0-G_LPF_D_av*dt) + Kp_av*( 2.0*(delta_dtheta3-delta_dtheta3_pre) + G_LPF_D_av*dt*(delta_dtheta3+delta_dtheta3_pre) ) + 2.0*Kd_av*G_LPF_D_av*(delta_dtheta3-delta_dtheta3_pre) );
        // PD_controller4_av = 1.0 / (2.0 + G_LPF_D_av * dt) * ( PD_controller4_av_pre*(2.0-G_LPF_D_av*dt) + Kp_av*( 2.0*(delta_dtheta4-delta_dtheta4_pre) + G_LPF_D_av*dt*(delta_dtheta4+delta_dtheta4_pre) ) + 2.0*Kd_av*G_LPF_D_av*(delta_dtheta4-delta_dtheta4_pre) );

          // * Save previous values
          PD_controller1_av_pre = PD_controller1_av;
          PD_controller2_av_pre = PD_controller2_av;
          PD_controller3_av_pre = PD_controller3_av;
          PD_controller4_av_pre = PD_controller4_av;

          delta_dtheta1_pre = delta_dtheta1;
          delta_dtheta2_pre = delta_dtheta2;
          delta_dtheta3_pre = delta_dtheta3;
          delta_dtheta4_pre = delta_dtheta4;
          // * Save previous values

        ddtheta1_ref = PD_controller1_av;
        ddtheta2_ref = PD_controller2_av;
        ddtheta3_ref = PD_controller3_av;
        ddtheta4_ref = PD_controller4_av;
        #endif


        #ifdef Enable_Driving_Force_Control_Jointspace_Part
        // #ifdef Enable_Driving_Force_Control
        ddtheta1_ref = Kp_av_df * (dtheta1_cmd - dtheta1_res);
        ddtheta2_ref = Kp_av_df * (dtheta2_cmd - dtheta2_res);
        ddtheta3_ref = Kp_av_df * (dtheta3_cmd - dtheta3_res);
        ddtheta4_ref = Kp_av_df * (dtheta4_cmd - dtheta4_res);
        #endif

        #ifdef Enable_Inertia_Mass_Matrix_by_Lagrange
        // i1_ref = (M11*ddtheta1_ref + M12*ddtheta2_ref + M13*ddtheta3_ref + M14*ddtheta4_ref)/( Gear * Ktn );
        // i2_ref = (M21*ddtheta1_ref + M22*ddtheta2_ref + M23*ddtheta3_ref + M24*ddtheta4_ref)/( Gear * Ktn );
        // i3_ref = (M31*ddtheta1_ref + M32*ddtheta2_ref + M33*ddtheta3_ref + M34*ddtheta4_ref)/( Gear * Ktn );
        // i4_ref = (M41*ddtheta1_ref + M42*ddtheta2_ref + M43*ddtheta3_ref + M44*ddtheta4_ref)/( Gear * Ktn );

        i1_ref = M11*ddtheta1_ref / ( Gear * Ktn );
        i2_ref = M22*ddtheta2_ref / ( Gear * Ktn );
        i3_ref = M33*ddtheta3_ref / ( Gear * Ktn );
        i4_ref = M44*ddtheta4_ref / ( Gear * Ktn );
        #endif

        #ifdef Enable_Inertia_Identification
        // * When identifying F and D
        i1_ref = M11 * ddtheta1_ref / ( Gear * Ktn );
        i2_ref = M22 * ddtheta2_ref / ( Gear * Ktn );
        i3_ref = M33 * ddtheta3_ref / ( Gear * Ktn );
        i4_ref = M44 * ddtheta4_ref / ( Gear * Ktn );// Gear * Gear * J4 
        // * When identifying F and D
        #endif

        #ifdef Enable_DOB

        // * Raw data ( Without LPF )
        // tau_dis1_raw = Gear * Ktn * i1_ref - 
        // * Raw data ( Without LPF )

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
        // tau_dfob1 = integral_tau_dfob1 - M11 * G_DFOB * dtheta1_res;// * Continuous
        // tau_dfob2 = integral_tau_dfob2 - M22 * G_DFOB * dtheta2_res;// * Continuous
        // tau_dfob3 = integral_tau_dfob3 - M33 * G_DFOB * dtheta3_res;// * Continuous
        // tau_dfob4 = integral_tau_dfob4 - M44 * G_DFOB * dtheta4_res;// * Continuous

        // * Withdraw Non diagonal elements
        tau_dfob1 = integral_tau_dfob1 - M11 * G_DFOB * dtheta1_res - M12 * G_DFOB * dtheta2_res - M13 * G_DFOB * dtheta3_res - M14 * G_DFOB * dtheta4_res;// * Continuous
        tau_dfob2 = integral_tau_dfob2 - M21 * G_DFOB * dtheta1_res - M22 * G_DFOB * dtheta2_res - M23 * G_DFOB * dtheta3_res - M24 * G_DFOB * dtheta4_res;// * Continuous
        tau_dfob3 = integral_tau_dfob3 - M31 * G_DFOB * dtheta1_res - M32 * G_DFOB * dtheta2_res - M33 * G_DFOB * dtheta3_res - M34 * G_DFOB * dtheta4_res;// * Continuous
        tau_dfob4 = integral_tau_dfob4 - M41 * G_DFOB * dtheta1_res - M42 * G_DFOB * dtheta2_res - M43 * G_DFOB * dtheta3_res - M44 * G_DFOB * dtheta4_res;// * Continuous

        switch(direc1){
          case 0:
            tau_fric1 = F1_plus + D1_plus * dtheta1_res;
            break;
          case 1:
            tau_fric1 = F1_minus + D1_minus * dtheta1_res;
            break;
        }
        switch(direc2){
          case 0:
            tau_fric2 = F2_plus + D2_plus * dtheta2_res;
            break;
          case 1:
            tau_fric2 = F2_minus + D2_minus * dtheta2_res;
            break;
        }
        switch(direc3){
          case 0:
            tau_fric3 = F3_plus + D3_plus * dtheta3_res;
            break;
          case 1:
            tau_fric3 = F3_minus + D3_minus * dtheta3_res;
            break;
        }
        switch(direc4){
          case 0:
            tau_fric4 = F4_plus + D4_plus * dtheta4_res;
            break;
          case 1:
            tau_fric4 = F4_minus + D4_minus * dtheta4_res;
            break;
        }

        if( dtheta1_res < 0.5 && dtheta1_res > -0.5 ) tau_fric1 = 0.0;
        if( dtheta2_res < 0.5 && dtheta2_res > -0.5 ) tau_fric2 = 0.0;
        if( dtheta3_res < 0.5 && dtheta3_res > -0.5 ) tau_fric3 = 0.0;
        if( dtheta4_res < 0.5 && dtheta4_res > -0.5 ) tau_fric4 = 0.0;

        // * Utilized when identify F & D ( Utilize DFOB as second DOB. DFOB is an observer for identification only at this time. )
        // tau_fric1 = 0.0;
        // tau_fric2 = 0.0;
        // tau_fric3 = 0.0;
        // tau_fric4 = 0.0;
        // * Identification of F & D

        // * Continuous
        // integral_tau_dfob1 = integral_tau_dfob1 + ( Gear * Ktn * ia1_ref + M11 * G_DFOB * dtheta1_res - tau_fric1 - integral_tau_dfob1) * G_DFOB * dt;
        // integral_tau_dfob2 = integral_tau_dfob2 + ( Gear * Ktn * ia2_ref + M22 * G_DFOB * dtheta2_res - tau_fric2 - integral_tau_dfob2) * G_DFOB * dt;
        // integral_tau_dfob3 = integral_tau_dfob3 + ( Gear * Ktn * ia3_ref + M33 * G_DFOB * dtheta3_res - tau_fric3 - integral_tau_dfob3) * G_DFOB * dt;
        // integral_tau_dfob4 = integral_tau_dfob4 + ( Gear * Ktn * ia4_ref + M44 * G_DFOB * dtheta4_res - tau_fric4 - integral_tau_dfob4) * G_DFOB * dt;

        integral_tau_dfob1 = integral_tau_dfob1 + ( Gear * Ktn * ia1_ref + M11*G_DFOB*dtheta1_res + M12*G_DFOB*dtheta2_res + M13*G_DFOB*dtheta3_res + M14*G_DFOB*dtheta4_res - tau_fric1 - integral_tau_dfob1) * G_DFOB * dt;
        integral_tau_dfob2 = integral_tau_dfob2 + ( Gear * Ktn * ia2_ref + M21*G_DFOB*dtheta1_res + M22*G_DFOB*dtheta2_res + M23*G_DFOB*dtheta3_res + M24*G_DFOB*dtheta4_res - tau_fric2 - integral_tau_dfob2) * G_DFOB * dt;
        integral_tau_dfob3 = integral_tau_dfob3 + ( Gear * Ktn * ia3_ref + M31*G_DFOB*dtheta1_res + M32*G_DFOB*dtheta2_res + M33*G_DFOB*dtheta3_res + M34*G_DFOB*dtheta4_res - tau_fric3 - integral_tau_dfob3) * G_DFOB * dt;
        integral_tau_dfob4 = integral_tau_dfob4 + ( Gear * Ktn * ia4_ref + M41*G_DFOB*dtheta1_res + M42*G_DFOB*dtheta2_res + M43*G_DFOB*dtheta3_res + M44*G_DFOB*dtheta4_res - tau_fric4 - integral_tau_dfob4) * G_DFOB * dt;
        // * Continuous

        // * Backward Difference : Not work well
        // tau_dfob1 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob1_pre + G_DFOB * dt * ( Gear * Ktn * ia1_ref - tau_fric1 ) - G_DFOB*M11*(dtheta1_res - dtheta1_res_pre));
        // tau_dfob2 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob2_pre + G_DFOB * dt * ( Gear * Ktn * ia2_ref - tau_fric2 ) - G_DFOB*M22*(dtheta2_res - dtheta2_res_pre));
        // tau_dfob3 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob3_pre + G_DFOB * dt * ( Gear * Ktn * ia3_ref - tau_fric3 ) - G_DFOB*M33*(dtheta3_res - dtheta3_res_pre));
        // tau_dfob4 = 1.0 / ( G_DFOB * dt ) * ( tau_dfob4_pre + G_DFOB * dt * ( Gear * Ktn * ia4_ref - tau_fric4 ) - G_DFOB*M44*(dtheta4_res - dtheta4_res_pre));
        // * Backward Difference

          // * Save previous values
          tau_dfob1_pre = tau_dfob1;
          tau_dfob2_pre = tau_dfob2;
          tau_dfob3_pre = tau_dfob3;
          tau_dfob4_pre = tau_dfob4;
          // * Save previous values
        
        fd_hat1 = tau_dfob1 / Rw;// [N] Element of fd's wheel rotation direction
        fd_hat2 = tau_dfob2 / Rw;
        fd_hat3 = tau_dfob3 / Rw;
        fd_hat4 = tau_dfob4 / Rw;

        // fd_hat1 = tau_dfob1 / Rw * sqrt(2.0);// [N] Element of fd's wheel rotation direction
        // fd_hat2 = tau_dfob2 / Rw * sqrt(2.0);
        // fd_hat3 = tau_dfob3 / Rw * sqrt(2.0);
        // fd_hat4 = tau_dfob4 / Rw * sqrt(2.0);

        fx_hat = 1.0 / Rw             * (   tau_dfob1 - tau_dfob2 + tau_dfob3 - tau_dfob4 );// Substantially, 1.0 / sqrt(2.0) * fd : as of 2021/01/08
        fy_hat = 1.0 / Rw             * (   tau_dfob1 + tau_dfob2 + tau_dfob3 + tau_dfob4 );
        Mz_hat = 1.0 / Rw * ( L + W ) * ( - tau_dfob1 - tau_dfob2 + tau_dfob3 + tau_dfob4 );

        #endif

        #ifdef Enable_WOB
        WOB_x_input   = fx_ref - Mass * Acc_x_correct;
        WOB_y_input   = fy_ref - Mass * Acc_y_correct;
        // WOB_phi_input = Mz_ref - Jz   * 
        
        // F_x_dis = 1.0 / (2.0 + G_WOB * dt) * ( (2.0 - G_WOB * dt) * F_x_dis_pre + G_WOB * dt * ( WOB_x_input + WOB_x_input_pre ) );// LPF
        // F_y_dis = 1.0 / (2.0 + G_WOB * dt) * ( (2.0 - G_WOB * dt) * F_y_dis_pre + G_WOB * dt * ( WOB_y_input + WOB_y_input_pre ) );// LPF

        Fx_dis = 1.0 / (1.0 + G_WOB * dt) * ( Fx_dis_pre + G_WOB * dt * WOB_x_input );// LPF : Backward Difference
        Fy_dis = 1.0 / (1.0 + G_WOB * dt) * ( Fy_dis_pre + G_WOB * dt * WOB_y_input );// LPF : Backward Difference
        ddphi_dis = 1.0 / (1.0 + G_WOB * dt) * ( ddphi_dis_pre + G_WOB * dt * ddphi_ref - G_WOB * ( yaw_rate - yaw_rate_pre ) );// LPF + Pseudo Derivative : Backward Difference
        // Mz_dis = 1.0 / (1.0 + G_WOB * dt) * ( Mz_dis_pre + G_WOB * dt * Mz_ref - G_WOB * Jz * ( yaw_rate - yaw_rate_pre ) );// LPF + Pseudo Derivative : Backward Difference

        Fx_dis = - Fx_dis;// Apply + - sign direction to the sign direction of slip ratio observer
        Fy_dis = - Fy_dis;
        Mz_dis = - Jz * ddphi_dis;

          // * Save previous values
          // WOB_x_input_pre = WOB_x_input;
          // WOB_y_input_pre = WOB_y_input;

          Fx_dis_pre = Fx_dis;
          Fy_dis_pre = Fy_dis;
          ddphi_dis_pre = ddphi_dis;
          // Mz_dis_pre = Mz_dis;
          // * Save previous values
        #endif

        // * YMO ( Yaw Moment Observer )
        // M_YMO = 1.0 / (1.0 + G_YMO * dt) * ( M_YMO_pre + G_YMO * dt * YMO_input_pre );// LPF : Backward Difference
        M_YMO = 1.0 / (1.0 + G_YMO * dt) * ( M_YMO_pre + G_YMO * ( yaw_rate - yaw_rate_pre ) - G_YMO * dt * Mz_hat );// LPF + Pseudo Derivative : Backward Difference

          // * Save previous values
          M_YMO_pre     = M_YMO;
          // * Save previous values
        // * YMO ( Yaw Moment Observer )

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

        if(PWM1 >= PWM_rsl * 0.9){
          PWM1 = PWM_rsl * 0.85;
        }
        if(PWM2 >= PWM_rsl * 0.9){
          PWM2 = PWM_rsl * 0.85;
        }
        if(PWM3 >= PWM_rsl * 0.9){
          PWM3 = PWM_rsl * 0.85;
        }
        if(PWM4 >= PWM_rsl * 0.9){
          PWM4 = PWM_rsl * 0.85;
        }

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

        // * Save previous values
        Acc_x_correct_pre = Acc_x_correct;
        Acc_y_correct_pre = Acc_y_correct;
        d_yawrate_pre     = d_yawrate;

        // yaw_rate_pre2       = yaw_rate_pre;
        yaw_rate_pre        = yaw_rate;
        // yaw_rate_notch_pre2 = yaw_rate_notch_pre;
        // yaw_rate_notch_pre  = yaw_rate_notch;
        // * Save previous values

        // if(loop % 1000 == 0){
        // printf("%.3f, ", t);

    		// printf("%d, ", cnt1);
    		// printf("%d, ", cnt2);
    		// printf("%d, ", cnt3);
    		// printf("%d, ", cnt4);

    		// printf("%d, ", digit1);
    		// printf("%d, ", digit2);
    		// printf("%d, ", digit3);
    		// printf("%d, ", digit4);

    		// printf("%f, ", theta1_res);
    		// printf("%f, ", theta2_res);
    		// printf("%f, ", theta3_res);
    		// printf("%f, ", theta4_res);

    		// printf("%f, ", a);
    		// printf("%f, ", b);

    		// printf("%.8f, ", J1);
    		// printf("%f, ", M11);
    		// printf("%f, ", M12);
    		// printf("%f, ", M13);
    		// printf("%f, ", M14);

        // printf("%.3f, ", theta1_res);
        // printf("%.3f, ", theta2_res);
        // printf("%.3f, ", theta3_res);
        // printf("%.3f, ", theta4_res);

    		// printf("%.3f, ", dtheta1_res_raw);
    		// printf("%.3f, ", dtheta2_res_raw);
    		// printf("%.3f, ", dtheta3_res_raw);
    		// printf("%.3f, ", dtheta4_res_raw);

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

        // printf("%.3f, ", yaw);
        // printf("%.3f, ", yaw_digit);

        // printf("%.2f, %.2f, %.2f,   ", Euler.x, Euler.y, Euler.z);// yaw, roll, pitch
        // printf("%.2f, ", Gyro.x);
        // printf("%.2f, ", Gyro.y);
        // printf("%.2f,   ", Gyro.z);
        // printf("%.2f, %.2f, %.2f,   ", Acc.x,        Acc.y,        Acc.z);

        // printf("%f,   ", alpha_1);
        // printf("%f,   ", w1);
        // printf("%f,   ", w2);
        // printf("%f,   ", w3);
        // printf("%f,   ", w4);

        // printf("\r\n");
        // }

        // if(i_save < N_SRAM){
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

          ddtheta1_res_SRAM[i_save] = ddtheta1_res;
          ddtheta2_res_SRAM[i_save] = ddtheta2_res;
          ddtheta3_res_SRAM[i_save] = ddtheta3_res;
          ddtheta4_res_SRAM[i_save] = ddtheta4_res;

          // i1_ref_SRAM[i_save] = i1_ref;
          // i2_ref_SRAM[i_save] = i2_ref;
          // i3_ref_SRAM[i_save] = i3_ref;
          // i4_ref_SRAM[i_save] = i4_ref;

          ia1_ref_SRAM[i_save] = ia1_ref;
          ia2_ref_SRAM[i_save] = ia2_ref;
          ia3_ref_SRAM[i_save] = ia3_ref;
          ia4_ref_SRAM[i_save] = ia4_ref;

          PWM1_SRAM[i_save] = PWM1;
          PWM2_SRAM[i_save] = PWM2;
          PWM3_SRAM[i_save] = PWM3;
          PWM4_SRAM[i_save] = PWM4;

          fd1_ref_normal_SRAM[i_save] = fd1_ref_normal;
          fd1_ref_SRAM[i_save] = fd1_ref;

          fd2_ref_normal_SRAM[i_save] = fd2_ref_normal;
          fd2_ref_SRAM[i_save] = fd2_ref;

          fd3_ref_normal_SRAM[i_save] = fd3_ref_normal;
          fd3_ref_SRAM[i_save] = fd3_ref;

          fd4_ref_normal_SRAM[i_save] = fd4_ref_normal;
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

          yaw_SRAM[i_save]   = yaw;//Euler.x;
          roll_SRAM[i_save]  = roll;//Euler.y;
          pitch_SRAM[i_save] = pitch;//Euler.z;

          yaw_rate_SRAM[i_save]   = yaw_rate;//Gyro.z;

          // yaw_rate_notch_SRAM[i_save]  = yaw_rate_notch;

          roll_rate_SRAM[i_save]  = roll_rate;//Gyro.x;
          pitch_rate_SRAM[i_save] = pitch_rate;//Gyro.y;

          Acc_x_SRAM[i_save] = Acc_x;//Acc.x;
          Acc_y_SRAM[i_save] = Acc_y;//Acc.y;
          Acc_z_SRAM[i_save] = Acc_z;//Acc.z;

          Acc_x_correct_SRAM[i_save] = Acc_x_correct;
          Acc_y_correct_SRAM[i_save] = Acc_y_correct;
          Acc_z_correct_SRAM[i_save] = Acc_z_correct;

          Acc_x_LPF_SRAM[i_save] = Acc_x_LPF;
          Acc_y_LPF_SRAM[i_save] = Acc_y_LPF;
          d_yawrate_SRAM[i_save] = d_yawrate;

          Fx_dis_SRAM[i_save] = Fx_dis;
          Fy_dis_SRAM[i_save] = Fy_dis;
          Mz_dis_SRAM[i_save] = Mz_dis;

          M_YMO_SRAM[i_save] = M_YMO;

          alpha_1_SRAM[i_save] = alpha_1;
          alpha_2_SRAM[i_save] = alpha_2;
          alpha_3_SRAM[i_save] = alpha_3;
          alpha_4_SRAM[i_save] = alpha_4;

          alpha_1_hat_SRAM[i_save] = alpha_1_hat;
          alpha_2_hat_SRAM[i_save] = alpha_2_hat;
          alpha_3_hat_SRAM[i_save] = alpha_3_hat;
          alpha_4_hat_SRAM[i_save] = alpha_4_hat;

          tan_beta_1_hat_SRAM[i_save] = tan_beta_1_hat;
          tan_beta_2_hat_SRAM[i_save] = tan_beta_2_hat;
          tan_beta_3_hat_SRAM[i_save] = tan_beta_3_hat;
          tan_beta_4_hat_SRAM[i_save] = tan_beta_4_hat;

          Kappa_1_SRAM[i_save] = Kappa_1;
          Kappa_2_SRAM[i_save] = Kappa_2;
          Kappa_3_SRAM[i_save] = Kappa_3;
          Kappa_4_SRAM[i_save] = Kappa_4;

          #ifdef Enable_Slip_Ratio_Observer
          lambda_1_hat_SRAM[i_save] = lambda_1_hat;
          lambda_2_hat_SRAM[i_save] = lambda_2_hat;
          lambda_3_hat_SRAM[i_save] = lambda_3_hat;
          lambda_4_hat_SRAM[i_save] = lambda_4_hat;

          lambda_1_hat_acc_SRAM[i_save] = lambda_1_hat_acc;
          lambda_2_hat_acc_SRAM[i_save] = lambda_2_hat_acc;
          lambda_3_hat_acc_SRAM[i_save] = lambda_3_hat_acc;
          lambda_4_hat_acc_SRAM[i_save] = lambda_4_hat_acc;

          d_lambda_1_hat_SRAM[i_save] = d_lambda_1_hat;
          d_lambda_2_hat_SRAM[i_save] = d_lambda_2_hat;
          d_lambda_3_hat_SRAM[i_save] = d_lambda_3_hat;
          d_lambda_4_hat_SRAM[i_save] = d_lambda_4_hat;

          d_lambda_1_hat_acc_SRAM[i_save] = d_lambda_1_hat_acc;
          d_lambda_2_hat_acc_SRAM[i_save] = d_lambda_2_hat_acc;
          d_lambda_3_hat_acc_SRAM[i_save] = d_lambda_3_hat_acc;
          d_lambda_4_hat_acc_SRAM[i_save] = d_lambda_4_hat_acc;

          v1_hat_SRAM[i_save] = v1_hat;
          v2_hat_SRAM[i_save] = v2_hat;
          v3_hat_SRAM[i_save] = v3_hat;
          v4_hat_SRAM[i_save] = v4_hat;

          v1_hat_acc_SRAM[i_save] = v1_hat_acc;
          v2_hat_acc_SRAM[i_save] = v2_hat_acc;
          v3_hat_acc_SRAM[i_save] = v3_hat_acc;
          v4_hat_acc_SRAM[i_save] = v4_hat_acc;
          #endif

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
    if( t == 0.0 || t > t_experiment ){// 14.999
		  mode++;
    }
		// printf("%d, ", mode);
		// printf("\r\n");

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

        if( isFirst == 0 ){
          tau_dfob1 = 0.0;
          tau_dfob2 = 0.0;
          tau_dfob3 = 0.0;
          tau_dfob4 = 0.0;

          tau_dfob1_pre = 0.0;
          tau_dfob1_pre = 0.0;
          tau_dfob1_pre = 0.0;
          tau_dfob1_pre = 0.0;
          
          integral_tau_dfob1 = 0.0;
          integral_tau_dfob2 = 0.0;
          integral_tau_dfob3 = 0.0;
          integral_tau_dfob4 = 0.0;

          isFirst++;
        }


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

          printf("%f, ", ddtheta1_res_SRAM[i_output]);
          printf("%f, ", ddtheta2_res_SRAM[i_output]);
          printf("%f, ", ddtheta3_res_SRAM[i_output]);
          printf("%f, ", ddtheta4_res_SRAM[i_output]);

          // printf("%f, ", i1_ref_SRAM[i_output]);
          // printf("%f, ", i2_ref_SRAM[i_output]);
          // printf("%f, ", i3_ref_SRAM[i_output]);
          // printf("%f, ", i4_ref_SRAM[i_output]);

          printf("%f, ", ia1_ref_SRAM[i_output]);
          printf("%f, ", ia2_ref_SRAM[i_output]);
          printf("%f, ", ia3_ref_SRAM[i_output]);
          printf("%f, ", ia4_ref_SRAM[i_output]);

          printf("%d, ", PWM1_SRAM[i_output]);
          printf("%d, ", PWM2_SRAM[i_output]);
          printf("%d, ", PWM3_SRAM[i_output]);
          printf("%d, ", PWM4_SRAM[i_output]);

          printf("%f, ", fd1_ref_normal_SRAM[i_output]);
          printf("%f, ", fd1_ref_SRAM[i_output]);
          
          printf("%f, ", fd2_ref_normal_SRAM[i_output]);
          printf("%f, ", fd2_ref_SRAM[i_output]);

          printf("%f, ", fd3_ref_normal_SRAM[i_output]);
          printf("%f, ", fd3_ref_SRAM[i_output]);

          printf("%f, ", fd4_ref_normal_SRAM[i_output]);
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

          printf("%f, ", yaw_SRAM[i_output]);
          printf("%f, ", roll_SRAM[i_output]);
          printf("%f, ", pitch_SRAM[i_output]);

          printf("%f, ", yaw_rate_SRAM[i_output]);
          // printf("%f, ", yaw_rate_notch_SRAM[i_output]);

          printf("%f, ", roll_rate_SRAM[i_output]);
          printf("%f, ", pitch_rate_SRAM[i_output]);

          printf("%f, ", Acc_x_SRAM[i_output]);
          printf("%f, ", Acc_y_SRAM[i_output]);
          printf("%f, ", Acc_z_SRAM[i_output]);

          printf("%f, ", Acc_x_correct_SRAM[i_output]);
          printf("%f, ", Acc_y_correct_SRAM[i_output]);
          printf("%f, ", Acc_z_correct_SRAM[i_output]);

          printf("%f, ", Acc_x_LPF_SRAM[i_output]);
          printf("%f, ", Acc_y_LPF_SRAM[i_output]);
          printf("%f, ", d_yawrate_SRAM[i_output]);

          printf("%f, ", Fx_dis_SRAM[i_output]);
          printf("%f, ", Fy_dis_SRAM[i_output]);
          printf("%f, ", Mz_dis_SRAM[i_output]);
          
          printf("%f, ", M_YMO_SRAM[i_output]);

          printf("%f, ", alpha_1_SRAM[i_output]);
          printf("%f, ", alpha_2_SRAM[i_output]);
          printf("%f, ", alpha_3_SRAM[i_output]);
          printf("%f, ", alpha_4_SRAM[i_output]);

          printf("%f, ", alpha_1_hat_SRAM[i_output]);
          printf("%f, ", alpha_2_hat_SRAM[i_output]);
          printf("%f, ", alpha_3_hat_SRAM[i_output]);
          printf("%f, ", alpha_4_hat_SRAM[i_output]);

          printf("%f, ", tan_beta_1_hat_SRAM[i_output]);
          printf("%f, ", tan_beta_2_hat_SRAM[i_output]);
          printf("%f, ", tan_beta_3_hat_SRAM[i_output]);
          printf("%f, ", tan_beta_4_hat_SRAM[i_output]);

          printf("%f, ", Kappa_1_SRAM[i_output]);
          printf("%f, ", Kappa_2_SRAM[i_output]);
          printf("%f, ", Kappa_3_SRAM[i_output]);
          printf("%f, ", Kappa_4_SRAM[i_output]);

          #ifdef Enable_Slip_Ratio_Observer
          printf("%f, ", lambda_1_hat_SRAM[i_output]);
          printf("%f, ", lambda_2_hat_SRAM[i_output]);
          printf("%f, ", lambda_3_hat_SRAM[i_output]);
          printf("%f, ", lambda_4_hat_SRAM[i_output]);

          printf("%f, ", lambda_1_hat_acc_SRAM[i_output]);
          printf("%f, ", lambda_2_hat_acc_SRAM[i_output]);
          printf("%f, ", lambda_3_hat_acc_SRAM[i_output]);
          printf("%f, ", lambda_4_hat_acc_SRAM[i_output]);

          printf("%f, ", d_lambda_1_hat_SRAM[i_output]);
          printf("%f, ", d_lambda_2_hat_SRAM[i_output]);
          printf("%f, ", d_lambda_3_hat_SRAM[i_output]);
          printf("%f, ", d_lambda_4_hat_SRAM[i_output]);
          
          printf("%f, ", d_lambda_1_hat_acc_SRAM[i_output]);
          printf("%f, ", d_lambda_2_hat_acc_SRAM[i_output]);
          printf("%f, ", d_lambda_3_hat_acc_SRAM[i_output]);
          printf("%f, ", d_lambda_4_hat_acc_SRAM[i_output]);
          
          printf("%f, ", v1_hat_SRAM[i_output]);
          printf("%f, ", v2_hat_SRAM[i_output]);
          printf("%f, ", v3_hat_SRAM[i_output]);
          printf("%f, ", v4_hat_SRAM[i_output]);

          printf("%f, ", v1_hat_acc_SRAM[i_output]);
          printf("%f, ", v2_hat_acc_SRAM[i_output]);
          printf("%f, ", v3_hat_acc_SRAM[i_output]);
          printf("%f, ", v4_hat_acc_SRAM[i_output]);
          #endif

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
  MX_I2C1_Init();
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

  #ifdef Enable_I2C
  bno055_assignI2C(&hi2c1);
  bno055_reset();
  bno055_setup();
  bno055_setOperationModeNDOF();
  #endif

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0,  GPIO_PIN_SET); // Green

//  printf("\r\n initialized Success!!\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    #ifdef Enable_I2C
    Euler      = bno055_getVectorEuler();
    Gyro       = bno055_getVectorGyroscope();
    Acc        = bno055_getVectorAccelerometer();
    // Acc_Linear = bno055_getVectorLinearAccel();
    #endif
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
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2010091A;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
