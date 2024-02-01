#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#ifndef _PARAMS_H_
#define _PARAMS_H_

#define param_ADAPT_A false;
#define param_ADAPT_B false;
#define param_KEEP_CONTROL true;

#define param_mass 12.0;
#define param_g_gravity 9.81;

#define param_pbc_x 0.01;// 0.0025;//-0.175;//-0.087; without default geom friction and floor friction 1 1 1 
#define param_pbc_y 0.01;//0.001;//0.0125;
#define param_pbc_z 0;


#define param_dt 0.002;

#define param_alpha 150.0;
#define param_slope 0.0001;

#define param_w0 50;
#define param_w_max 70000;

#define param_Kp_hip 5; //8
#define param_Kp_thing 3;  //5
#define param_Kp_calf 2; //3

#define param_Kv_hip 3.5;
#define param_Kv_thing 2.0 ;
#define param_Kv_calf 2.0;

/**********************  TRACKING   *************************/

#define param_kp_TRACKING 2800;//2800;
#define param_kv_TRACKING 350;//350.0;
#define param_ko_TRACKING 15;//15.0;

/********************** STATIC LOCOMOTION   *************************/

#define param_kp_LOC 250;//250; //150 reduces time, increases oscillations 
#define param_kv_LOC 10;//30.0;  //20   reduces oscillations, increases time 
#define param_ko_LOC 100;//100;  //200; 100
#define param_ki_LOC 20;//20; //80; 20   increases oscillations, decreases steady state error

#define param_kx_r_LOC 100; // gain region controller
#define param_ky_r_LOC 100; // gain region controller
#define param_kz_r_LOC 0; // gain region controller

/* Decide one of the above tasks/modes */
#define param_TRACKING_MODE true;
#define param_TARGET_MODE false;
#define param_LOCOMOTION_MODE false;

// Locomotion Static Gait

#define param_A_SGaus 1.0;
#define param_b_SGaus 10.0;
#define param_t0_SGaus 0.3;

#define param_Freq_Swing  2.0;//3.0;
#define param_t0_Swing    0.4;
#define param_thalf_Swing 0.65;//1.1;
#define param_slot_Swing  1.1;//1.8;

/**********************  EXPLORATION   *************************/

#define param_kp_EXP 150;
#define param_kv_EXP 20;
#define param_ko_EXP 100;
#define param_ki_EXP 20;

#define param_Freq_Swing_EXP 2;
#define param_t0_Swing_EXP   0.8;//0.3;

/**********************  DYNAMIC LOCOMOTION  FAST *************************/

// #define param_DYNA_LOCO true;
// #define param_kv_DYNA 20; //20
// #define param_ko_DYNA 1000; //100

// #define param_A_SGaus_DYNA 1.0;
// #define param_b_SGaus_DYNA 10.0;
// #define param_t0_SGaus_DYNA 0.008;
// #define param_Freq_Swing_DYNA 20.0;
// #define param_t0_Swing_DYNA 0.02;
// #define param_thalf_Swing_DYNA 0.05;
// #define param_slot_Swing_DYNA 0.11;
// #define param_dp_cmd 2.7;
// // SET THIS TO OFFSET Eigen::Vector3d(0.1, controller->robot->leg[(int)controller->robot->swingL_id_a]->pros*0.01, 0.02)

/********************** ******************** *************************/
#define param_thalf_Swing_DYNA 10.3;
#define param_slot_Swing_DYNA 10.8;
#define param_A_SGaus_DYNA 1.0;
#define param_b_SGaus_DYNA 10.0;
#define param_t0_SGaus_DYNA 0.008;
/**********************  DYNAMIC LOCOMOTION  FAST *************************/

// #define param_DYNA_LOCO true;
// #define param_ko_DYNA 1000; //100
// #define param_kv_DYNA 20; //20

// #define param_Freq_Swing_DYNA 2.0;
// #define param_t0_Swing_DYNA 0.1;

// #define param_dp_cmd 0.0; // m/s

// #define param_c1 100;
// #define param_c2 0.1;
// #define param_force_thres 0.01;

// ofset x / t_slot = dp_cmd

/********************** ******************** *************************/

/**********************  DYNAMIC LOCOMOTION  FAST *************************/

#define param_DYNA_LOCO true;
#define param_ko_DYNA 500; //1000
#define param_kv_DYNA 100; //20
#define param_kp_DYNA 30;

#define param_Freq_Swing_DYNA 4.0;
#define param_t0_Swing_DYNA 0.1;

#define param_dp_cmd_x 0.2; // m/s
#define param_dp_cmd_y 0.0; // m/s
#define param_dp_cmd_z 0.0; // m/s

#define param_c1 100;
#define param_force_thres 0.001;

#define param_c1tip 200; //200
#define param_c2tip 0.02; //0.02
#define param_tip_target_z 0.0192;

#define param_percentage 0.95;
// ofset x / t_slot = dp_cmd
#define param_robot_z 0.32;

/********************** ******************** *************************/
#endif