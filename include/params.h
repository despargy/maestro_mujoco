#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#ifndef _PARAMS_H_
#define _PARAMS_H_


#define param_ADAPT_A false;
#define param_ADAPT_B false;

#define param_KEEP_CONTROL true;



#define GO1_param_mass 12.0; // go1
#define GO1_UNITREE_param_mass 13.0; // go1 unitree  
#define GO2_UNITREE_param_mass 15.0; // unitree go2 

#define param_g_gravity 9.81;

#define param_pbc_x 0.0;// 0.11 gia 0.6
#define param_pbc_y 0.0;//0.001;//0.0125;
#define param_pbc_z 0;


#define param_dt 0.002;

#define param_alpha 150.0;
#define param_slope 0.0001;

#define param_w0 50;
#define param_w_max 10000000; // 70000


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

#define param_thalf_Swing_DYNA 10.3;
#define param_slot_Swing_DYNA 10.8;
#define param_A_SGaus_DYNA 1.0;
#define param_b_SGaus_DYNA 10.0;
#define param_t0_SGaus_DYNA 0.008;


/********************** DYNAMIC LOCOMOTION  FAST *************************/

#define param_DYNA_LOCO true;
#define param_ko_DYNA 800;  //800
#define param_kv_DYNA 1500; //3500
#define param_kw_DYNA 70; //70
#define param_kp_DYNA 1000; //3000

#define param_Freq_Swing_DYNA 8.0; // 8.0
#define param_t0_Swing_DYNA 0.01; //0.01
#define param_c1 100; //100
#define param_force_thres -10;//0.01;  //0.01
#define param_c1tip 500;  //200
#define param_c2tip 0.02; //0.02
#define param_tip_target_z 0.0192;
#define param_percentage 0.95;  //0.95 
 
/********************** DYNAMIC LOCOMOTION  FAST *************************/


/**********************         MODEL BASED      *************************/


#define GO1_param_dp_cmd_x 0.6; // go1 0.5 h 0.6  m/s
#define GO1_param_dp_cmd_y 0.0; // m/s
#define GO1_param_dp_cmd_z 0.0; // m/s

#define GO1_UNITREE_param_dp_cmd_x 0.75; //unitree go1 0.75 h 0.8 m/s
#define GO1_UNITREE_param_dp_cmd_y 0.0; // m/s
#define GO1_UNITREE_param_dp_cmd_z 0.0; // m/s

#define GO2_UNITREE_param_dp_cmd_x 0.68; //0.68
#define GO2_UNITREE_param_dp_cmd_y 0.0; // m/s
#define GO2_UNITREE_param_dp_cmd_z 0.0; // m/s

#define GO2_UNITREE_param_w_cmd_x 0.0; //rad/s
#define GO2_UNITREE_param_w_cmd_y 0.0; // rad/s
#define GO2_UNITREE_param_w_cmd_z 0.0; // rad/s


#define GO1_param_robot_z 0.32; // go1 ->0.32, unitree go1  0.34
#define GO1_UNITREE_param_robot_z 0.34; // go1 ->0.32, unitree go1  0.34
#define GO2_UNITREE_param_robot_z 0.4; // go1 ->0.32, unitree go1  0.34

#define GO1_param_Kp_hip 8;     //go1 8
#define GO1_param_Kp_thing 5;   //go1 5
#define GO1_param_Kp_calf 3;    //go1 3

#define GO1_UNITREE_param_Kp_hip 16;     //unitree go1  16
#define GO1_UNITREE_param_Kp_thing 10;   //unitree go1  10
#define GO1_UNITREE_param_Kp_calf 6;     //unitree go1  6

#define GO2_UNITREE_param_Kp_hip 16;     //unitree go1  16
#define GO2_UNITREE_param_Kp_thing 10;   //unitree go1  10
#define GO2_UNITREE_param_Kp_calf 6;     //unitree go1  6


#define GO1_param_alpha_DYNA 0.0;//  less 0 ->100000;
#define GO1_UNITREE_param_alpha_DYNA 0.0;//  less 0 ->100000;
#define GO2_UNITREE_param_alpha_DYNA 1000.0;//  less 0 ->100000; 1000000.0 //10000

#define GO1_param_k_clik 96; //go 1 64 
#define GO1_UNITREE_param_k_clik 96; //go 1 64 
#define GO2_UNITREE_param_k_clik 192; //go 1 64  192

#define GO1_param_d 0.128;
#define GO2_UNITREE_param_d 0.128;

#define GO1_param_l1 0.17;
#define GO1_param_l2 0.19;

#define GO1_UNITREE_param_l1 0.19;
#define GO1_UNITREE_param_l2 0.23;

#define GO2_UNITREE_param_l1 0.17;
#define GO2_UNITREE_param_l2 0.25; // 0.23

#define GO1_param_sit1_0 0.0;
#define GO1_param_sit1_1 0.8;
#define GO1_param_sit1_2 -1.5;

#define GO2_UNITREE_param_sit1_0 0.0;
#define GO2_UNITREE_param_sit1_1 0.8;
#define GO2_UNITREE_param_sit1_2 -1.5;

#define param_tau_lim 200;

#define param_model 2; // 0 -> go1 , 1 -> unitree o1, 2 -> unitree go2

#define param_terrain_height 0.01; 

/********************** ******************** *************************/
#endif

// gia vel 0.6 , 8.0 freq , kp,kv ->1000



// param_Freq_Swing_DYNA 4.0 +  param_dp_cmd_x 0.2; thress 50, 100, 20 20
// param_Freq_Swing_DYNA 8.0 +  param_dp_cmd_x 0.5;
// param_Freq_Swing_DYNA 10.0 +  param_dp_cmd_x 0.8;
// param_Freq_Swing_DYNA 16.0 +  param_dp_cmd_x 1.0;


// clik was 96 -> 164
//clik 164,freq swig 16.0, force thress 0.001, param_Freq_Swing_DYNA 16.0 +  param_dp_cmd_x 1.2; kv = 1000 -> 1.1
//clik 164,freq swig 16.0, force thress 0.001, param_Freq_Swing_DYNA 16.0 +  param_dp_cmd_x 1.0; kv = 1000 -> 0.92
// param_Freq_Swing_DYNA 8.0 +  param_dp_cmd_x 1.0; kv = 1000 -> 0.92


// Simualtion flow idea
//without PCE, 
    //clik 164,freq swig 16.0, force thress 0.001, param_Freq_Swing_DYNA 16.0 +  param_dp_cmd_x 1.2; kv = 1000 -> 1.1
    //clik 164,freq swig 16.0, force thress 0.001, param_Freq_Swing_DYNA 16.0 +  param_dp_cmd_x 0.8; kv = 1000 -> 1.1

//with PCE,
    //CoM lower, less W so
