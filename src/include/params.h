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

#define param_pbc_x -0.097;//-0.175;//-0.087; without default geom friction and floor friction 1 1 1 
#define param_pbc_y 0.0;//0.0125;
#define param_pbc_z 0.0;

#define param_kp_TRACKING 2800;
#define param_kv_TRACKING 350.0;
#define param_ko_TRACKING 15.0;

#define param_kp_TARGET 2400; //2400 200 180 220 reduces time, increases oscillations 
#define param_kv_TARGET 350.0;  //350  40  40 10   reduces oscillations, increases time 
#define param_ko_TARGET 1100;  //1100 40  40 20
#define param_ki_TARGET 350;   //350  0   0 100  increases oscillations, decreases steady state error

#define param_robot_z 0.3;

#define param_dt 0.002;

#define param_alpha 150.0;
#define param_slope 0.0001;

// #define param_thres_r 0.06;

// #define param_sat_force 0.1;

/* Decide one of the above tasks/modes */
#define param_TRACKING_MODE true;
#define param_TARGET_MODE false;
#define param_LOCOMOTION_MODE false;

// Locomotion Static Gait

#define param_A_SGaus 1.0;
#define param_b_SGaus 10.0;
#define param_t0_SGaus 1.0;

#define param_Freq_Swing  1.0;
#define param_t0_Swing    1.2;
#define param_thalf_Swing 1.7;
#define param_slot_Swing  2.5;







#endif