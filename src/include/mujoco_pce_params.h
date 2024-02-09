#ifndef _MUJOCO_PCE_PARAMS_H_
#define _MUJOCO_PCE_PARAMS_H_

// LEG 0
#define LEG0_ax_bias  6.6466;
#define LEG0_ay_bias  -0.016673;
#define LEG0_az_bias  7.21504;
#define LEG0_wx_bias  -2.69784e-05;
#define LEG0_wy_bias  0.00509618;
#define LEG0_wz_bias  -7.55211e-06;

#define LEG0_ax_std  0.0301358;
#define LEG0_ay_std  8.56867e-05;
#define LEG0_az_std  0.0278302;
#define LEG0_wx_std  1.59764e-05;
#define LEG0_wy_std  0.0026324;
#define LEG0_wz_std  2.28894e-05;
// LEG 1
#define LEG1_ax_bias 6.64859;
#define LEG1_ay_bias 0.0055141;
#define LEG1_az_bias 7.21323;
#define LEG1_wx_bias 0.000149642;
#define LEG1_wy_bias 0.00519746;
#define LEG1_wz_bias -0.000170278;

#define LEG1_ax_std  0.0307933;
#define LEG1_ay_std  0.00201243;
#define LEG1_az_std  0.0284525;
#define LEG1_wx_std  1.83086e-05;
#define LEG1_wy_std  0.00264352;
#define LEG1_wz_std  2.4286e-05;
// LEG 2
#define LEG2_ax_bias 6.70788;
#define LEG2_ay_bias -0.0110305;
#define LEG2_az_bias 7.15813;
#define LEG2_wx_bias -4.6159e-05;
#define LEG2_wy_bias 0.00518883;
#define LEG2_wz_bias 1.06019e-05;

#define LEG2_ax_std  0.0303551;
#define LEG2_ay_std  0.000288458;
#define LEG2_az_std  0.0285103;
#define LEG2_wx_std  1.11579e-05;
#define LEG2_wy_std  0.00265308;
#define LEG2_wz_std  1.84438e-05;
// LEG 3
#define LEG3_ax_bias 6.71018;
#define LEG3_ay_bias 0.00331892;
#define LEG3_az_bias 7.15597;
#define LEG3_wx_bias 0.000119148;
#define LEG3_wy_bias 0.00527494;
#define LEG3_wz_bias -0.000144362;

#define LEG3_ax_std  0.0309065;
#define LEG3_ay_std  0.00164658;
#define LEG3_az_std  0.0290476;
#define LEG3_wx_std  1.36955e-05;
#define LEG3_wy_std  0.00266415;
#define LEG3_wz_std  1.97416e-05;

// Thressholds Ax,Ay,Az
#define LEG0_thres_ax  3;//2; 
#define LEG0_thres_ay  3;//2; 
#define LEG0_thres_az  3;//2; 
#define LEG1_thres_ax  3;//2;
#define LEG1_thres_ay  3;//2;
#define LEG1_thres_az  3;//2;
#define LEG2_thres_ax  3;//2;
#define LEG2_thres_ay  3;//2;
#define LEG2_thres_az  3;//2;
#define LEG3_thres_ax  3;//2;
#define LEG3_thres_ay  3;//2;
#define LEG3_thres_az  3;//2;

// Thressholds Wx,Wy,Wz
#define LEG0_thres_wx  1.0;//0.5
#define LEG0_thres_wy  1.0;//0.5
#define LEG0_thres_wz  0.6;//0.5

#define LEG1_thres_wx  1.0;//0.5
#define LEG1_thres_wy  1.0;//0.5
#define LEG1_thres_wz  0.6;//0.5

#define LEG2_thres_wx  1.0; // 0.5
#define LEG2_thres_wy  1.0;// 0.5
#define LEG2_thres_wz  0.6;// 0.5

#define LEG3_thres_wx  1.0//0.5
#define LEG3_thres_wy  1.0;//0.4
#define LEG3_thres_wz  0.6; //0.6


#define mujoco_batch_size 50;
#define mujoco_eval_samples 300;

#endif
