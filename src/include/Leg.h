#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <kdl/chainfksolver.hpp>


#ifndef _LEG_H_
#define _LEG_H_

class Leg
{   
    public:
        
        double sit0[3] = {0.0, 1.13, -2.7};
        double sit1[3] = {0.0, 0.87, -1.4};
        
        std::string name;
        int id, n_superV_joints;
        /* Weights*/
        double w0;
        Eigen::Vector3d wv_leg;
        /* Probability of Stable Contact */
        double prob_stable;
        
        /* Mujoco id's-variables*/
        int body__, site__;
        int q_hip__, q_thigh__, q_calf__ ; // joint position
        int dq_hip__, dq_thigh__, dq_calf__ ; // joint velocity
        int torque_hip__, torque_thigh__, torque_calf__ ; // joint position
        int tip_x__, tip_y__, tip_z__; // tip pos ???????
        int quat_w__, quat_x__, quat_y__, quat_z__; // quat pos  ????????
        int acc_x__, acc_y__, acc_z__; // tip acc
        int gyro_x__, gyro_y__, gyro_z__ ;  // tip acc
        int force_x__, force_y__, force_z__;

        int cmd_q_hip__, cmd_q_thigh__, cmd_q_calf__;
        int cmd_dq_hip__, cmd_dq_thigh__, cmd_dq_calf__;

        KDL::JntArray q, dq, q_out;    // Joint pos qs
        Eigen::Vector3d dq_out;

        Eigen::Vector3d f, f_cmd, tau; //applyied force to the tip 
        Eigen::MatrixXd J;            // Jacobian Eigen
        Eigen::Vector3d p_i;
        Eigen::Matrix3d R_i;

        Eigen::Matrix4d g_0bo_init; // store init leg config in each phase before swing
        Eigen::Matrix4d g_o_world, g_o; // store init leg config in each phase before swing

        Leg();
        ~Leg();
        
        void storeInitG();
        void initQout();

};

#endif