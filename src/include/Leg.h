#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
// #include <boost/scoped_ptr.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/treejnttojacsolver.hpp>
// #include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolver.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames.hpp>
// #include <tf/tf.h>

#ifndef _LEG_H_
#define _LEG_H_

class Leg
{   
    public:
        
        double sit0[3] = {0.0, 1.13, -2.7};
        double sit1[3] = {0.0, 0.67, -1.5};
        
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

        KDL::JntArray q, dq;    // Joint pos qs
        // KDL::Chain kdl_chain;
        // KDL::Frame p_frame;          // Tip pose with respect to CoM 
        // KDL::Jacobian jacobian_kdl;
        // boost::scoped_ptr<KDL::ChainJntToJacSolver> kdl_solver;
        // boost::scoped_ptr<KDL::ChainFkSolverPos> kdl_solver_pos;
        
        Eigen::Vector3d f, f_cmd, tau; //applyied force to the tip 
        Eigen::MatrixXd J;            // Jacobian Eigen
        Eigen::Vector3d p_i;

        Leg();
        // Leg(KDL::Tree robot_kin);
        ~Leg();
        // void kdlSolver();

};

#endif