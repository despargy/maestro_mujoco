#include <vector>
#include <Math.h>

#ifndef _TRAJECTORY_H_
#define _TRAJECTORY_H_

class Trajectory
{
    public:
        // vectors for body trajectory tracking 
        Eigen::Vector3d p_d, dp_d, ddp_d;
        // init state for tracking
        Eigen::Vector3d p_d0; Eigen::Matrix3d R_d_0; Eigen::Quaterniond Q_0; 
        // Desired orientation variables
        Eigen::Matrix3d R_d; Eigen::Matrix3d dR_d;
        Eigen::Vector3d w_d;
        
        // vector for target position, inside locomotion mode
        Eigen::Vector3d p_T, p0_ofphase; 
        Eigen::Matrix3d R_T; Eigen::Quaterniond Q0_ofphase; 
        

        
        Trajectory();
        ~Trajectory();
        /* Tracking Periodic Motion*/
        void nextDesiredTraj(double dt, double t_to_use);
        Eigen::Vector3d get_pDesiredTrajectory(double t_real);
        Eigen::Vector3d get_dpDesiredTrajectory(Eigen::Vector3d p_d_prev, double dt, double t_real);
        Eigen::Vector3d get_ddpDesiredTrajectory(Eigen::Vector3d p_d_prev,Eigen::Vector3d dp_d_prev, double dt, double t_real);
        Eigen::Matrix3d get_RDesiredRotationMatrix(double t_real);
        Eigen::Matrix3d get_dRDesiredRotationMatrix(Eigen::Matrix3d R_prev, double dt, double t_real);        
        

};

#endif