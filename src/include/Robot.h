#include <Leg.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Math.h>

#ifndef _ROBOT_H_
#define _ROBOT_H_


class Robot
{
    public:
       
        int n_legs = 4;
        Leg **leg; 
        
        double mass, g_gravity;
        double height_z; 
        double swingL_id;
        double swingL_id_a, swingL_id_b, stanceL_id_a, stanceL_id_b;
        double l1, l2, d;

        /* Reference to sensordata ids CoM [.] - Mujoco xml*/
        int acc_x__ , acc_y__ , acc_z__ ; // Body acceleration ids
        int gyro_x__, gyro_y__, gyro_z__; // Body gyro    ids
        int com_x__ , com_y__ , com_z__ ; // Body position ids (x,y,z`)
        int quat_w__, quat_x__, quat_y__, quat_z__; // Body quaternion ids (w,x,y,z)
        int vel_x__, vel_y__, vel_z__; // Body velocity ids(x,y,z)

        /* Pose CoM */
        Eigen::Vector3d p_c, p_c0; Eigen::Matrix3d R_c, R_c0;
        Eigen::Vector3d dp_c; // Just added
        Eigen::Vector3d com_p_prev, dCoM_p;
        Eigen::Vector3d w_CoM;
        Eigen::Matrix3d R_CoM_prev, dR_CoM;

        Eigen::MatrixXd Gq, Gq_sudo, H_c, C_c;
        Eigen::Matrix3d I, I_c;

        Eigen::VectorXd vvvv, F_a, F_c, gc;
        Eigen::DiagonalMatrix<double,12> W_inv;
        Eigen::Vector3d pbc ;

        Eigen::Matrix4d g_com ;

        bool KEEP_CONTROL;
        // KDL::Tree robot_kin;

        Robot();
        ~Robot();



};

#endif