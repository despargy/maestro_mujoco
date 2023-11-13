#include <Robot.h>
#include <Math.h>
#include <Trajectory.h>

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_


class Controller
{
    public:

        Robot* robot; // Robot pointer obj.
        Trajectory* traj; // Trajectory pointer obj.

        double t_real, t0, tv , d_tv, t_to_use, dt, t0_phase ; // time variables for adaptive tracking
        int loop_index; // for smooth stand up
        bool ADAPT_A, ADAPT_B;
        double kp, kv, ko, ki;
        double slope, alpha;
        int swing_leg;
        int* vp_order;
        // TODO delete if not use them 
        double thres_r, sat_force;
        

        // Error variables
        Eigen::Vector3d e_p, e_o; 
        Eigen::Vector3d e_p_int, e_o_int;
        Eigen::VectorXd e_v;
        Eigen::Vector3d RcRdTwd;
        // vector to help with eq. 11
        Eigen::VectorXd fcontrol1,fcontrol2,fcontrol3, pid_out; 
        Eigen::AngleAxisd ang;
        Eigen::Matrix3d Re;

        Eigen::MatrixXd Gbc;


        Controller();
        ~Controller();

        void PD(double* target);
        void PD_smooth(double* target, double smooth);

        /* Tracking */
        void initTracking(double time_now);
        void Tracking(double time_now);
        void positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d); 
        void velocityError(Eigen::Vector3d dp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d);
        void updateControlLaw();
        void computeSudoGq();
        void computeWeights();
        void fComputations(Eigen::Vector3d dp_d, Eigen::Vector3d ddp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d );
        
        /* Target */
        void initTarget(double top_time); // target is the centroid of polygon
        void initTarget(double top_time, Eigen::Vector3d target_); // overload with target as input
        void ReachTarget(double top_time);
        void velocityError();
        void fComputations();
        void PIDwithSat();
        void setPhaseTarget(int phase, double time_now);

        /* Locomotion */
        void LocomotionTarget(double time_now);
        void LocomotionSwing();

};

#endif