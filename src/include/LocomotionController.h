#include <Controller.h>


#ifndef _LOCOMOTIONCONTROLLER_H_
#define _LOCOMOTIONCONTROLLER_H_


class LocomotionController : public Controller
{
    public:

        double t0_phase, t_phase ; // time variables for adaptive tracking

        double A, b, t0_superG;
        double freq_swing, t0_swing, t_half_swing, swing_t_slot;
        double w_max;
        
        int swing_leg, ii;
        int* vp_order;
        int* static_free_gait;

        std::vector<double> bCurveX, dot_bCurveX; // Bezier Curve swinging tip
        std::vector<double> bCurveZ, dot_bCurveZ; // Bezier Curve swinging tip

        // vector for target position, inside locomotion mode
        Eigen::Vector3d p_T;
        // Eigen::Vector3d p0_ofphase; 
        Eigen::Matrix3d R_T;

        // // Error variables
        Eigen::Vector3d e_p_int, e_o_int;
        Eigen::VectorXd  pid_out; 

        Eigen::Vector4f d_tip_pos;
        Eigen::Vector3f d_tip_vel;

        LocomotionController();
        ~LocomotionController();


        void PD(double* target);
        void PD_smooth(double* target, double smooth);
        
        void setPhaseTarget();
        void computeWeightsSwing();
        void errors();
        void computeSudoGq();   
        void PIDwithSat();
        void fComputations();
        void inverseTip();
        void computeBesierCurve2D(double step);
        void CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_);

};

#endif

