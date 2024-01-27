#include <Controller.h>


#ifndef _LOCOMOTIONCONTROLLER_H_
#define _LOCOMOTIONCONTROLLER_H_


class LocomotionController : public Controller
{
    public:

        double t0_phase, t_phase ; // time variables for adaptive tracking

        double A, b, t0_superG;
        double freq_swing, t0_swing, t_half_swing, swing_t_slot, tA, tB;
        double w_max;
        bool  CHANGE_PHASE, A_PD, B_PD ; ;
        int ii;
        int* vp_order;
        int* static_free_gait;
        double kr;
        double ofset, step_bez;
        bool A_TOUCHED, B_TOUCHED;
        double c1, c2, force_thres;

        std::vector<double> bCurveX, dot_bCurveX; // Bezier Curve swinging tip
        std::vector<double> bCurveY, dot_bCurveY; // Bezier Curve swinging tip
        std::vector<double> bCurveZ, dot_bCurveZ; // Bezier Curve swinging tip

        Eigen::Vector3d f_applied, f_applied_a, f_applied_b;

        // vector for target position, inside locomotion mode
        Eigen::Vector3d p_T;
        // Eigen::Vector3d p0_ofphase; 
        Eigen::Matrix3d R_T;

        // // Error variables
        Eigen::Vector3d e_p_int, e_o_int;
        Eigen::VectorXd  pid_out; 

        Eigen::Vector4f d_tip_pos, d_world_pos;
        Eigen::Vector3f d_tip_vel;
        
        Eigen::Vector3f bezier_world, bezier_world_a, bezier_world_b;
        
        double dist_error;
        
        LocomotionController();
        ~LocomotionController();


        void PD(double* target);
        void PD_smooth(double* target, double smooth);
        
        void setPhaseTarget();
        void computeWeightsSwing();
        void errors();
        void CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_);
        void computeSudoGq();   
        void PIDwithSat();
        void fComputations();
        void dynamicBezier(Leg* l);
        void doubleInverseTip();

        void inverseTip();
        void signalFc();
        void computeBesierCurve2D(double step);
        void spiralExploration();
        void computeWeightsSigmoid();
        void generateBezier(double step);
        void swingTrajectory();
        void setDynamicPhase();
        void doubleCLIK(Eigen::Vector3f pd_0frame_A, Eigen::Vector3f dpd_0frame_A, int i);
        void dynaErrors(Eigen::Vector3d dp_cmd);
        void dynaControlSignal();
        void computeDynamicWeights();
        void initWeights();
        void freezedoubleCLIK(int i);
        void checkTouchDown();

};

#endif

