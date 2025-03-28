#include <Controller.h>

#ifndef _LOCOMOTIONCONTROLLER_H_
#define _LOCOMOTIONCONTROLLER_H_


class LocomotionController : public Controller
{
    public:

        double t0_phase, t_phase ; // time variables for adaptive tracking

        double A, b, t0_superG;
        double freq_swing, t0_swing, t_half_swing, swing_t_slot, tA, tB, t_down;
        double w_max;
        bool  CHANGE_PHASE, A_PD, B_PD ;
        int ii;
        int* vp_order;
        int* static_free_gait;
        double kr;
        double step_bez;
        bool A_TOUCHED, B_TOUCHED;
        bool INCLINATION;
        double c1, force_thres, k_clik;
        double tau_lim, terrain_height ;
        std::vector<double> bCurveX, dot_bCurveX; // Bezier Curve swinging tip
        std::vector<double> bCurveY, dot_bCurveY; // Bezier Curve swinging tip
        std::vector<double> bCurveZ, dot_bCurveZ; // Bezier Curve swinging tip

        Eigen::Vector3d f_applied, f_applied_a, f_applied_b, f_stance_a, f_stance_b;
        Eigen::Vector3d request_tau;
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
        double dz_offset, incl_a;
        double dist_error, c1tip, c2tip, ampli_A, ampli_B, tip_target_z, percentage;
        
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
        void dynamicBezier(Leg* l, Eigen::Vector3d dp_cmd);
        void dynamicBezier(Leg* l, Eigen::Vector3d dp_cmd, double dz_incl);

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
        void computeDynamicWeights(double incl_a);
        void initWeights();
        void freezedoubleCLIK(int i);
        void checkTouchDown();
        void change_Rd(Eigen::Vector3d euler_angle);
        void updateInclination(Eigen::Vector3d dp_cmd);

};

#endif

