#include <Controller.h>


#ifndef _LOCOMOTIONCONTROLLER_H_
#define _LOCOMOTIONCONTROLLER_H_


class LocomotionController : public Controller
{
    public:

        double t0_phase, t_phase ; // time variables for adaptive tracking

        double A, b, t0_superG;
        double freq_swing, t0_swing, t_half_swing, swing_t_slot;
        
        int swing_leg;
        int* vp_order;
        int* static_free_gait;

        // vector for target position, inside locomotion mode
        Eigen::Vector3d p_T;
        // Eigen::Vector3d p0_ofphase; 
        Eigen::Matrix3d R_T;

        // // Error variables
        Eigen::Vector3d e_p_int, e_o_int;
        Eigen::VectorXd  pid_out; 


        LocomotionController();
        ~LocomotionController();


        /* Tracking */

        // void positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d); 
        // void velocityError(Eigen::Vector3d dp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d);
        
        
        // void updateControlLaw();
        // void computeSudoGq();
        // void computeWeights();
        // void fComputations(Eigen::Vector3d dp_d, Eigen::Vector3d ddp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d );

        /* Target */
        // void initTarget(double top_time); // target is the centroid of polygon
        // void initTarget(double top_time, Eigen::Vector3d target_); // overload with target as input
        // void ReachTarget(double top_time);
        // void velocityError();
        // void fComputations();
        // void PIDwithSat();
        void setPhaseTarget();
            
        // /* Locomotion */
        // void LocomotionTarget(double time_now);
        // void LocomotionSwing();

};

#endif

