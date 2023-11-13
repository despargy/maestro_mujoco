#include <Controller.h>


#ifndef _TRAJCONTROLLER_H_
#define _TRAJCONTROLLER_H_


class TrajController : public Controller
{
    public:
        
        Trajectory* traj; // Trajectory pointer obj.

        Eigen::Vector3d RcRdTwd;
        Eigen::VectorXd fcontrol1,fcontrol2,fcontrol3; // vector to help with eq. 11

        TrajController();
        ~TrajController();
 
        // /* Tracking */
        void initTracking(double time_now);
        void Tracking(double time_now);

        void positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d); 
        void velocityError(Eigen::Vector3d dp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d);
        void updateControlLaw();
        void computeSudoGq();
        void computeWeights();
        void fComputations(Eigen::Vector3d dp_d, Eigen::Vector3d ddp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d );


};

#endif