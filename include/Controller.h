#include <Robot.h>
#include <Math.h>
#include <Trajectory.h>

#ifndef _CONTROLLER_H_
#define _CONTROLLER_H_


class Controller
{
    public:

        Robot* robot; // Robot pointer obj.

        double t_real, t0, tv , d_tv, t_to_use, dt ; // time variables for adaptive tracking
        int loop_index; // for smooth stand up
        bool ADAPT_A, ADAPT_B;
        double kp, kv, ko, ki, kw;
        double slope, alpha;

        // Error variables
        Eigen::Vector3d e_p, e_o; 
        Eigen::VectorXd e_v;
        Eigen::AngleAxisd ang;
        Eigen::Matrix3d Re;

        Eigen::MatrixXd Gbc;

        Controller();
        ~Controller();

        void PD(double* target);
        void PD_smooth(double* target, double smooth);

        /* General */
        void positionError();
        void velocityError();
        void updateControlLaw();
        void computeSudoGq();
        void computeWeights();
        void fComputations();
        
};

#endif