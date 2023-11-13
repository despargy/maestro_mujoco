#include <Trajectory.h>

Trajectory::Trajectory()
{
    this->p_d = Eigen::Vector3d::Zero(); // will be set as trajectory 0
    this->dp_d = Eigen::Vector3d::Zero();
    this->ddp_d = Eigen::Vector3d::Zero();
}
Trajectory::~Trajectory(){}
/* CoM Periodic Trajectory */
void Trajectory::nextDesiredTraj(double dt, double t_to_use)
{
    // get next DESIRED position
    ddp_d = get_ddpDesiredTrajectory(p_d, dp_d, dt, t_to_use);
    dp_d  = get_dpDesiredTrajectory(p_d, dt, t_to_use);
    p_d   = get_pDesiredTrajectory(t_to_use);
    // get next DESIRED orientation
    dR_d = get_dRDesiredRotationMatrix(R_d, dt, t_to_use);
    R_d = get_RDesiredRotationMatrix(t_to_use);
    // DESIRED angular velocity of Com
    w_d = scewSymmetricInverse(dR_d*R_d.transpose());
}
Eigen::Vector3d Trajectory::get_pDesiredTrajectory(double t_real)
{
    // x,y axis
    double freq = 0.25; //1.0/(2*M_PI); // sim 0.65
    
    // x, z axis 
    p_d(0) = p_d0(0) - 0.03*sin(2*M_PI*freq*t_real); 
    p_d(1) = p_d0(1) ;//+ 0.0; 
    p_d(2) = p_d0(2)  -(0.02*( 1 - cos(2*M_PI*freq*t_real) ) );

    // // x, z axis 
    // p_d(0) = p_d0(0) - 0.03*sin(2*M_PI*freq*t_real); 
    // p_d(1) = p_d0(1) -(0.02*( 1 - cos(2*M_PI*freq*t_real) ) );//+ 0.0; 
    // p_d(2) = p_d0(2)  ;

    return p_d;
}
Eigen::Vector3d Trajectory::get_dpDesiredTrajectory(Eigen::Vector3d p_d_prev, double dt, double t_real)
{
    return ( get_pDesiredTrajectory(t_real) - p_d_prev)/dt;
}  
Eigen::Vector3d Trajectory::get_ddpDesiredTrajectory(Eigen::Vector3d p_d_prev,Eigen::Vector3d dp_d_prev, double dt, double t_real)
{
    return ( get_dpDesiredTrajectory(p_d_prev, dt, t_real) - dp_d_prev)/dt;
}    
// Orientation
Eigen::Matrix3d Trajectory::get_RDesiredRotationMatrix(double t_real)
{
    Eigen::Quaterniond temp = Q_0;
    temp.x() = Q_0.x();// + 0.2*sin(2*0.2*M_PI*t_real); // 
    temp.normalize();
    return temp.toRotationMatrix(); 
}
Eigen::Matrix3d Trajectory::get_dRDesiredRotationMatrix(Eigen::Matrix3d R_prev,double dt, double t_real)
{
    return ( get_RDesiredRotationMatrix(t_real) - R_prev) / dt;
}


/* CoM Reach Target */
void Trajectory::updateTarget(Eigen::Vector3d p_T_, Eigen::Vector3d p0_ofphase_, Eigen::Matrix3d R_T_)
{
    this->p_T = p_T_;
    this->p0_ofphase = p0_ofphase_;
    this->R_T = R_T_ ;
}
// void Trajectory::nextDTarget(Eigen::Vector3d p0_ofphase, double t_ofphase, double dt)
// {
//     // get next until Target position
//     ddp_d = get_ddpTarget(p0_ofphase, p_d, dp_d, dt, t_ofphase);
//     dp_d  = get_dpTarget(p0_ofphase, p_d, dt, t_ofphase);
//     p_d   = get_pTarget(p0_ofphase, t_ofphase);
//     // get next until Target orientation
//     dR_d = get_dRTargetRotationMatrix(Q0_ofphase, R_d, dt, t_ofphase);
//     R_d = get_RTargetRotationMatrix(Q0_ofphase, t_ofphase);
//     // DESIRED angular velocity of Com
//     w_d = scewSymmetricInverse(dR_d*R_d.transpose());
// }
// Eigen::Vector3d Trajectory::get_pTarget(Eigen::Vector3d p0_ofphase_, double t_ofphase)
// {
//     // reach target
//     return p0_ofphase_ - this->p_T ;
// }
// Eigen::Vector3d Trajectory::get_dpTarget(Eigen::Vector3d p0_ofphase_,Eigen::Vector3d p_d_prev, double dt, double t_ofphase)
// {
//     return (get_pTarget(p0_ofphase_, t_ofphase) - p_d_prev)/dt;
// }  
// Eigen::Vector3d Trajectory::get_ddpTarget(Eigen::Vector3d p0_ofphase_,Eigen::Vector3d p_d_prev,Eigen::Vector3d dp_d_prev, double dt, double t_ofphase)
// {
//     return ( get_dpTarget(p0_ofphase_, p_d_prev, dt, t_ofphase) - dp_d_prev)/dt;
// } 
// // Orientation
// Eigen::Matrix3d Trajectory::get_RTargetRotationMatrix(Eigen::Quaterniond Q0_ofphase, double t_ofphase)
// {
//     Eigen::Quaterniond temp = Q0_ofphase;
//     // temp.x() = Q0_ofphase.x();
//     // temp.normalize();
//     return temp.toRotationMatrix(); 
// }
// Eigen::Matrix3d Trajectory::get_dRTargetRotationMatrix(Eigen::Quaterniond Q0_ofphase, Eigen::Matrix3d R_prev,double dt, double t_ofphase)
// {
//     return (get_RTargetRotationMatrix(Q0_ofphase, t_ofphase) - R_prev)/ dt;
// }