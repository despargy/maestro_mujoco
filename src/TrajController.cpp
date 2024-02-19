#include <TrajController.h>
/* Constructor*/

TrajController::TrajController()
{
    std::cout<<"Constractor TrajController"<<std::endl;

    this->traj = new Trajectory();
    
    this->RcRdTwd = Eigen::Vector3d::Zero();
    this->fcontrol1.resize(6);
    this->fcontrol2.resize(6);
    this->fcontrol3.resize(6);

    std::cout<<"Constractor controller"<<std::endl;
    this->robot = new Robot();

    this->e_v.resize(6);
    this->Gbc = Eigen::MatrixXd::Identity(6,6);
    
}
TrajController::~TrajController()
{
    delete this->traj;
}

void TrajController::initTracking(double time_now)
{
    // t0 -> start of tracking
    t0 = time_now;
    t_real = 0.0; //time_now - t0;
    tv = 0.0;
    d_tv = 1.0;
    t_to_use = 0.0;

    // init Trajectory p_d0 -> robot's p0, rotation matrix
    traj->p_d0 = robot->p_c ;
    traj->R_d_0 = robot->R_c;
    traj->Q_0 = Eigen::Quaterniond(traj->R_d_0);

    // Tracking
    traj->p_d = traj->get_pDesiredTrajectory(t_to_use);
    traj->R_d = traj->get_RDesiredRotationMatrix(t_to_use);

}
void TrajController::Tracking(double time_now)
{
    t_real = time_now - t0; //update time
    // printf("Tracking t0 = %f", t0);
    // printf("Tracking t_real = %f \n", t_real);

    t_to_use = t_real;

    // if check slip detection 
    if (ADAPT_A)
    {
        computeWeights();
    }
    if (ADAPT_B)
    {
        d_tv = robot->leg[0]->w0 / std::fmin( std::fmin( robot->leg[0]->wv_leg(0), robot->leg[1]->wv_leg(0)) , std::fmin( robot->leg[2]->wv_leg(0),robot->leg[3]->wv_leg(0)  ));
        tv += d_tv*dt; // compute the virtual time
        printf("Tracking tv = %f \n", tv);
        t_to_use = tv; // use the virtual time
    }

    // desired Trajectory 
    traj->nextDesiredTraj(dt, t_to_use);
    
    // Position Error
    positionError(traj->p_d, traj->R_d);
    // Velocity Error
    velocityError(traj->dp_d, traj->R_d, traj->w_d);
    // updates Coriolis/Inertia Matrix etc.
    updateControlLaw();
    // Compute torque commands
    fComputations(traj->dp_d, traj->ddp_d, traj->R_d, traj->w_d );

}
void TrajController::positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d)
{
    // compute position ERROR
    e_p = robot->p_c - p_d;
    // compute orientation ERROR
    Re = robot->R_c*R_d.transpose();
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();
}

void TrajController::velocityError(Eigen::Vector3d dp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d)
{
    e_v.block(0,0,3,1) = robot->dCoM_p - dp_d;
    e_v.block(3,0,3,1) = robot->w_CoM - robot->R_c*R_d.transpose()*w_d ;
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ;
}
void TrajController::updateControlLaw()
{
    // compute sudo Gq
    computeSudoGq();
    // update Coriolis and Inertia
    robot->I_c = robot->R_c*robot->I*robot->R_c.transpose();
    robot->H_c.block(3,3,3,3) =  robot->I_c ; 
    robot->C_c.block(3,3,3,3) = scewSymmetric(robot->I_c*robot->w_CoM);
}
void TrajController::computeSudoGq()
{
    // compute Gq eq. 2
    // top Identities remain the same
    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->Gq.block(3,l*3,3,3) =  scewSymmetric(robot->R_c*robot->leg[l]->p_i); //eq. 2 //SCEW
    }
    // compute Gp_sude eq. 7
    robot->Gq_sudo = robot->W_inv * robot->Gq.transpose()*(robot->Gq*robot->W_inv*robot->Gq.transpose()).inverse() ;
}
void TrajController::computeWeights()
{
    for(int l = 0; l < robot->n_legs ; l++)
    {
        // adapt weights
        robot->leg[l]->wv_leg(1) = alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(1) ; // y
        robot->leg[l]->wv_leg(0) = alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(0); // x
            
        // update vvvv vector of robot                                                                  // z stays 1.0 do not change
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   
    }
    // save as matrix the inverse of diagonal vvvv vector
    robot->W_inv = (robot->vvvv.asDiagonal()).inverse();

    std::cout << "vvvv" <<std::endl;
    std::cout << robot->vvvv.transpose() <<std::endl;
}
void TrajController::fComputations(Eigen::Vector3d dp_d, Eigen::Vector3d ddp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d )
{
    // first term of Fc eq. 11
    fcontrol1.block(0,0,3,1) = ddp_d;
    fcontrol1.block(3,0,3,1) = deriv_RcRdTwd( RcRdTwd, robot->R_c*R_d.transpose()*w_d, this->dt); 
    RcRdTwd = robot->R_c*R_d.transpose()*w_d;
    // second term of Fc eq. 11
    fcontrol2.block(0,0,3,1) = dp_d;
    fcontrol2.block(3,0,3,1) = RcRdTwd;
    // third term of Fc eq. 11
    fcontrol3.block(0,0,3,1) = -kp*e_p;
    fcontrol3.block(3,0,3,1) = -ko*e_o; 

    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);

    // Final Fc ep. 11
    robot->F_c = robot->H_c*fcontrol1 + robot->C_c*fcontrol2  + fcontrol3 - this->kv*this->e_v + Gbc*robot->gc ;
    // solve eq. 1 with respect to Fa
    robot->F_a = robot->Gq_sudo*robot->F_c ;

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->f_cmd = -robot->F_a.block(l*3,0,3,1); // slip Fa eq. 3
        robot->leg[l]->tau =  (robot->R_c*(robot->leg[l]->J.block<3,3>(0,0))).transpose()*robot->leg[l]->f_cmd; // compute eq. 4
    }
}
void TrajController::PD(double* target)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        robot->leg[l]->tau(0) = -100*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
        robot->leg[l]->tau(1) = -100*(robot->leg[l]->q(1) - target[1])-10*robot->leg[l]->dq(1);
        robot->leg[l]->tau(2) = -100*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    }
}
void TrajController::PD_smooth(double* target, double smooth)
{   
    double percent = (double)loop_index/smooth;
    for(int l=0; l< robot->n_legs; l++)
    {
        robot->leg[l]->tau(0) = -100*percent*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
        robot->leg[l]->tau(1) = -100*percent*(robot->leg[l]->q(1) - target[1]) -10*robot->leg[l]->dq(1);
        robot->leg[l]->tau(2) = -100*percent*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    }
    loop_index += 1;
}