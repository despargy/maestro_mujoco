#include <Controller.h>
/* Constructor*/
Controller::Controller()
{
    this->robot = new Robot();
    this->traj = new Trajectory();

    this->e_v.resize(6);
    this->RcRdTwd = Eigen::Vector3d::Zero();
    this->fcontrol1.resize(6);
    this->fcontrol2.resize(6);
    this->fcontrol3.resize(6);

    this->pid_out.resize(6);

    this->e_p_int = Eigen::Vector3d::Zero();
    this->e_o_int = Eigen::Vector3d::Zero();

    this->Gbc = Eigen::MatrixXd::Identity(6,6);

    vp_order = new int[robot->n_legs];
    vp_order[0] = 0; vp_order[1] = 2; vp_order[2] = 3; vp_order[3] = 1; // define gait order

    swing_leg = 0;  // start with FR swing TODO leave only fsm phase init
}
/* De-Constructor*/
Controller::~Controller()
{
    delete this->robot;
    delete this->traj;
}
void Controller::PD(double* target)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        robot->leg[l]->tau(0) = -100*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
        robot->leg[l]->tau(1) = -100*(robot->leg[l]->q(1) - target[1])-10*robot->leg[l]->dq(1);
        robot->leg[l]->tau(2) = -100*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    }
}
void Controller::PD_smooth(double* target, double smooth)
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
void Controller::initTracking(double time_now)
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
void Controller::Tracking(double time_now)
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
void Controller::positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d)
{
    // compute position ERROR
    e_p = robot->p_c - p_d;
    // compute orientation ERROR
    Re = robot->R_c*R_d.transpose();
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();
}
void Controller::velocityError(Eigen::Vector3d dp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d)
{
    e_v.block(0,0,3,1) = robot->dCoM_p - dp_d;
    e_v.block(3,0,3,1) = robot->w_CoM - robot->R_c*R_d.transpose()*w_d ;
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ;
}
void Controller::updateControlLaw()
{
    // compute sudo Gq
    computeSudoGq();
    // update Coriolis and Inertia
    robot->I_c = robot->R_c*robot->I*robot->R_c.transpose();
    robot->H_c.block(3,3,3,3) =  robot->I_c ; 
    robot->C_c.block(3,3,3,3) = scewSymmetric(robot->I_c*robot->w_CoM);
}
void Controller::computeSudoGq()
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
void Controller::computeWeights()
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
void Controller::fComputations(Eigen::Vector3d dp_d, Eigen::Vector3d ddp_d, Eigen::Matrix3d R_d, Eigen::Vector3d w_d )
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

void Controller::initTarget(double time_now)
{
    // t0 -> start of tracking
    t0 = time_now;
    t_real = 0.0; //time_now - t0;
    t0_phase = time_now; // when that phase started

    // // compute centroid of support polygon 4 legs
    // std::vector<std::pair<double, double> > vp = 
    // {
    //     // {robot->leg[0]->p_i(0),robot->leg[0]->p_i(1)},
    //     {robot->leg[2]->p_i(0),robot->leg[2]->p_i(1)},
    //     {robot->leg[3]->p_i(0),robot->leg[3]->p_i(1)},
    //     {robot->leg[1]->p_i(0),robot->leg[1]->p_i(1)}

    // } ;
    std::vector<std::pair<double, double> > vp;
    for(int l=0; l<robot->n_legs; l++)
    {
        vp.push_back({robot->leg[vp_order[l]]->p_i(0),robot->leg[vp_order[l]]->p_i(1)});
    }


    std::pair<double, double> C = find_Centroid(vp);
    Eigen::Vector3d target(C.first,C.second,robot->p_c(2));
    std::cout<< "target "<<target<<std::endl;
    // set target goal position target, starting position, rotation target
    traj->updateTarget(target, robot->p_c, robot->R_c); 

}
void Controller::initTarget(double time_now, Eigen::Vector3d target_)
{
    // t0 -> start of tracking
    t0 = time_now;
    t_real = 0.0; //time_now - t0;
    t0_phase = time_now; // when that phase started

    // set target goal
    traj->updateTarget(target_, robot->p_c, robot->R_c);
}
void Controller::setPhaseTarget(int phase, double time_now)
{

    t0_phase = time_now; // when that phase started     TODO 0.0 or now
    std::vector<std::pair<double, double> > vp;
    for(int l = 0; l<robot->n_legs; l++)
    {
        if((vp_order[l] != swing_leg) )
        {
            vp.push_back({robot->leg[vp_order[l]]->p_i(0),robot->leg[vp_order[l]]->p_i(1)});
        }
       
    }
    std::pair<double, double> C = find_Centroid(vp);
    Eigen::Vector3d target(C.first,C.second,robot->p_c(2));

    // set target goal position target, starting position, rotation target
    traj->updateTarget(target, robot->p_c, robot->R_c); 

}
void Controller::ReachTarget(double time_now)
{
    t_real = time_now - t0; //update time
    t_to_use = t_real;
    // printf("Target time_now - t0 = %f \n",time_now - t0);
    
    // Position Error
    positionError(traj->p_T, traj->R_T);
    // Velocity Error for Target Reaching
    velocityError();
    // compute Gq_sudo
    computeSudoGq();
    // PID controll with saturation
    PIDwithSat();
    // Compute torque commands
    fComputations();
}
void Controller::velocityError() // Target Control
{
    e_v.block(0,0,3,1) = robot->dCoM_p ;
    e_v.block(3,0,3,1) = robot->w_CoM ; // TODO this is zero ?
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ; 
}
void Controller::fComputations() // Target Control
{

    // Final Fc ep. 11 -> Target Control
    robot->F_c = pid_out ;

    // solve eq. 1 with respect to Fa 
    robot->F_a = robot->Gq_sudo*robot->F_c ;

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->f_cmd = -robot->F_a.block(l*3,0,3,1); // slip Fa eq. 3
        robot->leg[l]->tau =  (robot->R_c*(robot->leg[l]->J.block<3,3>(0,0))).transpose()*robot->leg[l]->f_cmd; // compute eq. 4
    }
}

void Controller::PIDwithSat()
{
    // third term of Fc eq. 11
    pid_out.block(0,0,3,1) = -kp*e_p;
    pid_out.block(3,0,3,1) = -ko*e_o; 
    pid_out -=  this->kv*this->e_v ;

    // Add Integral -> Target Control
    for (int axis = 0; axis<3; axis++)
    {
        //position
        if(fabs(e_p(axis))<0.01)
        {
            e_p_int(axis) += e_p(axis)*dt;
        }
        //orientation
        if(fabs(e_o(axis))<0.2)
        {
            e_o_int(axis) += e_o(axis)*dt;
        }
    }

    pid_out.block(0,0,3,1) += -ki*e_p_int.block(0,0,3,1) ;
    pid_out.block(3,0,3,1) += -0.3*ki*e_o_int ;

    if(pid_out.block(0,0,3,1).norm()>20.0){
        pid_out.block(0,0,3,1) = pid_out.block(0,0,3,1) * 20.0 / pid_out.block(0,0,3,1).norm();
    }
    if(pid_out.block(3,0,3,1).norm()>20.0){
        pid_out.block(3,0,3,1) = pid_out.block(3,0,3,1) * 20.0 / pid_out.block(3,0,3,1).norm();
    }
    // std::cout<< "e_p_int "<<e_p_int.transpose() << std::endl;
    // std::cout<< "e_o_int "<<e_o_int.transpose() << std::endl;
    // std::cout<< "pid_out "<<pid_out.transpose() << std::endl;

    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);
    pid_out += Gbc*robot->gc;
    
}
void Controller::LocomotionSwing()
{

}
void Controller::LocomotionTarget(double time_now)
{
    t_real = time_now - t0; //update time
    t_to_use = t_real;
    // printf("Target time_now - t0 = %f \n",time_now - t0);
    
    // Position Error
    positionError(traj->p_T, traj->R_T);
    // Velocity Error for Target Reaching
    velocityError();
    
}