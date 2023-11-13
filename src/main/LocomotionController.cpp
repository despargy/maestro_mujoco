#include <LocomotionController.h>
/* Constructor*/

LocomotionController::LocomotionController()
{

    std::cout<<"Constractor controller"<<std::endl;
    this->robot = new Robot();

    this->e_v.resize(6);
    this->Gbc = Eigen::MatrixXd::Identity(6,6);
    
    this->pid_out.resize(6);

    this->e_p_int = Eigen::Vector3d::Zero();
    this->e_o_int = Eigen::Vector3d::Zero();

    vp_order = new int[robot->n_legs];
    vp_order[0] = 0; vp_order[1] = 2; vp_order[2] = 3; vp_order[3] = 1; // define gait order

    swing_leg = 0;  // start with FR swing TODO leave only fsm phase init
}
LocomotionController::~LocomotionController(){}
void LocomotionController::initTarget(double time_now)
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
    this->p_T = target;
    this->p0_ofphase = robot->p_c;
    this->R_T = robot->R_c ;

}
void LocomotionController::initTarget(double time_now, Eigen::Vector3d target_)
{
    // t0 -> start of tracking
    t0 = time_now;
    t_real = 0.0; //time_now - t0;
    t0_phase = time_now; // when that phase started

    // set target goal
    this->p_T = target_;
    this->p0_ofphase = robot->p_c;
    this->R_T = robot->R_c ;}
void LocomotionController::setPhaseTarget(int phase, double time_now)
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
    p_T = robot->p_c;
    R_T = robot->R_c0;

    this->p_T = target;
    this->p0_ofphase = robot->p_c;
    this->R_T = robot->R_c ;
}
void LocomotionController::ReachTarget(double time_now)
{
    t_real = time_now - t0; //update time
    t_to_use = t_real;
    // printf("Target time_now - t0 = %f \n",time_now - t0);
    
    // Position Error
    positionError(p_T, R_T);
    // Velocity Error for Target Reaching
    velocityError();
    // compute Gq_sudo
    computeSudoGq();
    // PID controll with saturation
    PIDwithSat();
    // Compute torque commands
    fComputations();
}
void LocomotionController::velocityError() // Target Control
{
    e_v.block(0,0,3,1) = robot->dCoM_p ;
    e_v.block(3,0,3,1) = robot->w_CoM ; // TODO this is zero ?
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ; 
}
void LocomotionController::fComputations() // Target Control
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

void LocomotionController::PIDwithSat()
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
void LocomotionController::LocomotionSwing()
{

}
void LocomotionController::LocomotionTarget(double time_now)
{
    t_real = time_now - t0; //update time
    t_to_use = t_real;
    // printf("Target time_now - t0 = %f \n",time_now - t0);
    
    // Position Error
    positionError(p_T, R_T);
    // Velocity Error for Target Reaching
    velocityError();
    
}
void LocomotionController::positionError(Eigen::Vector3d p_d, Eigen::Matrix3d R_d)
{
    // compute position ERROR
    e_p = robot->p_c - p_d;
    // compute orientation ERROR
    Re = robot->R_c*R_d.transpose();
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();
}