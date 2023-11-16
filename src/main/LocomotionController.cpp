#include <LocomotionController.h>
/* Constructor*/

LocomotionController::LocomotionController()
{

    std::cout<<"Constractor LocomotionController"<<std::endl;
    this->robot = new Robot();

    this->e_v.resize(6);
    this->Gbc = Eigen::MatrixXd::Identity(6,6);
    
    this->pid_out.resize(6);

    this->e_p_int = Eigen::Vector3d::Zero();
    this->e_o_int = Eigen::Vector3d::Zero();

    vp_order = new int[robot->n_legs];
    vp_order[0] = 0; vp_order[1] = 2; vp_order[2] = 3; vp_order[3] = 1; // define support polygon order


    static_free_gait = new int[robot->n_legs];
    static_free_gait[0] = 0; static_free_gait[1] = 3; static_free_gait[2] = 1; static_free_gait[3] = 2; // define gait order

}
LocomotionController::~LocomotionController(){}
void LocomotionController::setPhaseTarget()
{
    robot->leg[(int) robot->swingL_id]->initQout(); // set all q_out as zeros

    this->t_phase = 0.0;
    this->t0_phase = t_real; // when that phase started relatve to real time
    this->ii = -1; // counter for bezier execution

    std::vector<std::pair<double, double> > vp;
    for(int l = 0; l<robot->n_legs; l++)
    {
        if((vp_order[l] != robot->swingL_id) )
        {
            vp.push_back({robot->leg[vp_order[l]]->p_i(0),robot->leg[vp_order[l]]->p_i(1)});
        }
        // std::cout<<l<<": "<<robot->leg[l]->p_i(0)<<" \t"<<robot->leg[l]->p_i(1)<<std::endl;

    }
    std::pair<double, double> C = find_Centroid(vp);
    Eigen::Vector3d target(C.first,C.second,robot->z);

    this->p_T =  target;
    // std::cout<<"pc0"<< robot->p_c0<<std::endl;
    // std::cout<<"pc"<< robot->p_c<<std::endl;
    // std::cout<<"target"<< target<<std::endl;

    this->R_T = robot->R_c0 ;
    // std::cout<<"target RT"<< this->R_T<<std::endl;
    // std::cout<<"Rc"<< robot->R_c<<std::endl;

    e_p_int = Eigen::Vector3d::Zero();
    e_o_int = Eigen::Vector3d::Zero();
    pid_out = Eigen::VectorXd::Zero(6);
    
    robot->leg[(int)robot->swingL_id]->storeInitG(); 
}
void LocomotionController::computeWeightsSwing()
{

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->wv_leg(1) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(1) ; // y
        robot->leg[l]->wv_leg(0) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(0); // x

        // update vvvv vector of robot                          // z stays 1.0 do not change
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   
    }
    /* Added to simuate swing leg weights t inf */ //TODO
    robot->leg[(int) robot->swingL_id]->wv_leg = robot->leg[(int) robot->swingL_id]->w0*Eigen::Vector3d::Ones();
    // robot->leg[(int) robot->swingL_id]->wv_leg = robot->leg[(int) robot->swingL_id]->w0*Eigen::Vector3d::Ones() + w_max*superGaussian(A,b,t_half_swing-t0_superG,t_phase - t_half_swing, 13)*Eigen::Vector3d::Ones(); 
    robot->vvvv.block((int) robot->swingL_id*3,0,3,1) = robot->leg[(int) robot->swingL_id]->wv_leg;   // update vvvv vector of robot 
    
    robot->W_inv = (robot->vvvv.asDiagonal()).inverse(); // save as matrix the inverse of diagonal vvvv vector

}
void LocomotionController::errors()
{
    
    e_p = robot->p_c - p_T; // compute position error
    Re = robot->R_c*R_T.transpose(); // compute orientation error
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();

    e_v.block(0,0,3,1) = robot->dCoM_p ; // compute velocity error
    e_v.block(3,0,3,1) = robot->w_CoM ; 
    e_v.block(3,0,3,1) = 1.0*this->e_v.block(3,0,3,1) ; 

    
    // std::cout<<"e_p"<<std::endl;
    // std::cout<<e_p<<std::endl;
    // std::cout<<"e_o"<<std::endl;
    // std::cout<<e_o<<std::endl;
    // std::cout<<"e_v"<<std::endl;
    // std::cout<<e_v<<std::endl;
}
void LocomotionController::computeSudoGq()
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
void LocomotionController::fComputations() // Target Control
{
    robot->F_c = pid_out ; // Final Fc ep. 11 -> Target Control
    robot->F_a = robot->Gq_sudo*robot->F_c ;     // solve eq. 1 with respect to Fa

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->f_cmd = -robot->F_a.block(l*3,0,3,1); // slip Fa eq. 3
        robot->leg[l]->f_cmd(2) =  std::fmin(robot->leg[l]->f_cmd(2), 0.0);
        robot->leg[l]->tau =  (robot->R_c*(robot->leg[l]->J.block<3,3>(0,0))).transpose()*robot->leg[l]->f_cmd; // compute eq. 4
    }
}

void LocomotionController::PIDwithSat()
{
    pid_out.block(0,0,3,1) = -kp*e_p; // third term of Fc eq. 11
    pid_out.block(3,0,3,1) = -ko*e_o; 
    pid_out -=  this->kv*this->e_v ;

    /*  Integral -> Target Control */ 
    for (int axis = 0; axis<3; axis++)
    {
        if(fabs(e_p(axis))<0.01)
        {
            e_p_int(axis) += e_p(axis)*dt; //position
        }
        if(fabs(e_o(axis))<0.2)
        {
            e_o_int(axis) += e_o(axis)*dt; //orientation
        }
    }

    pid_out.block(0,0,3,1) += -ki*e_p_int.block(0,0,3,1) ;
    pid_out.block(3,0,3,1) += -0.3*ki*e_o_int ;

    if(pid_out.block(0,0,3,1).norm()>80.0){
        pid_out.block(0,0,3,1) = pid_out.block(0,0,3,1) * 80.0 / pid_out.block(0,0,3,1).norm();
    }
    if(pid_out.block(3,0,3,1).norm()>80.0){
        pid_out.block(3,0,3,1) = pid_out.block(3,0,3,1) * 80.0 / pid_out.block(3,0,3,1).norm();
    }

    // // std::cout<< "e_p_int "<<e_p_int.transpose() << std::endl;
    // // std::cout<< "e_o_int "<<e_o_int.transpose() << std::endl;
    // // std::cout<< "pid_out "<<pid_out.transpose() << std::endl;

    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);
    pid_out += Gbc*robot->gc;
    
}
void LocomotionController::inverseTip()
{
    // /**************** Bezier curve ***************/ 
    // if(t_phase<t0_swing)
    // {
    //     d_tip_pos<< 0.0, 0.0, 0.0, 1.0; 
    //     d_tip_vel<< 0.0, 0.0, 0.0;
    // }
    // else if( t_phase>=t0_swing & t_phase<=(t0_swing + 1/freq_swing) ) 
    // {
    //     ++ii;
    //     d_tip_pos << bCurveX[ii] , robot->leg[(int)robot->swingL_id]->y_tip,  bCurveZ[ii] , 1.0;
    //     d_tip_vel << dot_bCurveX[ii], 0.0,  dot_bCurveZ[ii];
    // }
    // else 
    // {
    //     d_tip_vel<< 0.0, 0.0, 0.0;
    // }  
    // /**************** Bezier curve ***************/ 
    // d_traj_tipframe(0) = d_tip_pos(0);
    // d_traj_tipframe(1) = d_tip_pos(1);
    // d_traj_tipframe(2) = d_tip_pos(2);
    // Eigen::Matrix4f BC_T = Eigen::Matrix4f::Zero(); // let B be system B at tip, then BO_T is the transformation from B to world frame {0}, with:
    // BC_T(3,3) = 1.0;
    // BC_T.block(0,0,3,3) = Eigen::Matrix3f::Identity() ;// robot_->g_com_init.block(0,0,3,3).cast<float>();// ->g_com_init.block(0,0,3,3).inverse().cast<float>(); // B from C is inv. the robot R_c orientation as the robot frame {0}
    // BC_T.block(0,3,3,1) = robot->leg[(int)robot->swingL_id]->g_0bo_init.block(0,3,3,1).cast<float>();  // translation as tip init pose, from {0}
    
    // // desired swinging-tip trajectory represented from {0} is:
    // Eigen::Vector4f d_CoM_pos =  BC_T*d_tip_pos;
    // if(robot_->swingL_id == 0)
    //     d_traj_0frame = d_CoM_pos.block(0,0,3,1); // cut last 1
    // CLIK(d_CoM_pos.block(0,0,3,1), d_tip_vel);
    
}
void LocomotionController::CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_)
{
    // Eigen::Vector3f d_q_ = (   robot->R_c* robot->leg[(int)robot->swingL_id]->J.block<3,3>(0,0)).inverse().cast<float>()*(dpd_0frame_ - 16*(robot->leg[(int)robot->swingL_id]->g_o_world.block(0,3,3,1).cast<float>() - pd_0frame_) );
    // // std::cout<<"leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>():"<< leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>().transpose()<<std::endl;
    // // std::cout<<"pd_0frame_:"<< pd_0frame_.transpose()<<std::endl;
    // // std::cout<<"p--------------------------------------------"<< std::endl;
    // leg_mng[(int)robot_->swingL_id].q_out(0) = d_q_(0)*dt + leg_mng[(int)robot_->swingL_id].q_out(0);
    // leg_mng[(int)robot_->swingL_id].q_out(1) = d_q_(1)*dt + leg_mng[(int)robot_->swingL_id].q_out(1);
    // leg_mng[(int)robot_->swingL_id].q_out(2) = d_q_(2)*dt + leg_mng[(int)robot_->swingL_id].q_out(2);
    // leg_mng[(int)robot_->swingL_id].dq_out(0) = d_q_(0);
    // leg_mng[(int)robot_->swingL_id].dq_out(1) = d_q_(1);
    // leg_mng[(int)robot_->swingL_id].dq_out(2) = d_q_(2);
}
void LocomotionController::computeBesierCurve2D(double step) // static gait pre-fixed swinging trajectory
{
    std::vector<double> xX{0.0, 0.08, 0.09, 0.07};
    std::vector<double> zZ{0.0, 0.1, 0.05, 0.0};
    double bCurveXt, dot_bCurveXt;
    double bCurveZt, dot_bCurveZt;
    for (double t = 0.0; t <= 1; t += step)
    {
        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3];
        bCurveZt = std::pow((1 - t), 3) * zZ[0] + 3 * std::pow((1 - t), 2) * t * zZ[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * zZ[2] + std::pow(t, 3) * zZ[3];
        bCurveX.push_back(bCurveXt);
        bCurveZ.push_back(bCurveZt);
        dot_bCurveXt = 3 * std::pow((1 - t), 2) *(xX[1] - xX[0])  + 6 * (1 - t) * t * (xX[2] - xX[1]) + 3 * std::pow(t, 2) * ( xX[3] - xX[2] );
        dot_bCurveZt = 3 * std::pow((1 - t), 2) *(zZ[1] - zZ[0])  + 6 * (1 - t) * t * (zZ[2] - zZ[1]) + 3 * std::pow(t, 2) * ( zZ[3] - zZ[2] );
        dot_bCurveX.push_back(dot_bCurveXt);
        dot_bCurveZ.push_back(dot_bCurveZt);
    }
}
void LocomotionController::PD(double* target)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        robot->leg[l]->tau(0) = -100*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
        robot->leg[l]->tau(1) = -100*(robot->leg[l]->q(1) - target[1])-10*robot->leg[l]->dq(1);
        robot->leg[l]->tau(2) = -100*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    }
}
void LocomotionController::PD_smooth(double* target, double smooth)
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