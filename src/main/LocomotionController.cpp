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

    A_PD = false;
    B_PD = false;
}
LocomotionController::~LocomotionController(){}
void LocomotionController::setPhaseTarget()
{
    robot->leg[(int) robot->swingL_id]->initQout(); // set all q_out as current value

    this->t_phase = 0.0;
    this->t0_phase = t_real; // when that phase started relatve to real time
    this->ii = -1; // counter for bezier execution

    std::vector<std::pair<double, double> > vp;
    for(int l = 0; l<robot->n_legs; l++)
    {
        if((vp_order[l] != robot->swingL_id) )
        {
            vp.push_back({robot->leg[vp_order[l]]->g_o_world(0,3),robot->leg[vp_order[l]]->g_o_world(1,3)});
        }
        // std::cout<<"forces "<<l<<": "<<robot->leg[l]->f(0)<<" \t"<<robot->leg[l]->f(1)<<" \t"<<robot->leg[l]->f(2)<<std::endl;

    }
    std::pair<double, double> C = find_Centroid(vp);
    Eigen::Vector3d target(C.first,C.second,robot->height_z);

    this->p_T =  target;
    this->R_T = robot->R_c0 ;

    e_p_int = Eigen::Vector3d::Zero();
    e_o_int = Eigen::Vector3d::Zero();
    pid_out = Eigen::VectorXd::Zero(6);

    d_world_pos = Eigen::Vector4f::Ones();

    robot->leg[(int)robot->swingL_id]->storeInitG(); 
}
void LocomotionController::setDynamicPhase()
{
    robot->leg[(int) robot->swingL_id_a]->initQout(); // set all q_out as current value
    robot->leg[(int) robot->swingL_id_b]->initQout(); // set all q_out as current value

    this->t_phase = 0.0;
    this->t0_phase = t_real; // when that phase started relatve to real time
    this->ii = -1; // counter for bezier execution

    this->R_T = robot->R_c0 ;
    this->robot->p_c0 = this->robot->p_c;

    robot->leg[(int)robot->swingL_id_a]->storeInitG(); 
    robot->leg[(int)robot->swingL_id_b]->storeInitG(); 

    A_TOUCHED = false;
    B_TOUCHED = false;

    A_PD = false;
    B_PD = false; 

    this->tA = 0.0;
    this->tB = 0.0;

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
    robot->leg[(int) robot->swingL_id]->wv_leg = robot->leg[(int) robot->swingL_id]->w0*Eigen::Vector3d::Ones() + w_max*superGaussian(A,b,t_half_swing-t0_superG,t_phase - t_half_swing, 13)*Eigen::Vector3d::Ones(); 
    robot->vvvv.block((int) robot->swingL_id*3,0,3,1) = robot->leg[(int) robot->swingL_id]->wv_leg;   // update vvvv vector of robot 
    
    robot->W_inv = (robot->vvvv.asDiagonal()).inverse(); // save as matrix the inverse of diagonal vvvv vector

}
void LocomotionController::computeDynamicWeights()
{

    // Stance A weights updated based on probability
    int l = (int)robot->stanceL_id_a;
        robot->leg[l]->wv_leg(1) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(1) ; // y
        robot->leg[l]->wv_leg(0) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(0); // x
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   
    // Stance B weights updated based on probability
    l = (int)robot->stanceL_id_b;
        robot->leg[l]->wv_leg(1) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(1) ; // y
        robot->leg[l]->wv_leg(0) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(0); // x
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   

    if (A_TOUCHED)
        robot->leg[(int) robot->swingL_id_a]->wv_leg = robot->leg[(int) robot->swingL_id_a]->w0*Eigen::Vector3d::Ones() + w_max*(1 - sigmoid(t_phase - tA, c1*10, t0_swing/10))*Eigen::Vector3d::Ones(); 
    else
        robot->leg[(int) robot->swingL_id_a]->wv_leg = robot->leg[(int) robot->swingL_id_a]->w0*Eigen::Vector3d::Ones() + w_max*sigmoid(t_phase, c1, t0_swing)*Eigen::Vector3d::Ones(); 
    
    if (B_TOUCHED)
        robot->leg[(int) robot->swingL_id_b]->wv_leg = robot->leg[(int) robot->swingL_id_b]->w0*Eigen::Vector3d::Ones() + w_max*(1 - sigmoid(t_phase - tB, c1*10, t0_swing/10) )*Eigen::Vector3d::Ones(); 
    else
        robot->leg[(int) robot->swingL_id_b]->wv_leg = robot->leg[(int) robot->swingL_id_b]->w0*Eigen::Vector3d::Ones() + w_max*sigmoid(t_phase, c1, t0_swing)*Eigen::Vector3d::Ones(); 

    robot->vvvv.block((int) robot->swingL_id_a*3,0,3,1) = robot->leg[(int) robot->swingL_id_a]->wv_leg;   // update vvvv vector of robot 
    robot->vvvv.block((int) robot->swingL_id_b*3,0,3,1) = robot->leg[(int) robot->swingL_id_b]->wv_leg;   // update vvvv vector of robot 

    robot->W_inv = (robot->vvvv.asDiagonal()).inverse(); // save as matrix the inverse of diagonal vvvv vector

}
void LocomotionController::initWeights()
{

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->wv_leg = robot->leg[l]->w0*Eigen::Vector3d::Ones() ; // y

        // update vvvv vector of robot                          // z stays 1.0 do not change
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   
    }
}
void LocomotionController::errors()
{
    
    e_p = robot->p_c - p_T; // compute position error
    Re = robot->R_c*R_T.transpose(); // compute orientation error
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();

    e_v.block(0,0,3,1) = robot->dp_c ; // compute velocity error TODO
    e_v.block(3,0,3,1) = robot->w_CoM ; 
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ; 

}
void LocomotionController::computeSudoGq()
{
    // compute Gq eq. 2
    // top Identities remain the same
    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->Gq.block(3,l*3,3,3) =  scewSymmetric(robot->R_c*robot->leg[l]->p_i); //eq. 2 //SCEW
    }
    // std::cout<<"robot->Gq"<<std::endl;

    // std::cout<<robot->Gq<<std::endl;
    // compute Gp_sude eq. 7
    robot->Gq_sudo = robot->W_inv * robot->Gq.transpose()*(robot->Gq*robot->W_inv*robot->Gq.transpose()).inverse() ;
}
void LocomotionController::fComputations() // Target Control
{
    robot->F_c = pid_out; // Final Fc ep. 11 -> Target Control
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
        if(fabs(e_p(axis))<0.1)
        {
            e_p_int(axis) += e_p(axis)*dt; //position
        }
        if(fabs(e_o(axis))<0.1)
        {
            e_o_int(axis) += e_o(axis)*dt; //orientation
        }
    }

    pid_out.block(0,0,3,1) += -ki*e_p_int.block(0,0,3,1) ;
    pid_out.block(3,0,3,1) += -0.3*ki*e_o_int ;

    if(pid_out.block(0,0,3,1).norm()>40.0){
        pid_out.block(0,0,3,1) = pid_out.block(0,0,3,1) * 40.0 / pid_out.block(0,0,3,1).norm();
    }
    if(pid_out.block(3,0,3,1).norm()>40.0){
        pid_out.block(3,0,3,1) = pid_out.block(3,0,3,1) * 40.0 / pid_out.block(3,0,3,1).norm();
    }

    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);
    pid_out += Gbc*robot->gc;
    
}
void LocomotionController::inverseTip()
{
    // /**************** Bezier curve ***************/ 

        
    if(t_phase<t0_swing) //  yet there are forces
    {
        bezier_world = robot->leg[(int) robot->swingL_id]->g_0bo_init.block(0,3,3,1).cast<float>();
        CLIK(bezier_world , Eigen::Vector3f::Zero());

    }
    else if( t_phase>=t0_swing & t_phase<=(t0_swing + 1/freq_swing) ) 
    {
        ++ii;
        bezier_world = Eigen::Vector3f(bCurveX[ii],bCurveY[ii],bCurveZ[ii]);
        CLIK(bezier_world , Eigen::Vector3f(dot_bCurveX[ii],dot_bCurveY[ii],dot_bCurveZ[ii]));

    }
    else
    {
        // f_applied = robot->leg[(int)robot->swingL_id]->R_i.transpose()*robot->leg[(int)robot->swingL_id]->f;
        f_applied = robot->leg[(int)robot->swingL_id]->R_i.transpose()*robot->leg[(int)robot->swingL_id]->f;
        
        // if ( f_applied(2) < -7)
        // {
            bezier_world(2) -= 0.02*dt ;
            CLIK(bezier_world , Eigen::Vector3f(0.0,0.0,-0.02));
            
        // }
        // else
        // {
        //     CLIK(bezier_world , Eigen::Vector3f::Zero());

        // }
    }


}
void LocomotionController::doubleInverseTip()
{

    // /**************** Execute Bezier curve + Sigmoid to target z ***************/ 
    if(t_phase<t0_swing) //  yet there are forces
    {
        bezier_world_a = robot->leg[(int) robot->swingL_id_a]->g_0bo_init.block(0,3,3,1).cast<float>();
        bezier_world_b = robot->leg[(int) robot->swingL_id_b]->g_0bo_init.block(0,3,3,1).cast<float>();
        
        doubleCLIK(bezier_world_a, Eigen::Vector3f::Zero(), (int) robot->swingL_id_a);
        doubleCLIK(bezier_world_b, Eigen::Vector3f::Zero(), (int) robot->swingL_id_b);

    }
    else if( t_phase>=t0_swing & t_phase<(t0_swing + percentage*1/freq_swing) ) 
    {
        ++ii;
        bezier_world_a = Eigen::Vector3f(robot->leg[(int) robot->swingL_id_a]->bCurveX[ii],robot->leg[(int) robot->swingL_id_a]->bCurveY[ii],robot->leg[(int) robot->swingL_id_a]->bCurveZ[ii]);
        bezier_world_b = Eigen::Vector3f(robot->leg[(int) robot->swingL_id_b]->bCurveX[ii],robot->leg[(int) robot->swingL_id_b]->bCurveY[ii],robot->leg[(int) robot->swingL_id_b]->bCurveZ[ii]);

        doubleCLIK(bezier_world_a, Eigen::Vector3f(robot->leg[(int) robot->swingL_id_a]->dot_bCurveX[ii], robot->leg[(int) robot->swingL_id_a]->dot_bCurveY[ii],robot->leg[(int) robot->swingL_id_a]->dot_bCurveZ[ii]), (int) robot->swingL_id_a);
        doubleCLIK(bezier_world_b , Eigen::Vector3f(robot->leg[(int) robot->swingL_id_b]->dot_bCurveX[ii],robot->leg[(int) robot->swingL_id_b]->dot_bCurveY[ii],robot->leg[(int) robot->swingL_id_b]->dot_bCurveZ[ii]), (int) robot->swingL_id_b);
    
        A_PD = true; // Run PD mode
        B_PD = true; //Run PD mode
    }
    else if(t_phase>=(t0_swing + percentage*1/freq_swing) & t_phase<(t0_swing + percentage*1/freq_swing + dt))
    {
        ampli_A = robot->leg[(int)robot->swingL_id_a]->g_o_world(2,3) - tip_target_z;
        ampli_B = robot->leg[(int)robot->swingL_id_b]->g_o_world(2,3) - tip_target_z;
        freezedoubleCLIK((int)robot->swingL_id_a);
        freezedoubleCLIK((int)robot->swingL_id_b);
        t_down = t_phase + dt;
    }
    else
    {
        double t_virtual_A = inverse_sigmoid( 1 - (robot->leg[(int)robot->swingL_id_a]->g_o_world(2,3) - tip_target_z)/ampli_A  ,c1tip ,c2tip ) + t_down;
        double t_virtual_B = inverse_sigmoid( 1 - (robot->leg[(int)robot->swingL_id_b]->g_o_world(2,3) - tip_target_z)/ampli_B  ,c1tip ,c2tip ) + t_down;
        
        bezier_world_a = Eigen::Vector3f( bezier_world_a(0), bezier_world_a(1), tip_target_z + ampli_A*(1 - sigmoid( t_virtual_A + dt , c1tip, c2tip)));
        bezier_world_b = Eigen::Vector3f( bezier_world_b(0), bezier_world_b(1), tip_target_z + ampli_B*(1 - sigmoid( t_virtual_B + dt , c1tip, c2tip)));

        doubleCLIK(bezier_world_a ,Eigen::Vector3f(0.0,0.0,  ampli_A*(-der_sigmoid(t_virtual_A + dt , c1tip, c2tip) )) ,(int) robot->swingL_id_a );
        doubleCLIK(bezier_world_b ,Eigen::Vector3f(0.0,0.0,  ampli_B*(-der_sigmoid(t_virtual_B + dt , c1tip, c2tip) )) ,(int) robot->swingL_id_b );

        // bezier_world_a = Eigen::Vector3f( bezier_world_a(0), bezier_world_a(1), tip_target_z + ampli_A*(1 - sigmoid( t_phase - t_down , c1tip, c2tip)));
        // bezier_world_b = Eigen::Vector3f( bezier_world_b(0), bezier_world_b(1), tip_target_z + ampli_B*(1 - sigmoid( t_phase - t_down , c1tip, c2tip)));
// 
        // doubleCLIK(bezier_world_a ,Eigen::Vector3f(0.0,0.0,  ampli_A*(-der_sigmoid(t_phase - t_down , c1tip, c2tip) )) ,(int) robot->swingL_id_a );
        // doubleCLIK(bezier_world_b ,Eigen::Vector3f(0.0,0.0,  ampli_B*(-der_sigmoid(t_phase - t_down , c1tip, c2tip) )) ,(int) robot->swingL_id_b );

        checkTouchDown();
    }


}
void LocomotionController::checkTouchDown()
{
    f_applied_a = robot->leg[(int)robot->swingL_id_a]->R_i.transpose()*robot->leg[(int)robot->swingL_id_a]->f;
    f_applied_b = robot->leg[(int)robot->swingL_id_b]->R_i.transpose()*robot->leg[(int)robot->swingL_id_b]->f;

    f_stance_a = robot->leg[(int)robot->stanceL_id_a]->R_i.transpose()*robot->leg[(int)robot->stanceL_id_a]->f;
    f_stance_b = robot->leg[(int)robot->stanceL_id_b]->R_i.transpose()*robot->leg[(int)robot->stanceL_id_b]->f;


    if (A_TOUCHED)
    {
        freezedoubleCLIK((int) robot->swingL_id_a);
        // if( ( t_phase - tA) >= 2*(t0_swing/10)) //HERE CHECK IF IT NEEDED
            A_PD = false; // Cancel PD 
    }
    else if(f_applied_a(2) > force_thres)
    {
        A_TOUCHED = true;
        tA = t_phase;
    }


    if(B_TOUCHED)
    {
        freezedoubleCLIK((int) robot->swingL_id_b);
        // if( (t_phase - tB) >= 2*(t0_swing/10)) //HERE CHECK IF IT NEEDED
            B_PD = false; // Cancel PD 
    }
    else  if (f_applied_b(2) > force_thres )
    {
        B_TOUCHED = true;
        tB = t_phase;
    }
}
void LocomotionController::CLIK(Eigen::Vector3f pd_0frame_, Eigen::Vector3f dpd_0frame_)
{

    Eigen::Vector3f d_q_ = (   robot->R_c* robot->leg[(int)robot->swingL_id]->J.block<3,3>(0,0)).inverse().cast<float>()*(dpd_0frame_ - 64*(robot->leg[(int) robot->swingL_id]->g_o_world.block(0,3,3,1).cast<float>() - pd_0frame_) );
    // std::cout<<"leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>():"<< leg_mng[(int)robot_->swingL_id].g_o_world.block(0,3,3,1).cast<float>().transpose()<<std::endl;
    // std::cout<<"pd_0frame_:"<< pd_0frame_.transpose()<<std::endl;
    // std::cout<<"p--------------------------------------------"<< std::endl;
    robot->leg[(int)robot->swingL_id]->q_out(0) = d_q_(0)*dt + robot->leg[(int)robot->swingL_id]->q_out(0);
    robot->leg[(int)robot->swingL_id]->q_out(1) = d_q_(1)*dt + robot->leg[(int)robot->swingL_id]->q_out(1);
    robot->leg[(int)robot->swingL_id]->q_out(2) = d_q_(2)*dt + robot->leg[(int)robot->swingL_id]->q_out(2);
    robot->leg[(int)robot->swingL_id]->dq_out(0) = d_q_(0);
    robot->leg[(int)robot->swingL_id]->dq_out(1) = d_q_(1);
    robot->leg[(int)robot->swingL_id]->dq_out(2) = d_q_(2);
}
void LocomotionController::doubleCLIK(Eigen::Vector3f pd_0frame_A, Eigen::Vector3f dpd_0frame_A, int i)
{

    Eigen::Vector3f d_q_ = (   robot->R_c*robot->leg[i]->J.block<3,3>(0,0)).inverse().cast<float>()*(dpd_0frame_A - k_clik*(robot->leg[i]->g_o_world.block(0,3,3,1).cast<float>() - pd_0frame_A) );

    robot->leg[i]->q_out(0) =  d_q_(0)*dt + robot->leg[i]->q_out(0);
    robot->leg[i]->q_out(1) =  d_q_(1)*dt + robot->leg[i]->q_out(1);
    robot->leg[i]->q_out(2) =  d_q_(2)*dt + robot->leg[i]->q_out(2);
    robot->leg[i]->dq_out(0) = d_q_(0);
    robot->leg[i]->dq_out(1) = d_q_(1);
    robot->leg[i]->dq_out(2) = d_q_(2);


}
void LocomotionController::freezedoubleCLIK(int i)
{
    robot->leg[i]->q_out(0) =   robot->leg[i]->q_out(0);
    robot->leg[i]->q_out(1) =   robot->leg[i]->q_out(1);
    robot->leg[i]->q_out(2) =   robot->leg[i]->q_out(2);
    robot->leg[i]->dq_out(0) = 0.0;
    robot->leg[i]->dq_out(1) = 0.0;
    robot->leg[i]->dq_out(2) = 0.0;
}
void LocomotionController::computeBesierCurve2D(double step) // static gait pre-fixed swinging trajectory
{
    std::vector<double> xX{0.0, 0.04, 0.05, 0.03}; // 0.08, 0.09, 0.07 HERE TODO
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
void LocomotionController::spiralExploration()
{
    std::cout<<"Now I'm exploring"<<std::endl;
}
void LocomotionController::computeWeightsSigmoid()
{

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->wv_leg(1) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(1) ; // y
        robot->leg[l]->wv_leg(0) = this->alpha*(1.0 - robot->leg[l]->prob_stable)*dt + robot->leg[l]->wv_leg(0); // x

        // update vvvv vector of robot                          // z stays 1.0 do not change
        robot->vvvv.block(l*3,0,3,1) = robot->leg[l]->wv_leg;   
    }
    /* Added to simuate swing leg weights t inf */ //TODO
    robot->leg[(int) robot->swingL_id]->wv_leg = robot->leg[(int) robot->swingL_id]->w0*Eigen::Vector3d::Ones() + w_max*sigmoid(t_phase, c1, t0_swing)*Eigen::Vector3d::Ones(); 
    robot->vvvv.block((int) robot->swingL_id*3,0,3,1) = robot->leg[(int) robot->swingL_id]->wv_leg;   // update vvvv vector of robot 
    
    robot->W_inv = (robot->vvvv.asDiagonal()).inverse(); // save as matrix the inverse of diagonal vvvv vector

}
void LocomotionController::signalFc()
{
    pid_out.block(0,0,3,1) = -kp*e_p; // third term of Fc eq. 11
    pid_out.block(3,0,3,1) = -ko*e_o; 
    pid_out -=  this->kv*this->e_v ;

    // /*  Integral -> Target Control */ 
    for (int axis = 0; axis<3; axis++)
    {
        if(fabs(e_p(axis))<0.1)
        {
            e_p_int(axis) += e_p(axis)*dt; //position
        }
        if(fabs(e_o(axis))<0.1)
        {
            e_o_int(axis) += e_o(axis)*dt; //orientation
        }
    }
    // if(fabs(e_p(2))<0.1)
    // {
    //     e_p_int(2) += e_p(2)*dt; //position
    // }
    pid_out.block(0,0,3,1) += -ki*e_p_int.block(0,0,3,1) ;
    // pid_out.block(3,0,3,1) += -0.3*ki*e_o_int ;

    // if(pid_out.block(0,0,3,1).norm()>40.0){
    //     pid_out.block(0,0,3,1) = pid_out.block(0,0,3,1) * 40.0 / pid_out.block(0,0,3,1).norm();
    // }
    // if(pid_out.block(3,0,3,1).norm()>40.0){
    //     pid_out.block(3,0,3,1) = pid_out.block(3,0,3,1) * 40.0 / pid_out.block(3,0,3,1).norm();
    // }


    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);
    pid_out += Gbc*robot->gc;
    
}
void LocomotionController::generateBezier(double step)
{
    dot_bCurveX.clear();
    dot_bCurveY.clear();
    dot_bCurveZ.clear();
    bCurveX.clear();
    bCurveY.clear();
    bCurveZ.clear();

    // double step = freq_swing*dt/(1+dt*freq_swing);

    Eigen::Vector3d ofset = Eigen::Vector3d(0.1, robot->leg[(int) robot->swingL_id]->pros*0.01, 0.02);
    // Eigen::Vector3d ofset = Eigen::Vector3d(0.05, robot->leg[(int) robot->swingL_id]->pros*0.01, 0.02);
    
    Eigen::Vector3d p0 = robot->leg[(int) robot->swingL_id]->g_0bo_init.block(0,3,3,1);
    Eigen::Vector3d p3 = p0 + ofset;
    Eigen::Vector3d p1, p2;
    p1(0) = p0(0) + 0.5*ofset(0);   p2(0) = p0(0) + 0.8*ofset(0);
    p1(1) = p0(1) + 0.5*ofset(1);     p2(1) = p0(1) + 0.8*ofset(1);
    p1(2) = p0(2) + 2.5*ofset(2);   p2(2) = p0(2) + 2.8*ofset(2);
    std::cout<<"swing leg "<<robot->swingL_id<<std::endl;
    std::cout<<"p0"<<p0(0)<<" "<<p0(1)<<" "<<p0(2)<<std::endl;
    std::cout<<"p1"<<p1(0)<<" "<<p1(1)<<" "<<p1(2)<<std::endl;
    std::cout<<"p2"<<p2(0)<<" "<<p2(1)<<" "<<p2(2)<<std::endl;
    std::cout<<"p3"<<p3(0)<<" "<<p3(1)<<" "<<p3(2)<<std::endl;

    std::vector<double> xX{p0(0), p1(0), p2(0), p3(0)}; 
    std::vector<double> yY{p0(1), p1(1), p2(1), p3(1)}; 
    std::vector<double> zZ{p0(2), p1(2), p2(2), p3(2)};

    double bCurveXt, dot_bCurveXt;
    double bCurveYt, dot_bCurveYt;
    double bCurveZt, dot_bCurveZt;
    for (double t = 0.0; t <= 1; t += step)
    {
        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3]; //  p0(0) + ofset(0)*t; //
        bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3]; // p0(1) + ofset(1)*t;// 
        bCurveZt = std::pow((1 - t), 3) * zZ[0] + 3 * std::pow((1 - t), 2) * t * zZ[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * zZ[2] + std::pow(t, 3) * zZ[3];
        bCurveX.push_back(bCurveXt);
        bCurveY.push_back(bCurveYt);
        bCurveZ.push_back(bCurveZt);
        dot_bCurveXt = 3 * std::pow((1 - t), 2) *(xX[1] - xX[0])  + 6 * (1 - t) * t * (xX[2] - xX[1]) + 3 * std::pow(t, 2) * ( xX[3] - xX[2] );
        dot_bCurveYt = 3 * std::pow((1 - t), 2) *(yY[1] - yY[0])  + 6 * (1 - t) * t * (yY[2] - yY[1]) + 3 * std::pow(t, 2) * ( yY[3] - yY[2] );
        dot_bCurveZt = 3 * std::pow((1 - t), 2) *(zZ[1] - zZ[0])  + 6 * (1 - t) * t * (zZ[2] - zZ[1]) + 3 * std::pow(t, 2) * ( zZ[3] - zZ[2] );
        dot_bCurveX.push_back(dot_bCurveXt);
        dot_bCurveY.push_back(dot_bCurveYt);
        dot_bCurveZ.push_back(dot_bCurveZt);
    }
}
void LocomotionController::dynamicBezier(Leg* l, Eigen::Vector3d dp_cmd)
{
    l->dot_bCurveX.clear();
    l->dot_bCurveY.clear();
    l->dot_bCurveZ.clear();
    l->bCurveX.clear();
    l->bCurveY.clear();
    l->bCurveZ.clear();

    Eigen::Vector3d p0 = l->g_0bo_init.block(0,3,3,1);

    // Eigen::Vector2d help_vect = robot->R_c.block(0,0,2,2)*(robot->p_c0.block(0,0,2,1) + l->TIP_EXT) ;
    Eigen::Vector3d p3(robot->p_c0 + robot->R_c0*l->TIP_EXT + dp_cmd*(t0_swing + 1/freq_swing + 2*c2tip) ) ;
    p3(2) = p0(2);
    l->foothold = p3; 
    Eigen::Vector3d ofset = p3 - p0;
    ofset(2) = 0.019;

    Eigen::Vector3d p1, p2;
    p1(0) = p0(0) + 0.5*ofset(0);   p2(0) = p0(0) + 0.8*ofset(0);
    p1(1) = p0(1) + 0.5*ofset(1);   p2(1) = p0(1) + 0.8*ofset(1);
    p1(2) = p0(2) + 1.5*ofset(2);   p2(2) = p0(2) + 1.8*ofset(2); // 0.019 1.5 1.8 

    std::vector<double> xX{p0(0), p1(0), p2(0), p3(0)}; 
    std::vector<double> yY{p0(1), p1(1), p2(1), p3(1)}; 
    std::vector<double> zZ{p0(2), p1(2), p2(2), p3(2)};

    double bCurveXt, dot_bCurveXt;
    double bCurveYt, dot_bCurveYt;
    double bCurveZt, dot_bCurveZt;
    for (double t = 0.0; t <= 1; t += step_bez)
    {
        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3]; //  p0(0) + ofset(0)*t; //
        bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3]; // p0(1) + ofset(1)*t;// 
        bCurveZt = std::pow((1 - t), 3) * zZ[0] + 3 * std::pow((1 - t), 2) * t * zZ[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * zZ[2] + std::pow(t, 3) * zZ[3];
        l->bCurveX.push_back(bCurveXt);
        l->bCurveY.push_back(bCurveYt);
        l->bCurveZ.push_back(bCurveZt);
        dot_bCurveXt = 3 * std::pow((1 - t), 2) *(xX[1] - xX[0])  + 6 * (1 - t) * t * (xX[2] - xX[1]) + 3 * std::pow(t, 2) * ( xX[3] - xX[2] );
        dot_bCurveYt = 3 * std::pow((1 - t), 2) *(yY[1] - yY[0])  + 6 * (1 - t) * t * (yY[2] - yY[1]) + 3 * std::pow(t, 2) * ( yY[3] - yY[2] );
        dot_bCurveZt = 3 * std::pow((1 - t), 2) *(zZ[1] - zZ[0])  + 6 * (1 - t) * t * (zZ[2] - zZ[1]) + 3 * std::pow(t, 2) * ( zZ[3] - zZ[2] );
        l->dot_bCurveX.push_back(dot_bCurveXt);
        l->dot_bCurveY.push_back(dot_bCurveYt);
        l->dot_bCurveZ.push_back(dot_bCurveZt);
    }
    
}
void LocomotionController::swingTrajectory()
{
    if(t_phase<t0_swing) //  yet there are forces
    {
        bezier_world = robot->leg[(int) robot->swingL_id]->g_0bo_init.block(0,3,3,1).cast<float>();
        CLIK(bezier_world , Eigen::Vector3f::Zero());

    }
    else if( t_phase>=t0_swing & t_phase<=(t0_swing + 1/freq_swing) ) 
    {
        ++ii;
        bezier_world = Eigen::Vector3f(bCurveX[ii],bCurveY[ii],bCurveZ[ii]);
        CLIK(bezier_world , Eigen::Vector3f(dot_bCurveX[ii],dot_bCurveY[ii],dot_bCurveZ[ii]));

    }
    else
    {
        CLIK(bezier_world , Eigen::Vector3f::Zero());
    }
}
void LocomotionController::dynaControlSignal()
{
    // robot->F_c.block(0,0,3,1) = - kv*(robot->dp_c - dp_cmd) + Eigen::Vector3d(0,0,robot->mass*robot->g_gravity);
    Gbc.block(3,0,3,3) = scewSymmetric(robot->R_c*robot->pbc);
    robot->F_c = - kv*e_v + Gbc*robot->gc;
    robot->F_c.block(3,0,3,1) -= ko*e_o;

    robot->F_c(2) -= kp*e_p(2); 

    computeSudoGq();

    robot->F_a = robot->Gq_sudo*robot->F_c ;

    for(int l = 0; l < robot->n_legs ; l++)
    {
        robot->leg[l]->f_cmd = -robot->F_a.block(l*3,0,3,1); // slip Fa eq. 3
        // robot->leg[l]->f_cmd(2) =  std::fmin(robot->leg[l]->f_cmd(2), 0.0);
        robot->leg[l]->tau =  (robot->R_c*(robot->leg[l]->J.block<3,3>(0,0))).transpose()*robot->leg[l]->f_cmd; // compute eq. 4
    }

}
void LocomotionController::dynaErrors(Eigen::Vector3d dp_cmd)
{
    // compute orientation ERROR
    Re = robot->R_c*robot->R_c0.transpose();
    ang.fromRotationMatrix(Re);
    e_o = ang.angle()*ang.axis();
    
    e_p(2) = kp*(robot->p_c(2) - robot->height_z);

    // HERE change commanded velocity based on the current robots ori??
    // here robot->dCoM_p
    e_v.block(0,0,3,1) = (robot->dp_c - robot->R_c*dp_cmd) ; // compute velocity error TODO //robot->R_c0*
    e_v.block(3,0,3,1) = robot->w_CoM ; 
    e_v.block(3,0,3,1) = 0.7*this->e_v.block(3,0,3,1) ; 
}