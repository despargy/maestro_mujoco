#include <LocomotionTopLevelControl.h>
#include <Perception.h>
/* Construtor */
LocomotionTopLevelControl::LocomotionTopLevelControl()
{
    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new LocomotionController();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(); // com. with Mujoco/ROS
    t_last_c = 0.0;

    loc_i = -1 ;
    dp_cmd<< 0.0,0.0,0.0;
    foothold_id = 0;

}
LocomotionTopLevelControl::LocomotionTopLevelControl(std::string category_)
{
    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new LocomotionController();
    this->data = new Data(); // init Data 
    this->wrapper = new Wrapper(category_, controller->robot); // com. with Mujoco/ROS
    t_last_c = 0.0;
    loc_i = -1 ; 
    dp_cmd<< 0.0,0.0,0.0;
    foothold_id = 0;
}
/* De-Constructor */
LocomotionTopLevelControl::~LocomotionTopLevelControl()
{
    /* De-Allocate memory */ 
    delete this->fsm ;
    delete this->controller;
    delete this->data;
    delete this->wrapper;
}


/* Function to initialize variables and/or call other init functions */
void LocomotionTopLevelControl::setParams()
{
    data->init_save_data_locomotion(); // initialization function of Data
    controller->loop_index = 0; // is used to save data in specific loop frequency
    
    /*  get from params.h */
    controller->ADAPT_A = param_ADAPT_A;
    controller->ADAPT_B = param_ADAPT_B;
    controller->robot->KEEP_CONTROL = param_KEEP_CONTROL;
    controller->robot->mass = GO1_param_mass; // same as mujoco .xml
    controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
    controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;
    controller->robot->pbc(0) = param_pbc_x;
    controller->robot->pbc(1) = param_pbc_y;
    controller->robot->pbc(2) = param_pbc_z;

    controller->dt = param_dt; // same as mujoco .xml

    DYNA_LOCO = param_DYNA_LOCO;

    controller->kp = param_kp_LOC;
    controller->kv = param_kv_LOC;
    controller->ko = param_ko_LOC;
    controller->ki = param_ki_LOC;

    controller->alpha = param_alpha; // same as mujoco .xml
    controller->slope = param_slope; // same as mujoco .xml

    controller->A = param_A_SGaus;
    controller->b = param_b_SGaus;
    controller->t0_superG = param_t0_SGaus;
    controller->freq_swing =    param_Freq_Swing;
    controller->t0_swing =      param_t0_Swing;
    controller->t_half_swing =  param_thalf_Swing ; 
    controller->swing_t_slot =  param_slot_Swing; 

    controller->robot->height_z = GO1_param_robot_z;
    controller->w_max = param_w_max;

    wrapper->Kp_hip   = GO1_param_Kp_hip;
    wrapper->Kp_thing = GO1_param_Kp_thing;
    wrapper->Kp_calf  = GO1_param_Kp_calf;
    wrapper->Kv_hip   = param_Kv_hip;
    wrapper->Kv_thing = param_Kv_thing;
    wrapper->Kv_calf  = param_Kv_calf;

}
/* Function to initialize variables and/or call other init functions */
void LocomotionTopLevelControl::setParamsDynamic()
{
    data->init_save_data_dynamic(); // initialization function of Data
    controller->loop_index = 0; // is used to save data in specific loop frequency
    
                    /*  get from params.h */

    controller->robot->KEEP_CONTROL = param_KEEP_CONTROL;

    controller->robot->pbc(0) = param_pbc_x;
    controller->robot->pbc(1) = param_pbc_y;
    controller->robot->pbc(2) = param_pbc_z;

    controller->dt = param_dt; // same as mujoco .xml
    controller->kp = param_kp_DYNA;
    controller->kv = param_kv_DYNA;
    controller->ko = param_ko_DYNA;
    controller->kw = param_kw_DYNA;
    controller->A = param_A_SGaus_DYNA;
    controller->b = param_b_SGaus_DYNA;
    controller->t0_superG =     param_t0_SGaus_DYNA;
    controller->freq_swing =    param_Freq_Swing_DYNA;
    controller->t0_swing =      param_t0_Swing_DYNA;
    controller->t_half_swing =  param_thalf_Swing_DYNA ; 
    controller->swing_t_slot =  param_slot_Swing_DYNA; 
    controller->ADAPT_A = param_ADAPT_A;
    controller->ADAPT_B = param_ADAPT_B;

    controller->w_max = param_w_max;
    controller->c1 = param_c1;
    controller->force_thres = param_force_thres;
    controller->c1tip = param_c1tip;
    controller->c2tip = param_c2tip;
    controller->tip_target_z = param_tip_target_z;
    controller->percentage = param_percentage;

    wrapper->Kv_hip   = param_Kv_hip;   // not used
    wrapper->Kv_thing = param_Kv_thing; // not used
    wrapper->Kv_calf  = param_Kv_calf;  // not used

    controller->tau_lim = param_tau_lim;
    controller->INCLINATION = param_INCLINATION
                    /**** Specified by each model ****/
    int model_id = param_model;
    std::cout<<model_id<<std::endl;
    if( model_id == 0)
    {
        controller->robot->mass = GO1_param_mass; // same as mujoco .xml
        controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
        controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;

        controller->robot->leg[0]->sit1[0] = GO1_param_sit1_0;
        controller->robot->leg[0]->sit1[1] = GO1_param_sit1_1;
        controller->robot->leg[0]->sit1[2] = GO1_param_sit1_2;

        dp_cmd(0) = GO1_param_dp_cmd_x ;
        dp_cmd(1) = GO1_param_dp_cmd_y ;
        dp_cmd(2) = GO1_param_dp_cmd_z ;

        controller->robot->height_z = GO1_param_robot_z;
        controller->terrain_height = param_terrain_height;
        
        wrapper->Kp_hip   = GO1_param_Kp_hip;
        wrapper->Kp_thing = GO1_param_Kp_thing;
        wrapper->Kp_calf  = GO1_param_Kp_calf;

        controller->k_clik = GO1_param_k_clik;

        controller->robot->l1 = GO1_param_l1;
        controller->robot->l2 = GO1_param_l2;
        controller->robot->d =  GO1_param_d;

        controller->alpha = GO1_param_alpha_DYNA; 

    }
    else if (model_id == 1)
    {
        controller->robot->mass = GO1_UNITREE_param_mass; // same as mujoco .xml
        controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
        controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;

        controller->robot->leg[0]->sit1[0] = GO1_param_sit1_0;
        controller->robot->leg[0]->sit1[1] = GO1_param_sit1_1;
        controller->robot->leg[0]->sit1[2] = GO1_param_sit1_2;

        dp_cmd(0) = GO1_UNITREE_param_dp_cmd_x ;
        dp_cmd(1) = GO1_UNITREE_param_dp_cmd_y ;
        dp_cmd(2) = GO1_UNITREE_param_dp_cmd_z ;

        controller->robot->height_z = GO1_UNITREE_param_robot_z;

        wrapper->Kp_hip   = GO1_UNITREE_param_Kp_hip;
        wrapper->Kp_thing = GO1_UNITREE_param_Kp_thing;
        wrapper->Kp_calf  = GO1_UNITREE_param_Kp_calf;

        controller->k_clik = GO1_UNITREE_param_k_clik;

        controller->robot->l1 = GO1_UNITREE_param_l1;
        controller->robot->l2 = GO1_UNITREE_param_l2;
        controller->robot->d =  GO1_param_d;

        controller->alpha = GO1_UNITREE_param_alpha_DYNA; 

    }
    else if (model_id == 2)
    {
        controller->robot->mass = GO2_UNITREE_param_mass; // same as mujoco .xml
        controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
        controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;

        controller->robot->leg[0]->sit1[0] = GO2_UNITREE_param_sit1_0;
        controller->robot->leg[0]->sit1[1] = GO2_UNITREE_param_sit1_1;
        controller->robot->leg[0]->sit1[2] = GO2_UNITREE_param_sit1_2;

        dp_cmd(0) = GO2_UNITREE_param_dp_cmd_x ;
        dp_cmd(1) = GO2_UNITREE_param_dp_cmd_y ;
        dp_cmd(2) = GO2_UNITREE_param_dp_cmd_z ;

        controller->robot->height_z = GO2_UNITREE_param_robot_z;
        controller->terrain_height = param_terrain_height;

        wrapper->Kp_hip   = GO2_UNITREE_param_Kp_hip;
        wrapper->Kp_thing = GO2_UNITREE_param_Kp_thing;
        wrapper->Kp_calf  = GO2_UNITREE_param_Kp_calf;

        controller->k_clik = GO2_UNITREE_param_k_clik;

        controller->robot->l1 = GO2_UNITREE_param_l1;
        controller->robot->l2 = GO2_UNITREE_param_l2;
        controller->robot->d =  GO2_UNITREE_param_d; 

        controller->alpha = GO2_UNITREE_param_alpha_DYNA; 

        controller->robot->w_d(0) = GO2_UNITREE_param_w_cmd_x ;
        controller->robot->w_d(1) = GO2_UNITREE_param_w_cmd_y ;
        controller->robot->w_d(2) = GO2_UNITREE_param_w_cmd_z ;

    }

    controller->robot->leg[0]->TIP_EXT = Eigen::Vector3d(+controller->robot->l1, -controller->robot->d, 0.019);
    controller->robot->leg[2]->TIP_EXT = Eigen::Vector3d(-controller->robot->l2, -controller->robot->d, 0.019);
    controller->robot->leg[1]->TIP_EXT = Eigen::Vector3d(+controller->robot->l1, +controller->robot->d, 0.019);
    controller->robot->leg[3]->TIP_EXT = Eigen::Vector3d(-controller->robot->l2, +controller->robot->d, 0.019);

}

/* Function to initialize variables and/or call other init functions */
void LocomotionTopLevelControl::setParamsExploration()
{
    data->init_save_data_locomotion(); // initialization function of Data
    controller->loop_index = 0; // is used to save data in specific loop frequency
    
    // /*  get from params.h */
    // controller->ADAPT_A = param_ADAPT_A;
    // controller->ADAPT_B = param_ADAPT_B;
    // controller->robot->KEEP_CONTROL = param_KEEP_CONTROL;
    controller->robot->mass = GO1_param_mass; // same as mujoco .xml
    controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
    controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;
    controller->robot->pbc(0) = param_pbc_x;
    controller->robot->pbc(1) = param_pbc_y;
    controller->robot->pbc(2) = param_pbc_z;

    controller->dt = param_dt; // same as mujoco .xml

    controller->kp = param_kp_EXP;
    controller->kv = param_kv_EXP;
    controller->ko = param_ko_EXP;
    controller->ki = param_ki_EXP;

    controller->alpha = param_alpha; // same as mujoco .xml
    controller->slope = param_slope; // same as mujoco .xml

    controller->freq_swing =    param_Freq_Swing_EXP;
    controller->t0_swing =      param_t0_Swing_EXP;
    // controller->t_half_swing =  param_thalf_Swing ; 
    // controller->swing_t_slot =  param_slot_Swing; 

    controller->robot->height_z = GO1_param_robot_z;

    controller->w_max = param_w_max;

    wrapper->Kp_hip   = GO1_param_Kp_hip;
    wrapper->Kp_thing = GO1_param_Kp_thing;
    wrapper->Kp_calf  = GO1_param_Kp_calf;
    wrapper->Kv_hip   = param_Kv_hip;
    wrapper->Kv_thing = param_Kv_thing;
    wrapper->Kv_calf  = param_Kv_calf;
}
void LocomotionTopLevelControl::compute(double top_time)
{
    /*          Compute 
        Case definition at FSM.h    */

    switch (fsm->state)
    {
    /* Configure init stay down pose for robot */
    case S0: 
        controller->PD(controller->robot->leg[0]->sit0);
        
        if(top_time > fsm->t_S1)
            this->fsm->state = S1 ; // next fsm state -> StandUp S1
            // control fsm sequence based on time
        break;
    
    /* Configure init stay down pose for robot */
    case S1:

        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
        if(top_time > fsm->t_MC)
        {
            init_StaticGait();
            controller->t0 = top_time;  // t0 -> start of tracking
            controller->t_real = 0.0; //time_now - t0;

            fsm->state = STATIC_GAIT;
            fsm->phase = PH_TARGET;
        }
        break; // this break might add 0.002 = dt to time

    /* Locomotion cases based on gait - Upcoming gaits... */ 
    case STATIC_GAIT:
        
        controller->t_real = top_time - controller->t0; //update time
        // TODO change condition - make it better
        if( controller->t_phase >= controller->swing_t_slot )// TODO change by forces not time 
            fsm->phase = PH_TARGET;

        switch (fsm->phase )
        {
        case PH_TARGET:
            controller->robot->swingL_id = controller->static_free_gait[(int)(++loc_i%controller->robot->n_legs)]; // change swing leg by free gat order
            std::cout<<controller->robot->swingL_id <<std::endl;
            controller->setPhaseTarget(); // setphase target etc.
            controller->generateBezier(controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing));
            fsm->phase = PH_SWING;
            
        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;
            controller->computeWeightsSwing();
            controller->errors();
            controller->computeSudoGq();
            controller->PIDwithSat();
            controller->fComputations();
            controller->inverseTip();


            break;
        }

        data->save_loc(controller->t_real,controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(0,3),
                        controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(1,3), controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(2,3),
                        controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
                        controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        // controller->d_world_pos(0), controller->d_world_pos(1),controller-> d_world_pos(2),
                        controller->bezier_world(0),controller->bezier_world(1),controller->bezier_world(2),
                        controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                        controller->p_T(0),controller->p_T(1),controller->p_T(2),
                        controller->f_applied.norm()); 
        break;    
  
    }

    t_last_c += controller->dt;

    
}
void LocomotionTopLevelControl::computeDynamic(double top_time)
{
    /*          Compute 
        Case definition at FSM.h    */

    switch (fsm->state)
    {
    /* Configure init stay down pose for robot */
    case S0: 
        controller->PD(controller->robot->leg[0]->sit0);
        
        if(top_time > fsm->t_S1)
            this->fsm->state = S2 ; // next fsm state -> StandUp S1

        break;

    /* Configure StandUp pose for robot */
    case S2:
        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
        if(top_time > fsm->t_S3)
        {
            // std::cout<< " New packet"<<std::endl;
            // for(int l=0; l< controller->robot->n_legs; l++)
            // {
            //     std::cout<<controller->robot->leg[l]->tau(0)<<" , "<<
            //     controller->robot->leg[l]->tau(1)<<" , "<<
            //     controller->robot->leg[l]->tau(2)<<std::endl;
            // }
            
            fsm->state = S3; // stand up, go to init IMUs measurements
            std::cout<<"Robot stand up, I will move to init IMUs before locomotion"<<std::endl;
        }
        
        break;
    case S3:
    
        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000); // robot remains
        if(top_time > fsm->t_MC)
        {
            controller->t0 = top_time;  // t0 -> start of tracking
            controller->t_real = 0.0; //time_now - t0;
            controller->t_phase = 0.0;

            controller->robot->R_d = controller->robot->R_c;
            controller->robot->R_c0 = controller->robot->R_c;
            controller->robot->p_c0 = controller->robot->p_c;

            controller->robot->com_p_prev =  controller->robot->p_c0; // set init value to prev CoM position
            controller->robot->R_CoM_prev = controller->robot->R_c0;

            controller->step_bez = controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing);

            fsm->state = DYNA_GAIT;
            fsm->phase = PH_TARGET;

            // Change state if you need inclination adaptation
            if (controller->INCLINATION)
                fsm->state = INCLINED_DYNA_GAIT;

            controller->robot->leg[0]->foothold = controller->robot->leg[0]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[1]->foothold = controller->robot->leg[1]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[2]->foothold = controller->robot->leg[2]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[3]->foothold = controller->robot->leg[3]->g_0bo_init.block(0,3,3,1);
            
        }
        break;
   
    case DYNA_GAIT:

        controller->t_real = top_time - controller->t0; //update time
        // std::cout<<"Time : "<< controller->t_real <<std::endl;
        // //Dyna Locomotion
        switch (fsm->phase )
        {
        case PH_TARGET:
            
            controller->robot->swingL_id_a = (int)(++loc_i%2); // change swing leg by free gat order
            controller->robot->swingL_id_b = (int) (3 - controller->robot->swingL_id_a); //( case A.{0,3} , case B.{1,2} )
            controller->robot->stanceL_id_a = (int)((loc_i+1)%2);
            controller->robot->stanceL_id_b = (int) (3 - controller->robot->stanceL_id_a);
            phase_id = (int )controller->robot->swingL_id_a;
            controller->setDynamicPhase(); // setphase target etc. 

            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_a],dp_cmd);
            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_b],dp_cmd);

            fsm->phase = PH_SWING;

   
        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;

            // Commanded velocity
            controller->computeDynamicWeights();
            Eigen::Vector3d temp_dp_cmd = (controller->t_real < 4.0) ?  (controller->t_real)/4.0*dp_cmd : dp_cmd ;
            controller->dynaErrors(temp_dp_cmd);
            controller->dynaControlSignal();
            controller->doubleInverseTip();
            //CHECK THIS condition
            if ( controller->A_TOUCHED & controller->B_TOUCHED & (controller->robot->leg[(int)controller->robot->swingL_id_a]->wv_leg == controller->robot->leg[0]->w0*Eigen::Vector3d::Ones()) & (controller->robot->leg[(int)controller->robot->swingL_id_b]->wv_leg == controller->robot->leg[0]->w0*Eigen::Vector3d::Ones()) )
            {
                fsm->phase = PH_TARGET;
            }

            break;
        }
   
//THIS save_dyna was before
        // data->save_dyna(controller->t_real,
        //                 controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),

        //                 controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
        //                 controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        
        //                 controller->robot->leg[0]->foothold(0),controller->robot->leg[0]->foothold(1),controller->robot->leg[0]->foothold(2),
        //                 controller->robot->leg[1]->foothold(0),controller->robot->leg[1]->foothold(1),controller->robot->leg[1]->foothold(2),
        //                 controller->robot->leg[2]->foothold(0),controller->robot->leg[2]->foothold(1),controller->robot->leg[2]->foothold(2),
        //                 controller->robot->leg[3]->foothold(0),controller->robot->leg[3]->foothold(1),controller->robot->leg[3]->foothold(2),

        //                 controller->robot->leg[0]->g_o_world(0,3),           controller->robot->leg[0]->g_o_world(1,3),           controller->robot->leg[0]->g_o_world(2,3),
        //                 controller->robot->leg[1]->g_o_world(0,3),           controller->robot->leg[1]->g_o_world(1,3),           controller->robot->leg[1]->g_o_world(2,3),
        //                 controller->robot->leg[2]->g_o_world(0,3),           controller->robot->leg[2]->g_o_world(1,3),           controller->robot->leg[2]->g_o_world(2,3),
        //                 controller->robot->leg[3]->g_o_world(0,3),           controller->robot->leg[3]->g_o_world(1,3),           controller->robot->leg[3]->g_o_world(2,3),
                        
        //                 // controller->f_applied_a(0), controller->f_applied_a(1), controller->f_applied_a(2),
        //                 // controller->f_applied_b(0), controller->f_applied_b(1), controller->f_applied_b(2),

        //                 controller->bezier_world_a(0), controller->bezier_world_a(1), controller->bezier_world_a(2),
        //                 controller->bezier_world_b(0), controller->bezier_world_b(1), controller->bezier_world_b(2),
        //                                         controller->e_v(0),controller->e_v(1),controller->e_v(2)

                        
        //                 );

        // data->save_compare( controller->t_real, controller->robot->dp_c(0), controller->robot->dp_c(1), controller->robot->p_c(2),
        //                 controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
        //                 controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
        //                 controller->e_o(0), controller->e_o(1), controller->e_o(2),
        //                 controller->e_v(0), controller->e_v(1), controller->e_v(2),
        //                 controller->robot->leg[0]->tau.norm(), controller->robot->leg[1]->tau.norm(),controller->robot->leg[2]->tau.norm(),controller->robot->leg[3]->tau.norm());
// UNCOMMENT SAVE_SLIP //
        // data->save_slip(controller->t_real,
        //                 controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),

        //                 controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
        //                 controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        
        //                 controller->robot->leg[0]->prob_stable,controller->robot->leg[1]->prob_stable,
        //                 controller->robot->leg[2]->prob_stable, controller->robot->leg[3]->prob_stable,
                        
        //                 controller->robot->leg[(int)controller->robot->stanceL_id_a]->wv_leg(0),controller->robot->leg[(int)controller->robot->stanceL_id_b]->wv_leg(0),
        //                 controller->robot->leg[(int)controller->robot->stanceL_id_a]->prob_stable,controller->robot->leg[(int)controller->robot->stanceL_id_b]->prob_stable,

        //                 controller->robot->leg[(int)controller->robot->swingL_id_a]->wv_leg(0),controller->robot->leg[(int)controller->robot->swingL_id_b]->wv_leg(0),
        //                 controller->robot->leg[(int)controller->robot->swingL_id_a]->prob_stable,controller->robot->leg[(int)controller->robot->swingL_id_b]->prob_stable,

        //                 controller->e_v(0),controller->e_v(1),controller->e_v(2),

        //                 // controller->robot->leg[0]->imu(0),controller->robot->leg[0]->imu(1),controller->robot->leg[0]->imu(2),
                       
        //                 controller->f_applied_a(2),controller->f_applied_b(2),
        //                controller->f_stance_a(2),controller->f_stance_b(2),

        //                 controller->robot->dp_c(0),controller->robot->dp_c(1),controller->robot->dp_c(2)
        //                 );
                        
        // data->save_opt(controller->t_real, controller->robot->vvvv,controller->robot->Gq);       
        // data->save_Fa(controller->t_real, controller->robot->F_a);
        // data->save_Fc(controller->t_real, phase_id, controller->robot->F_c);
//////////////  Learning  /////////////
        // data->save_joints(controller->t_real, phase_id,
        //                     controller->robot->leg[0]->q_out(0), controller->robot->leg[0]->q_out(1), controller->robot->leg[0]->q_out(2),
        //                     controller->robot->leg[1]->q_out(0), controller->robot->leg[1]->q_out(1), controller->robot->leg[1]->q_out(2),
        //                     controller->robot->leg[2]->q_out(0), controller->robot->leg[2]->q_out(1), controller->robot->leg[2]->q_out(2),
        //                     controller->robot->leg[3]->q_out(0), controller->robot->leg[3]->q_out(1), controller->robot->leg[3]->q_out(2));
        // data->save_tau(controller->t_real, phase_id, controller->robot->leg[0]->tau, controller->robot->leg[1]->tau, controller->robot->leg[2]->tau, controller->robot->leg[3]->tau);
        // data->save_PP(controller->t_real, phase_id,
        //                 controller->robot->leg[0]->prob_stable,controller->robot->leg[1]->prob_stable,
        //                 controller->robot->leg[2]->prob_stable, controller->robot->leg[3]->prob_stable,
        //                 controller->robot->vvvv );

        data->save_CoM(controller->t_real, phase_id, 
                        controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                        controller->robot->dp_c(0),controller->robot->dp_c(1),controller->robot->dp_c(2));
        break;    

    case INCLINED_DYNA_GAIT:


        controller->t_real = top_time - controller->t0; //update time
        // std::cout<<"Time : "<< controller->t_real <<std::endl;
        // //Dyna Locomotion
        switch (fsm->phase )
        {
        case PH_TARGET:

            //HERE define dynamically the change in inclination
            controller->updateInclination(dp_cmd);

            controller->robot->swingL_id_a = (int)(++loc_i%2); // change swing leg by free gat order
            controller->robot->swingL_id_b = (int) (3 - controller->robot->swingL_id_a); //( case A.{0,3} , case B.{1,2} )
            controller->robot->stanceL_id_a = (int)((loc_i+1)%2);
            controller->robot->stanceL_id_b = (int) (3 - controller->robot->stanceL_id_a);
            phase_id = (int )controller->robot->swingL_id_a;
            controller->setDynamicPhase(); // setphase target etc. 

            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_a],dp_cmd, controller->dz_offset); //HERE: define dynamic dz_offset
            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_b],dp_cmd, controller->dz_offset); //HERE: define dynamic dz_offset

            fsm->phase = PH_SWING;

        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;

            // Commanded velocity
            controller->computeDynamicWeights(controller->incl_a); //controller->incl_a
            Eigen::Vector3d temp_dp_cmd = (controller->t_real < 4.0) ?  (controller->t_real)/4.0*dp_cmd : dp_cmd ;
            controller->dynaErrors(temp_dp_cmd);
            controller->dynaControlSignal();
            controller->doubleInverseTip();
            //CHECK THIS condition
            if ( controller->A_TOUCHED & controller->B_TOUCHED & (controller->robot->leg[(int)controller->robot->swingL_id_a]->wv_leg == controller->robot->leg[0]->w0*Eigen::Vector3d::Ones()) & (controller->robot->leg[(int)controller->robot->swingL_id_b]->wv_leg == controller->robot->leg[0]->w0*Eigen::Vector3d::Ones()) )
            {
                fsm->phase = PH_TARGET;
            }

            break;
        }
        data->save_CoM(controller->t_real, phase_id, 
            controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
            controller->robot->dp_c(0),controller->robot->dp_c(1),controller->robot->dp_c(2));
        break;  
    }

    t_last_c += controller->dt;

    
}
void LocomotionTopLevelControl::init_StaticGait()
{  
    // controller->computeBesierCurve2D(controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing));
    controller->robot->R_c0 = controller->robot->R_c;
    controller->robot->p_c0 = controller->robot->p_c;

    controller->robot->com_p_prev =  controller->robot->p_c0; // set init value to prev CoM position
    controller->robot->R_CoM_prev = controller->robot->R_c0;

    controller->t_phase = 0.0;

    // controller->ii = -1; // counter for bezier execution

}

void LocomotionTopLevelControl::init_topControl()
{
    printf("Default LocomotionTopLevelControl init_topControl \n");
    this->setParams();
    this->wrapper->initConst(); // TODO this is not for Mujoco
}
/* Overload init_topControl Mujoco*/
void LocomotionTopLevelControl::init_topControl(const mjModel* m, mjData* d)
{
    printf("Mujoco LocomotionTopLevelControl init_topControl \n");
    while(d->time < this->fsm->t_S0) // delay before start
    {
        mj_step(m,d);
    }

    /* Maestro - initialize topLevelControl things */
    this->setParams();
    this->wrapper->initConst(); // only for Mujoco
    
}
/* Overload init_topControl Mujoco*/
void LocomotionTopLevelControl::init_topControlDynamic(const mjModel* m, mjData* d)
{
    printf("Mujoco LocomotionTopLevelControl init_topControl \n");
    while(d->time < this->fsm->t_S0) // delay before start
    {
        mj_step(m,d);
    }

    /* Maestro - initialize topLevelControl things */
    this->setParamsDynamic();
    this->wrapper->initConst(); // only for Mujoco


}
/* Overload init_topControl Mujoco*/
void LocomotionTopLevelControl::init_topControlExploration(const mjModel* m, mjData* d)
{
    printf("Mujoco LocomotionTopLevelControl init_topControl \n");
    while(d->time < this->fsm->t_S0) // delay before start
    {
        mj_step(m,d);
    }

    /* Maestro - initialize topLevelControl things */
    this->setParamsExploration();
    this->wrapper->initConst(); // only for Mujoco
    
}
void LocomotionTopLevelControl::computeExploreTask(double top_time)
{
    /*          Compute 
        Case definition at FSM.h    */

    switch (fsm->state)
    {
    /* Configure init stay down pose for robot */
    case S0: 
        controller->PD(controller->robot->leg[0]->sit0);
        
        if(top_time > fsm->t_S1)    // control fsm sequence based on time
            this->fsm->state = S1 ; // next fsm state -> StandUp S1
        break;
    
    /* Configure init stay down pose for robot */
    case S1:

        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
        if(top_time > fsm->t_MC)
        {
            controller->robot->R_c0 = controller->robot->R_c;
            controller->robot->p_c0 = controller->robot->p_c;

            controller->robot->com_p_prev =  controller->robot->p_c0; // set init value to prev CoM position
            controller->robot->R_CoM_prev = controller->robot->R_c0;

            controller->t_phase = 0.0;
            controller->t0 = top_time;  // t0 -> start of tracking
            controller->t_real = 0.0; //time_now - t0;
        
            // controller->ii = -1; // counter for bezier execution

            fsm->state = STATIC_GAIT;
            fsm->phase = PH_TARGET;
        }
        break; // this break might add 0.002 = dt to time
   
    /* Locomotion cases based on gait - Upcoming gaits... */ 
    case STATIC_GAIT:
        
        controller->t_real = top_time - controller->t0; //update time
        // TODO change condition - make it better
        // if( controller->t_phase >= controller->swing_t_slot )// TODO change by forces not time 
        //     fsm->phase = PH_EXPLORE;

        switch (fsm->phase )
        {
        case PH_TARGET:
            
            controller->robot->swingL_id = controller->static_free_gait[(int)(++loc_i%controller->robot->n_legs)]; // change swing leg by free gat order
            controller->setPhaseTarget(); // setphase target etc.
            controller->generateBezier(controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing));
            fsm->phase = PH_SWING;

        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;
            controller->computeWeightsSigmoid();
            controller->errors();
            controller->computeSudoGq();
            controller->signalFc();
            controller->fComputations();
            controller->swingTrajectory();

            break;
        
        case PH_EXPLORE:

            controller->spiralExploration();

            break;
        }

        data->save_exp(controller->t_real,
                        controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
                        controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        controller->bezier_world(0),controller->bezier_world(1),controller->bezier_world(2),
                        controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(0,3), controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(1,3), controller->robot->leg[(int)controller->robot->swingL_id]->g_o_world(2,3),
                        controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                        controller->p_T(0),controller->p_T(1),controller->p_T(2)); 
        break;    
    }

    t_last_c += controller->dt;

}