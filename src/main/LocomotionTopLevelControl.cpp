#include <LocomotionTopLevelControl.h>
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
    controller->robot->mass = param_mass; // same as mujoco .xml
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

    controller->robot->z = param_robot_z;
    controller->w_max = param_w_max;

    wrapper->Kp_hip   = param_Kp_hip;
    wrapper->Kp_thing = param_Kp_thing;
    wrapper->Kp_calf  = param_Kp_calf;
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
    controller->ADAPT_A = param_ADAPT_A;
    controller->ADAPT_B = param_ADAPT_B;
    controller->robot->KEEP_CONTROL = param_KEEP_CONTROL;
    controller->robot->mass = param_mass; // same as mujoco .xml
    controller->robot->g_gravity = param_g_gravity; // same as mujoco .xml
    controller->robot->gc << 0,0,controller->robot->mass*controller->robot->g_gravity,0,0,0;
    controller->robot->pbc(0) = param_pbc_x;
    controller->robot->pbc(1) = param_pbc_y;
    controller->robot->pbc(2) = param_pbc_z;

    controller->dt = param_dt; // same as mujoco .xml

    controller->kv = param_kv_DYNA;
    controller->ko = param_ko_DYNA;
    controller->A = param_A_SGaus_DYNA;
    controller->b = param_b_SGaus_DYNA;
    controller->t0_superG =     param_t0_SGaus_DYNA;
    controller->freq_swing =    param_Freq_Swing_DYNA;
    controller->t0_swing =      param_t0_Swing_DYNA;
    controller->t_half_swing =  param_thalf_Swing_DYNA ; 
    controller->swing_t_slot =  param_slot_Swing_DYNA; 

    dp_cmd(0) = param_dp_cmd ;

    controller->robot->z = param_robot_z;
    controller->w_max = param_w_max;
    wrapper->Kp_hip   = param_Kp_hip;
    wrapper->Kp_thing = param_Kp_thing;
    wrapper->Kp_calf  = param_Kp_calf;
    wrapper->Kv_hip   = param_Kv_hip;
    wrapper->Kv_thing = param_Kv_thing;
    wrapper->Kv_calf  = param_Kv_calf;

    controller->c1 = param_c1;
    controller->c2 = param_c2;

    controller->force_thres = param_force_thres;
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
    controller->robot->mass = param_mass; // same as mujoco .xml
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

    controller->robot->z = param_robot_z;

    controller->w_max = param_w_max;

    wrapper->Kp_hip   = param_Kp_hip;
    wrapper->Kp_thing = param_Kp_thing;
    wrapper->Kp_calf  = param_Kp_calf;
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

    /* Configure init stay down pose for robot */
    case S2:

        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
        if(top_time > fsm->t_MC)
        {
            controller->t0 = top_time;  // t0 -> start of tracking
            controller->t_real = 0.0; //time_now - t0;
            controller->t_phase = 0.0;

            controller->robot->R_c0 = controller->robot->R_c;
            controller->robot->p_c0 = controller->robot->p_c;

            controller->robot->com_p_prev =  controller->robot->p_c0; // set init value to prev CoM position
            controller->robot->R_CoM_prev = controller->robot->R_c0;

            controller->step_bez = controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing);

            fsm->state = DYNA_GAIT;
            fsm->phase = PH_TARGET;

            controller->robot->leg[0]->foothold = controller->robot->leg[0]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[1]->foothold = controller->robot->leg[1]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[2]->foothold = controller->robot->leg[2]->g_0bo_init.block(0,3,3,1);
            controller->robot->leg[3]->foothold = controller->robot->leg[3]->g_0bo_init.block(0,3,3,1);
                
        }
        break;
   
    case DYNA_GAIT:

        controller->t_real = top_time - controller->t0; //update time
        // //Dyna Locomotion
        switch (fsm->phase )
        {
        case PH_TARGET:
            
            controller->robot->swingL_id_a = (int)(++loc_i%2); // change swing leg by free gat order
            controller->robot->swingL_id_b = (int) (3 - controller->robot->swingL_id_a); //( case A.{0,3} , case B.{1,2} )
            
            controller->setDynamicPhase(); // setphase target etc. 

            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_a]);
            controller->dynamicBezier(controller->robot->leg[(int)controller->robot->swingL_id_b]);

            fsm->phase = PH_SWING;

    
        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;

            // Commanded velocity
            controller->computeDynamicWeights();
            controller->dynaErrors(dp_cmd);
            controller->dynaControlSignal();
            controller->doubleInverseTip();
            if ( controller->A_TOUCHED & controller->B_TOUCHED & (controller->robot->leg[(int)controller->robot->swingL_id_a]->wv_leg == 50*Eigen::Vector3d::Ones()) & (controller->robot->leg[(int)controller->robot->swingL_id_b]->wv_leg == 50*Eigen::Vector3d::Ones()) )
            {
                fsm->phase = PH_TARGET;
            }

            break;
        }

        data->save_dyna(controller->t_real,
                        controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                        controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
                        controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        
                        controller->robot->leg[0]->foothold(0),controller->robot->leg[0]->foothold(1),controller->robot->leg[0]->foothold(2),
                        controller->robot->leg[1]->foothold(0),controller->robot->leg[1]->foothold(1),controller->robot->leg[1]->foothold(2),
                        controller->robot->leg[2]->foothold(0),controller->robot->leg[2]->foothold(1),controller->robot->leg[2]->foothold(2),
                        controller->robot->leg[3]->foothold(0),controller->robot->leg[3]->foothold(1),controller->robot->leg[3]->foothold(2),

                        controller->robot->leg[0]->g_o_world(0,3),           controller->robot->leg[0]->g_o_world(1,3),           controller->robot->leg[0]->g_o_world(2,3),
                        controller->robot->leg[1]->g_o_world(0,3),           controller->robot->leg[1]->g_o_world(1,3),           controller->robot->leg[1]->g_o_world(2,3),
                        controller->robot->leg[2]->g_o_world(0,3),           controller->robot->leg[2]->g_o_world(1,3),           controller->robot->leg[2]->g_o_world(2,3),
                        controller->robot->leg[3]->g_o_world(0,3),           controller->robot->leg[3]->g_o_world(1,3),           controller->robot->leg[3]->g_o_world(2,3),
                        
                        // controller->f_applied_a(0), controller->f_applied_a(1), controller->f_applied_a(2),
                        // controller->f_applied_b(0), controller->f_applied_b(1), controller->f_applied_b(2)

                        controller->bezier_world_a(0), controller->bezier_world_a(1), controller->bezier_world_a(2),
                        controller->bezier_world_b(0), controller->bezier_world_b(1), controller->bezier_world_b(2)

                        
                        );

                        

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