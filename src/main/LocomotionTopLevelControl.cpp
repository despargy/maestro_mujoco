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

}
LocomotionTopLevelControl::LocomotionTopLevelControl(std::string category_)
{

    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new LocomotionController();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(category_, controller->robot); // com. with Mujoco/ROS
    t_last_c = 0.0;

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
    
    // get from params.h
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

    controller->kp = param_kp_TARGET;
    controller->kv = param_kv_TARGET;
    controller->ko = param_ko_TARGET;
    controller->ki = param_ki_TARGET;

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
        // control fsm sequence based on time
        if(top_time > fsm->t_S1) 
            this->fsm->state = S1 ; // next fsm state -> StandUp S1
        break;
    
    /* Configure init stay down pose for robot */
    case S1:
        controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
        if(top_time > fsm->t_MC)
        {
            // t0 -> start of tracking
            controller->t0 = top_time;
            controller->t_real = 0.0; //time_now - t0;
            this->fsm->state = STATIC_GAIT ; // next fsm state -> Init before Target Reaching
        }
        break;
   
    /* Locomotion cases based on gait - Upcoming gaits... */ 
    case STATIC_GAIT:
        
        controller->t_real = top_time - controller->t0; //update time

        if( controller->t_phase > 5.0 ) // TODO change by forces not time
        {
            fsm->phase = PH_TARGET;
            std::cout<<"phase = PH_TARGET"<<std::endl;
            std::cout<< controller->t_phase<<std::endl;

        }
        switch (fsm->phase )
        {

        case PH_TARGET:
            
            // change swing leg by free gat order, circle using mod and counter
            controller->robot->swingL_id = controller->static_free_gait[(int)(++loc_i%controller->robot->n_legs)];
            // set all q_out as zeros  
            controller->robot->leg[(int) controller->robot->swingL_id]->initQout(); 

            // setphase target etc.
            controller->setPhaseTarget();
            std::cout<< "time:"<<controller->t_real<< std::endl;
            std::cout<< "swing:"<<controller->robot->swingL_id<< std::endl;

            fsm->phase = PH_SWING;

        case PH_SWING:

            controller->t_phase = controller->t_real - controller->t0_phase;
            // std::cout<< "swing"<< std::endl;
            break;
            // controller->swing_leg = -1; // no excisting swing leg, 4 legs support
        }
        //reach target
        // controller->ReachTarget(top_time);
        // data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
        //     controller->e_p(0),controller->e_p(1),controller->e_p(2),
        //     controller->p_T(0),controller->p_T(1),controller->p_T(2),
        //     controller->e_o(0),controller->e_o(1),controller->e_o(2)); 
        break;    
    }
    
    t_last_c += controller->dt;
}

/* ************************* Target task *************************** */
void LocomotionTopLevelControl::computeTargetTask(double top_time)
{
    // /*          Compute 
    //     Case definition at FSM.h    */

    // switch (fsm->state)
    // {
    // /* Configure init stay down pose for robot */
    // case S0: 
    //     controller->PD(controller->robot->leg[0]->sit0);
    //     // control fsm sequence based on time
    //     if(top_time > fsm->t_S1) 
    //         this->fsm->state = S1 ; // next fsm state -> StandUp S1
    //     break;
    
    // /* Configure init stay down pose for robot */
    // case S1:
    //     controller->PD_smooth(controller->robot->leg[0]->sit1, 1000);
    //     // control fsm sequence based on time  , TARGET_MODE
    //     if(top_time > fsm->t_MC)
    //         this->fsm->state = TARGET0 ; // next fsm state -> Init before Target Reaching  
    //     break;
   
    // case TARGET0:
    //     controller->initTarget(top_time);
    //     this->fsm->state = TARGET1 ;
    // case TARGET1:
    //     controller->ReachTarget(top_time);
    //     data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
    //             controller->e_p(0),controller->e_p(1),controller->e_p(2),
    //             controller->p_T(0),controller->p_T(1),controller->p_T(2),
    //             controller->e_o(0),controller->e_o(1),controller->e_o(2));        
    //     break;
 
    // }
    
    // t_last_c += controller->dt;
}

void LocomotionTopLevelControl::init_topControl()
{
    printf("Default LocomotionTopLevelControl init_topControl \n");
    // TODO usleep this->fsm->t_S0  
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

    // init CoM zero position 
    wrapper->robot->p_c0(0) = d->sensordata[wrapper->robot->com_x__] ;
    wrapper->robot->p_c0(1) = d->sensordata[wrapper->robot->com_y__] ;
    wrapper->robot->p_c0(2) = d->sensordata[wrapper->robot->com_z__] ;
    // set init value to prev CoM position
    wrapper->robot->com_p_prev =  wrapper->robot->p_c0; 

    // init CoM zero orientation 
    Eigen::Quaterniond cur_c(d->sensordata[wrapper->robot->quat_w__], d->sensordata[wrapper->robot->quat_x__], d->sensordata[wrapper->robot->quat_y__], d->sensordata[wrapper->robot->quat_z__]);
    cur_c.normalize();
    wrapper->robot->R_c0 = cur_c.toRotationMatrix(); 
    // set init value to prev CoM orientation
    wrapper->robot->R_CoM_prev =  wrapper->robot->R_c0; 

}