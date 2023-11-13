#include <TopLevelControl.h>
/* Construtor */
TopLevelControl::TopLevelControl()
{
    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new Controller();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(); // com. with Mujoco/ROS
    t_last_c = 0.0;
}
TopLevelControl::TopLevelControl(std::string category_)
{

    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new Controller();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(category_, controller->robot); // com. with Mujoco/ROS
    t_last_c = 0.0;

}
/* De-Constructor */
TopLevelControl::~TopLevelControl()
{
    /* De-Allocate memory */ 
    delete this->fsm ;
    delete this->controller;
    delete this->data;
    delete this->wrapper;
}
void TopLevelControl::init_topControl()
{
    printf("Default init_topControl \n");
    // TODO usleep this->fsm->t_S0  
    this->setParams();
    this->wrapper->initConst(); // TODO this is not for Mujoco

}
/* Overload init_topControl Mujoco*/
void TopLevelControl::init_topControl(const mjModel* m, mjData* d)
{
    printf("Mujoco init_topControl \n");
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
/* Function to initialize variables and/or call other init functions */
void TopLevelControl::setParams()
{
    data->init_save_data(); // initialization function of Data
    controller->loop_index = 0; // is used to save data in specific loop frequency
    
    LOCOMOTION_MODE = param_LOCOMOTION_MODE;
    TARGET_MODE = param_TARGET_MODE;
    TRACKING_MODE = param_TRACKING_MODE ;

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

    if (TRACKING_MODE)
    {
        controller->kp = param_kp_TRACKING;
        controller->kv = param_kv_TRACKING;
        controller->ko = param_ko_TRACKING;
    }
    else
    {
        controller->kp = param_kp_TARGET;
        controller->kv = param_kv_TARGET;
        controller->ko = param_ko_TARGET;
        controller->ki = param_ki_TARGET;

    }         

    controller->alpha = param_alpha; // same as mujoco .xml
    controller->slope = param_slope; // same as mujoco .xml

    controller->thres_r = param_thres_r; 
    controller->sat_force = param_sat_force; 

}

void TopLevelControl::compute(double top_time)
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
        // control fsm sequence based on time
        if(top_time > fsm->t_MC & TRACKING_MODE)
            this->fsm->state = M0 ; // next fsm state -> Init before Main Control MC    
        if(top_time > fsm->t_MC & TARGET_MODE)
            this->fsm->state = TARGET0 ; // next fsm state -> Init before Target Reaching  
        if(top_time > fsm->t_MC & LOCOMOTION_MODE)
        {
            // t0 -> start of tracking
            controller->t0 = top_time;
            controller->t_real = 0.0; //time_now - t0;
            this->fsm->state = LOC ; // next fsm state -> Init before Target Reaching            
        }
        break;
    /* Initialize Tracking Mode*/
    case M0:
        controller->initTracking(top_time); // do not add break, we want callBack to be called after
        this->fsm->state = MC ; 

    /* Trajectory Tracking Mode*/
    case MC:
        controller->Tracking(top_time); // (Adaptive) Trajectory Tracking - Control Callback
        data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                controller->e_p(0),controller->e_p(1),controller->e_p(2),
                controller->traj->p_d(0),controller->traj->p_d(1),controller->traj->p_d(2),
                controller->e_o(0),controller->e_o(1),controller->e_o(2));
        break;
    case TARGET0:
        controller->initTarget(top_time);
        this->fsm->state = TARGET1 ;
    case TARGET1:
        controller->ReachTarget(top_time);
        data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                controller->e_p(0),controller->e_p(1),controller->e_p(2),
                controller->traj->p_T(0),controller->traj->p_T(1),controller->traj->p_T(2),
                controller->e_o(0),controller->e_o(1),controller->e_o(2));        
        break;
 
    /* Locomotion cases*/
    case LOC:

        if (fsm->SET_TARGET) //set target
        {
            controller->setPhaseTarget(fsm->phase, top_time);
            fsm->SET_TARGET = false;
        }

        //phase 0, 4 support
        switch (fsm->phase)
        {
        case LOC_0:
    
            std::cout<< "LOC_0 "<< std::endl;
            controller->swing_leg = -1; // no excisting swing leg, 4 legs support
            break;

        case LOC_1:
    
            std::cout<< "LOC_1 "<< std::endl;
            controller->swing_leg = 0; // FR swing leg
            break;

        // case LOC_2:
    
        //     std::cout<< "LOC_2 "<< std::endl;
        //     controller->swing_leg = 3; // RL swing leg
        //     break;

        // case LOC_3:
    
        //     std::cout<< "LOC_3 "<< std::endl;
        //     controller->swing_leg = 1; //  FL swing leg
        //     break;

        // case LOC_4:
    
        //     std::cout<< "LOC_4 "<< std::endl;
        //     controller->swing_leg = 2; // RR swing leg
        //     break;
            
        }
        //reach target
        controller->ReachTarget(top_time);
        data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
            controller->e_p(0),controller->e_p(1),controller->e_p(2),
            controller->traj->p_T(0),controller->traj->p_T(1),controller->traj->p_T(2),
            controller->e_o(0),controller->e_o(1),controller->e_o(2)); 
        break;    
    }
    
    t_last_c += controller->dt;
}
