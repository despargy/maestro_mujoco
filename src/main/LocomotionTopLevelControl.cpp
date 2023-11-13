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
    data->init_save_data(); // initialization function of Data
    controller->loop_index = 0; // is used to save data in specific loop frequency
    
    LOCOMOTION_MODE = true;
    TARGET_MODE = param_TARGET_MODE;

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
        // control fsm sequence based on time   
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
   
    case TARGET0:
        controller->initTarget(top_time);
        this->fsm->state = TARGET1 ;
    case TARGET1:
        controller->ReachTarget(top_time);
        data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                controller->e_p(0),controller->e_p(1),controller->e_p(2),
                controller->p_T(0),controller->p_T(1),controller->p_T(2),
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

        
        }
        //reach target
        controller->ReachTarget(top_time);
        data->save(controller->t_real,controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
            controller->e_p(0),controller->e_p(1),controller->e_p(2),
            controller->p_T(0),controller->p_T(1),controller->p_T(2),
            controller->e_o(0),controller->e_o(1),controller->e_o(2)); 
        break;    
    }
    
    t_last_c += controller->dt;
}
