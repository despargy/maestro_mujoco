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
    loc_i = -1 ;
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

void LocomotionTopLevelControl::compute(double top_time)
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

        if( controller->t_phase >= controller->swing_t_slot ) // TODO change by forces not time 
            fsm->phase = PH_TARGET;

        switch (fsm->phase )
        {
        case PH_TARGET:
            
            controller->robot->swingL_id = controller->static_free_gait[(int)(++loc_i%controller->robot->n_legs)]; // change swing leg by free gat order
            controller->setPhaseTarget(); // setphase target etc.
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

        data->save_loc(controller->t_real,controller->robot->leg[0]->g_o_world(0,3),
                        controller->robot->leg[0]->g_o_world(1,3), controller->robot->leg[0]->g_o_world(2,3),
                        controller->robot->leg[0]->wv_leg(0),controller->robot->leg[1]->wv_leg(0),
                        controller->robot->leg[2]->wv_leg(0), controller->robot->leg[3]->wv_leg(0),
                        controller->d_world_pos(0), controller->d_world_pos(1),controller-> d_world_pos(2),
                        controller->robot->p_c(0),controller->robot->p_c(1),controller->robot->p_c(2),
                        controller->p_T(0),controller->p_T(1),controller->p_T(2)); 
        break;    
    }

    t_last_c += controller->dt;

    
}
void LocomotionTopLevelControl::init_StaticGait()
{  
    controller->computeBesierCurve2D(controller->freq_swing*controller->dt/(1+controller->dt*controller->freq_swing));
    controller->robot->R_c0 = controller->robot->R_c;
    controller->robot->p_c0 = controller->robot->p_c;

    controller->robot->com_p_prev =  controller->robot->p_c0; // set init value to prev CoM position
    controller->robot->R_CoM_prev = controller->robot->R_c0;

    controller->t_phase = 0.0;

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