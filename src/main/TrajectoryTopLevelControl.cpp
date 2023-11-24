#include <TrajectoryTopLevelControl.h>
/* Construtor */
TrajectoryTopLevelControl::TrajectoryTopLevelControl()
{
    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new TrajController();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(); // com. with Mujoco/ROS
    t_last_c = 0.0;
}
TrajectoryTopLevelControl::TrajectoryTopLevelControl(std::string category_)
{

    /* Allocate memory */ 
    this->fsm = new FSM();
    this->controller = new TrajController();
    this->data = new Data(); // init Data 

    this->wrapper = new Wrapper(category_, controller->robot); // com. with Mujoco/ROS
    t_last_c = 0.0;

}
/* De-Constructor */
TrajectoryTopLevelControl::~TrajectoryTopLevelControl()
{
    /* De-Allocate memory */ 
    delete this->fsm ;
    delete this->controller;
    delete this->data;
    delete this->wrapper;
}
/* Function to initialize variables and/or call other init functions */
void TrajectoryTopLevelControl::setParams()
{
    data->init_save_data(); // initialization function of Data
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

    controller->kp = param_kp_TRACKING;
    controller->kv = param_kv_TRACKING;
    controller->ko = param_ko_TRACKING;
       
    controller->alpha = param_alpha; // same as mujoco .xml
    controller->slope = param_slope; // same as mujoco .xml

}

void TrajectoryTopLevelControl::compute(double top_time)
{
    /*          Compute 
        Case definition at FSM.h    */

    switch (fsm->state)
    {
    /* Configure init stay down pose for robot */
    case S0: 
        controller->PD(this->controller->robot->leg[0]->sit0);
        // control fsm sequence based on time
        if(top_time > fsm->t_S1) 
            this->fsm->state = S1 ; // next fsm state -> StandUp S1
        break;
    
    /* Configure init stay down pose for robot */
    case S1:
        controller->PD_smooth(this->controller->robot->leg[0]->sit1, 1000);
        // control fsm sequence based on time
        if(top_time > fsm->t_MC )
            this->fsm->state = M0 ; // next fsm state -> Init before Main Control MC    
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
    }
    
    t_last_c += controller->dt;
}
void TrajectoryTopLevelControl::init_topControl()
{
    printf("Default init_topControl \n");
    this->setParams();
    this->wrapper->initConst(); // may adjust for non-Mujoco simulator
}
/* Overload init_topControl Mujoco*/
void TrajectoryTopLevelControl::init_topControl(const mjModel* m, mjData* d)
{
    printf("Mujoco TrajectoryTopLevelControl init_topControl \n");
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
    wrapper->robot->R_CoM_prev =  wrapper->robot->R_c0;  // set init value to prev CoM orientation

}