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
void TopLevelControl::setParams(){}
void TopLevelControl::compute(double top_time){}
