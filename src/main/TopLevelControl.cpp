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
TopLevelControl::~TopLevelControl(){}
void TopLevelControl::init_topControl(){}
/* Overload init_topControl Mujoco*/
void TopLevelControl::init_topControl(const mjModel* m, mjData* d){}
/* Function to initialize variables and/or call other init functions */
void TopLevelControl::setParams(){}
void TopLevelControl::compute(double top_time){}
