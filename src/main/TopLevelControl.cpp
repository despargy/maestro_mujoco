#include <TopLevelControl.h>
/* Construtor */
TopLevelControl::TopLevelControl()
{

}
TopLevelControl::TopLevelControl(std::string category_)
{

}
/* De-Constructor */
TopLevelControl::~TopLevelControl(){}
void TopLevelControl::init_topControl(){}
/* Overload init_topControl Mujoco*/
void TopLevelControl::init_topControl(const mjModel* m, mjData* d){}
/* Function to initialize variables and/or call other init functions */
void TopLevelControl::setParams(){}
void TopLevelControl::compute(double top_time){}
