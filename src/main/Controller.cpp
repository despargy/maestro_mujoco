#include <Controller.h>
/* Constructor*/
Controller::Controller(){}
/* De-Constructor*/
Controller::~Controller(){}
void Controller::PD(double* target)
{
    // for(int l=0; l< robot->n_legs; l++)
    // {
    //     robot->leg[l]->tau(0) = -100*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
    //     robot->leg[l]->tau(1) = -100*(robot->leg[l]->q(1) - target[1])-10*robot->leg[l]->dq(1);
    //     robot->leg[l]->tau(2) = -100*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    // }
}
void Controller::PD_smooth(double* target, double smooth)
{   
    // double percent = (double)loop_index/smooth;
    // for(int l=0; l< robot->n_legs; l++)
    // {
    //     robot->leg[l]->tau(0) = -100*percent*(robot->leg[l]->q(0) - target[0]) -10*robot->leg[l]->dq(0);
    //     robot->leg[l]->tau(1) = -100*percent*(robot->leg[l]->q(1) - target[1]) -10*robot->leg[l]->dq(1);
    //     robot->leg[l]->tau(2) = -100*percent*(robot->leg[l]->q(2) - target[2]) -10*robot->leg[l]->dq(2);
    // }
    // loop_index += 1;
}

void Controller::positionError(){}
void Controller::velocityError(){}
void Controller::updateControlLaw(){}
void Controller::computeSudoGq(){}
void Controller::computeWeights(){}
void Controller::fComputations(){}

