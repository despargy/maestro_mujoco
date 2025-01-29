#include <Controller.h>
/* Constructor*/
Controller::Controller(){}
/* De-Constructor*/
Controller::~Controller(){}
void Controller::PD(double* target){}
void Controller::PD_smooth(double* target, double smooth){ }
void Controller::positionError(){}
void Controller::velocityError(){}
void Controller::updateControlLaw(){}
void Controller::computeSudoGq(){}
void Controller::computeWeights(){}
void Controller::fComputations(){}

