#include <FSM.h>
/* Constructor*/
FSM::FSM()
{
    this->state = S0;
    this->phase = LOC_1;

    SET_TARGET = true;
}
/* De-Constructor*/
FSM::~FSM(){}


