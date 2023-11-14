#include <FSM.h>
/* Constructor*/
FSM::FSM()
{
    this->state = S0;
    this->phase = PH_TARGET; // locomotion

    // SET_TARGET = true;
}
/* De-Constructor*/
FSM::~FSM(){}


