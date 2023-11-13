#include <TopLevelControl.h>

#ifndef _LOCOMOTIONTOPLEVELCONTROL_H_
#define _LOCOMOTIONTOPLEVELCONTROL_H_




class LocomotionTopLevelControl : public TopLevelControl
{
    public:
        LocomotionController* controller; // LocomotionController pointer obj.

        LocomotionTopLevelControl();
        LocomotionTopLevelControl(std::string category_);
        ~LocomotionTopLevelControl();
        void setParams();
        void compute(double top_time);

};


#endif