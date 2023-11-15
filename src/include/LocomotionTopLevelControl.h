#include <TopLevelControl.h>

#ifndef _LOCOMOTIONTOPLEVELCONTROL_H_
#define _LOCOMOTIONTOPLEVELCONTROL_H_




class LocomotionTopLevelControl : public TopLevelControl
{
    public:
        LocomotionController* controller; // LocomotionController pointer obj.

        int loc_i;
        
        LocomotionTopLevelControl();
        LocomotionTopLevelControl(std::string category_);
        ~LocomotionTopLevelControl();
        void setParams();
        void compute(double top_time);
        void computeTargetTask(double top_time);    
        void init_topControl();
        void init_topControl(const mjModel* m, mjData* d);
        void init_StaticGait();

};


#endif