#include <FSM.h>
#include <Controller.h>
#include <Data.h>
#include <Wrapper.h>
#include <params.h>

#ifndef _TOPLEVELCONTROL_H_
#define _TOPLEVELCONTROL_H_


class TopLevelControl
{
    public:
        FSM* fsm; // Finite State Machine pointer obj.
        Controller* controller; // Controller pointer obj.
        Data* data;
        Wrapper* wrapper;
        double t_last_c ;
        bool LOCOMOTION_MODE, TARGET_MODE, TRACKING_MODE;
        bool goal_ach;
        
        TopLevelControl();
        TopLevelControl(std::string category_);
        ~TopLevelControl();
        void init_topControl();
        void init_topControl(const mjModel* m, mjData* d);
        void setParams();
        void compute(double top_time);


};

#endif