#include <TopLevelControl.h>


#ifndef _TRAJECTORYTOPLEVELCONTROL_H_
#define _TRAJECTORYTOPLEVELCONTROL_H_


class TrajectoryTopLevelControl : public TopLevelControl
{
    public:

        TrajController* controller; // Controller pointer obj.

        TrajectoryTopLevelControl();
        TrajectoryTopLevelControl(std::string category_);
        ~TrajectoryTopLevelControl();
        void setParams();
        void compute(double top_time);


};


#endif