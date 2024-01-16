#include <TopLevelControl.h>

#ifndef _LOCOMOTIONTOPLEVELCONTROL_H_
#define _LOCOMOTIONTOPLEVELCONTROL_H_




class LocomotionTopLevelControl : public TopLevelControl
{
    public:
        LocomotionController* controller; // LocomotionController pointer obj.

        int loc_i;
        bool DYNA_LOCO;
        Eigen::Vector3d dp_cmd;
        LocomotionTopLevelControl();
        LocomotionTopLevelControl(std::string category_);
        ~LocomotionTopLevelControl();
        void setParams();
        void setParamsExploration();
        void setParamsDynamic();
        void compute(double top_time);
        void computeExploreTask(double top_time);    
        void computeDynamic(double top_time);    

        void init_topControl();
        void init_topControl(const mjModel* m, mjData* d);
        void init_topControlDynamic(const mjModel* m, mjData* d);
        void init_topControlExploration(const mjModel* m, mjData* d);
        void init_StaticGait();
};


#endif