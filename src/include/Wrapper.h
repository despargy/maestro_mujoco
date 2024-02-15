
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <mujoco/mujoco.h>
#include <Robot.h>
#include <pce.h>
#include <mujoco_pce_params.h>

#ifndef _WRAPPER_H_
#define _WRAPPER_H_

class Wrapper
{
    public:
        std::string category;
        Robot* robot; // Controller pointer obj.
        double Kp_hip, Kp_thing, Kp_calf;
        double Kv_hip, Kv_thing, Kv_calf;

        pce pce_obj[4]; 

        bool once;
        Wrapper();
        Wrapper(std::string category_, Robot* t_);
        ~Wrapper();
        void initConst();
        void update();
        void update_locomotion(const mjModel* m, mjData* d, double dt);
        void update(const mjModel* m, mjData* d, double dt);
        void send();
        void send(const mjModel* m, mjData* d);
        void send_torque_pos(const mjModel* m, mjData* d);
        bool set_gains(const mjModel* m, mjData* d, bool A_TOUCHED, bool B_TOUCHED);
        void send_torque_pos_Dynamic(const mjModel* m, mjData* d, bool A_PD, bool B_PD);
        void init_PCE();
        void find_params_PCE(); // Only the first time you need to run it
        void update_PCE();
        void setParamsMujocoPCE();

};

#endif