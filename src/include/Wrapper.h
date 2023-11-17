
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <mujoco/mujoco.h>
#include <Robot.h>

#ifndef _WRAPPER_H_
#define _WRAPPER_H_

class Wrapper
{
    public:
        std::string category;

        Robot* robot; // Controller pointer obj.
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
        bool change_gains(const mjModel* m, mjData* d);

};

#endif