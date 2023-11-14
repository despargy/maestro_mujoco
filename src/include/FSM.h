
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#ifndef _FSM_H_
#define _FSM_H_

/* state cases */
const int S0 = 0; const int S1 = 1; 
const int M0 = 2; const int MC = 3; 
const int TARGET0 = 4; const int TARGET1 = 5; 
const int STATIC_GAIT = 6;

/* when state = LOC: phase can be */
const int PH_TARGET = 0;  // SUPPORT 4 LEGS
const int PH_SWING = 1;


class FSM
{
    public:

        int state, phase;
        // bool SET_TARGET;
        double t_S0 = 5.0, t_S1 = 7.0, t_MC = 10.0, traj_Dur = 27;
        double t_End = t_MC + traj_Dur;
        FSM();
        ~FSM();
};

#endif