
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
const int LOC = 6;

/* when state = LOC: phase can be */
const int LOC_0 = 0;  // SUPPORT 4 LEGS
const int LOC_1 = 1; const int SWING_1 = 101; 
const int LOC_2 = 2; const int SWING_2 = 102; 
const int LOC_3 = 3; const int SWING_3 = 103; 
const int LOC_4 = 4; const int SWING_4 = 104; 

class FSM
{
    public:

        int state, phase;
        bool SET_TARGET;
        double t_S0 = 5.0, t_S1 = 7.0, t_MC = 10.0, traj_Dur = 27;
        double t_End = t_MC + traj_Dur;
        FSM();
        ~FSM();
};

#endif