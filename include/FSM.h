
#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

#ifndef _FSM_H_
#define _FSM_H_

/* state cases */
// S0 for sit down pose, S1 for stand up pose before static gait, S2 for standup pose before dynamic

const int S0 = 0; const int S1 = 1; const int S2 = 8; const int S3 = 9;
const int M0 = 2; const int MC = 3; 

const int TARGET0 = 4; const int TARGET1 = 5; 
const int STATIC_GAIT = 6;
const int DYNA_GAIT = 7;
const int INCLINED_DYNA_GAIT = 10;

/* when state = LOC: phase can be */
const int PH_TARGET = 0;  // SUPPORT 4 LEGS
const int PH_SWING = 1;
const int PH_EXPLORE = 2; 

class FSM
{
    public:

        int state, phase;
        // bool SET_TARGET;
        // !!NOTE: t_MC -  t_S3 > batch_size*dt seconds
        double t_S0 = 5.0, t_S1 = 7.0, t_S3 = 10.0, t_MC = 13.0, traj_Dur = 27;
        double t_End = t_MC + traj_Dur;
        FSM();
        ~FSM();
};

#endif