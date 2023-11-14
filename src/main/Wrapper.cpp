#include <Wrapper.h>
/* Constructor*/
Wrapper::Wrapper()
{
    this->category = "None"; // defines Mujoco or Ros/Real
    once = true;
}
Wrapper::Wrapper(std::string category_, Robot* r_)
{
    this->category = category_; // defines Mujoco or Ros/Real
    this->robot = r_; // pass robot pointer to wrapper to update robot/legs profile
    once = true;
}
/* De-Constructor*/
Wrapper::~Wrapper(){}
void Wrapper::initConst()
{
    if(category == "Mujoco")
    {
        /* Set conts ids for Robot (CoM)*/
        robot->acc_x__ =  24, robot->acc_y__ = 25 , robot->acc_z__ = 26; // Body acceleration ids
        robot->gyro_x__ = 27, robot->gyro_y__ = 28, robot->gyro_z__ = 29; // Body gyro    ids
        robot->com_x__ = 30 , robot->com_y__ = 31 , robot->com_z__ = 32; // Body position ids (x,y,z`)
        robot->quat_w__ = 33, robot->quat_x__ = 34, robot->quat_y__ = 35, robot->quat_z__ = 36; // Body quaternion ids (w,x,y,z)

        /* Set conts ids for Legs*/
        if (robot->n_legs == 4) // future definition of k parametric leg
        {
            // set name for each leg
            robot->leg[0]->name = "FR_foot";
            robot->leg[1]->name = "FL_foot";
            robot->leg[2]->name = "RR_foot";
            robot->leg[3]->name = "RL_foot";

            // body id per leg
            robot->leg[0]->body__ = 4;
            robot->leg[1]->body__ = 7;
            robot->leg[2]->body__ = 10;
            robot->leg[3]->body__ = 13;

            // tip site per leg
            robot->leg[0]->site__ = 1; 
            robot->leg[1]->site__ = 2;
            robot->leg[2]->site__ = 3;
            robot->leg[3]->site__ = 4;

            for(int i=0; i<robot->n_legs; i++ )
            {
                robot->leg[i]->id = i; // 0, 1, 2, 3

                                /* Sensordata Mujoco*/
                // joint pos and vel    [0-23]
                robot->leg[i]->q_hip__  = 0 + robot->leg[i]->id*3 ; 
                robot->leg[i]->q_thigh__  = 1 + robot->leg[i]->id*3 ; 
                robot->leg[i]->q_calf__  = 2 + robot->leg[i]->id*3;
                
                robot->leg[i]->dq_hip__ = 12 +   robot->leg[i]->id*3 ; 
                robot->leg[i]->dq_thigh__ = 13 + robot->leg[i]->id*3; 
                robot->leg[i]->dq_calf__  = 14 + robot->leg[i]->id*3;

                // foot pos x, y, z      [37 - 64]       3 pos + 4 quat = 7 size per leg
                robot->leg[i]->tip_x__ = 37 + robot->leg[i]->id*7; robot->leg[i]->tip_y__ = 38 + robot->leg[i]->id*7; robot->leg[i]->tip_z__ = 39 + robot->leg[i]->id*7;
                // foot quat w,x,y,z
                robot->leg[i]->quat_w__ = 40 + robot->leg[i]->id*7; robot->leg[i]->quat_x__ = 41 + robot->leg[i]->id*7; robot->leg[i]->quat_y__ = 42 + robot->leg[i]->id*7; robot->leg[i]->quat_z__ = 43 + robot->leg[i]->id*7;
                
                // foot acc and gyro [65 - 88]
                robot->leg[i]->acc_x__ = 65 + robot->leg[i]->id*6, robot->leg[i]->acc_y__ = 66 + robot->leg[i]->id*6, robot->leg[i]->acc_z__  = 67 + robot->leg[i]->id*6; // tip acc
                robot->leg[i]->gyro_x__= 68 + robot->leg[i]->id*6, robot->leg[i]->gyro_y__= 69 + robot->leg[i]->id*6, robot->leg[i]->gyro_z__ = 70 + robot->leg[i]->id*6 ;  // tip acc
                
                // foot forces x, y, z  [89 - 100]
                robot->leg[i]->force_x__ = 89 + robot->leg[i]->id*3; robot->leg[i]->force_y__ = 90 + robot->leg[i]->id*3; robot->leg[i]->force_z__ = 91 + robot->leg[i]->id*3;
            
            
                                /* Ctrl Mujoco*/
                // torque control ids
                robot->leg[i]->torque_hip__   = 0 + robot->leg[i]->id*3 ; 
                robot->leg[i]->torque_thigh__ = 1 + robot->leg[i]->id*3 ; 
                robot->leg[i]->torque_calf__  = 2 + robot->leg[i]->id*3 ;

            }


        }
        // TODO
        else if(category == "Real")
        {
            printf("Real isetConsts TODO \n");

        }
    }
}
void Wrapper::update()
{
    // TODO
    printf("Default update \n");
}
void Wrapper::update(const mjModel* m, mjData* d, double dt)
{

                /* Robot - CoM */
    // CoM position
    robot->p_c(0) = d->sensordata[robot->com_x__] ;
    robot->p_c(1) = d->sensordata[robot->com_y__] ;
    robot->p_c(2) = d->sensordata[robot->com_z__] ;
    if (once)
    {
        robot->com_p_prev = robot->p_c;
        robot->R_CoM_prev = robot->R_c;
        once = false;
    }
    // CoM orientation
    Eigen::Quaterniond cur_c(d->sensordata[robot->quat_w__], d->sensordata[robot->quat_x__], d->sensordata[robot->quat_y__], d->sensordata[robot->quat_z__]);
    cur_c.normalize();
    robot->R_c = cur_c.toRotationMatrix(); 
    // CoM velocity 
    robot->dCoM_p = get_dp_CoM(robot->com_p_prev, robot->p_c, dt);  //TODO
    robot->dR_CoM = get_dR_CoM(robot->R_CoM_prev, robot->R_c, dt);  //TODO
    robot->w_CoM  = scewSymmetricInverse(robot->dR_CoM*robot->R_c.transpose());
    // store current as prev for the next control cycle 
    robot->com_p_prev = robot->p_c;
    robot->R_CoM_prev = robot->R_c;

                /* Legs */
    for(int i = 0 ; i <  robot->n_legs ; i++)
    {
        // q of Joints per leg (hip,thigh,calf)
        robot->leg[i]->q(0) = d->sensordata[robot->leg[i]->q_hip__];
        robot->leg[i]->q(1) = d->sensordata[robot->leg[i]->q_thigh__];
        robot->leg[i]->q(2) = d->sensordata[robot->leg[i]->q_calf__];
        // dq of Joints per leg (hip,thigh,calf)
        robot->leg[i]->dq(0) = d->sensordata[robot->leg[i]->dq_hip__];
        robot->leg[i]->dq(1) = d->sensordata[robot->leg[i]->dq_thigh__];
        robot->leg[i]->dq(2) = d->sensordata[robot->leg[i]->dq_calf__];
        // foot Force on tip 'current' (x,y,z)
        robot->leg[i]->f(0) = d->sensordata[robot->leg[i]->force_x__];
        robot->leg[i]->f(1) = d->sensordata[robot->leg[i]->force_y__];
        robot->leg[i]->f(2) = d->sensordata[robot->leg[i]->force_z__];       

        // tip pose (x,y,z)
        robot->leg[i]->p_i(0) = d->sensordata[robot->leg[i]->tip_x__];
        robot->leg[i]->p_i(1) = d->sensordata[robot->leg[i]->tip_y__];
        robot->leg[i]->p_i(2) = d->sensordata[robot->leg[i]->tip_z__];    


        Eigen::Quaterniond q_i(d->sensordata[robot->leg[i]->quat_w__], d->sensordata[robot->leg[i]->quat_x__], d->sensordata[robot->leg[i]->quat_y__], d->sensordata[robot->leg[i]->quat_z__]);
        q_i.normalize();
        robot->leg[i]->R_i = q_i.toRotationMatrix(); 

    }

    for(int i = 0 ; i <  robot->n_legs ; i++)
    {
        int nv = m->nv;
        double jacp1[3*nv];
        double point[3] = {d->sensordata[robot->leg[i]->tip_x__],d->sensordata[robot->leg[i]->tip_y__],d->sensordata[robot->leg[i]->tip_z__]};
        mj_jac(m,d,jacp1,NULL,point,robot->leg[i]->body__);

        robot->leg[i]->J(0,0) = jacp1[6+3*i+0];      robot->leg[i]->J(0,1) = jacp1[6+3*i+1];      robot->leg[i]->J(0,2) = jacp1[6+3*i+2]; 
        robot->leg[i]->J(1,0) = jacp1[6+3*i+0+nv];   robot->leg[i]->J(1,1) = jacp1[6+3*i+1+nv];   robot->leg[i]->J(1,2) = jacp1[6+3*i+2+nv]; 
        robot->leg[i]->J(2,0) = jacp1[6+3*i+0+2*nv]; robot->leg[i]->J(2,1) = jacp1[6+3*i+1+2*nv]; robot->leg[i]->J(2,2) = jacp1[6+3*i+2+2*nv]; 
    
        // Probability of Contact Estimation
        robot->leg[3]->prob_stable = 1.0; // TODO std::fmin(wrapper->slip[l],1.0);

    }

}
void Wrapper::send()
{
    // TODO send not mujoco
    printf("Default Wrapper \n");
}
// Torque control
void Wrapper::send(const mjModel* m, mjData* d)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        d->ctrl[robot->leg[l]->torque_hip__]   = robot->leg[l]->tau(0);
        d->ctrl[robot->leg[l]->torque_thigh__] = robot->leg[l]->tau(1);
        d->ctrl[robot->leg[l]->torque_calf__]  = robot->leg[l]->tau(2);
    }
}
