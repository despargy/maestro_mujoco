#include <Wrapper.h>
/* Constructor*/
Wrapper::Wrapper()
{
    this->category = "None"; // defines Mujoco or Ros/Real
    once = true;
    CHANGE_GAINS = false;
}
Wrapper::Wrapper(std::string category_, Robot* r_)
{
    this->category = category_; // defines Mujoco or Ros/Real
    this->robot = r_; // pass robot pointer to wrapper to update robot/legs profile
    once = true;
    CHANGE_GAINS = false;
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

        // CoM vel x, y, z  [101 - 103]
        robot->vel_x__ = 101 , robot->vel_y__ = 102 , robot->vel_z__ = 103; // Body velocity ids (x,y,z`)

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

                //                 /*    qpos local frame */
                // robot->leg[i]->qpos_hip__    = 0 + 4 ; // 5 6, x,y,z
                // robot->leg[i]->qpos_thigh__  =  ; 
                // robot->leg[i]->qpos_calf__   = 4 ;


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
            for(int i=0; i<robot->n_legs; i++ )
            {              
                robot->leg[i]->cmd_q_hip__   = 0 + 12 + robot->leg[i]->id*6 ; // TODO
                robot->leg[i]->cmd_dq_hip__  = 1 + 12 + robot->leg[i]->id*6 ; // TODO

                robot->leg[i]->cmd_q_thigh__ = 2 + 12 + robot->leg[i]->id*6 ;
                robot->leg[i]->cmd_dq_thigh__= 3 + 12 + robot->leg[i]->id*6 ;

                robot->leg[i]->cmd_q_calf__  = 4 + 12 + robot->leg[i]->id*6 ;
                robot->leg[i]->cmd_dq_calf__ = 5 + 12 + robot->leg[i]->id*6 ;
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

        /* init CoM zero position */
        robot->p_c0(0) = d->sensordata[robot->com_x__] ;
        robot->p_c0(1) = d->sensordata[robot->com_y__] ;
        robot->p_c0(2) = d->sensordata[robot->com_z__] ;    
        robot->com_p_prev =  robot->p_c0; // set init value to prev CoM position

        Eigen::Quaterniond cur_c(d->sensordata[robot->quat_w__], d->sensordata[robot->quat_x__], d->sensordata[robot->quat_y__], d->sensordata[robot->quat_z__]); // init CoM zero orientation 
        cur_c.normalize();
        robot->R_c0 = cur_c.toRotationMatrix(); 
        robot->R_CoM_prev = robot->R_c0;

        once = false;
    }
    // CoM orientation
    Eigen::Quaterniond cur_c(d->sensordata[robot->quat_w__], d->sensordata[robot->quat_x__], d->sensordata[robot->quat_y__], d->sensordata[robot->quat_z__]);
    cur_c.normalize();
    robot->R_c = cur_c.toRotationMatrix(); 

    robot->g_com.block(0,0,3,3) = robot->R_c;
    robot->g_com.block(0,3,3,1) = robot->p_c;

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


        Eigen::Quaterniond Q_i(d->sensordata[robot->leg[i]->quat_w__], d->sensordata[robot->leg[i]->quat_x__], d->sensordata[robot->leg[i]->quat_y__], d->sensordata[robot->leg[i]->quat_z__]);
        Q_i.normalize();
        robot->leg[i]->R_i = Q_i.toRotationMatrix(); 


        robot->leg[i]->g_o.block(0,0,3,3) = robot->leg[i]->R_i; // framepos ref to CoM
        robot->leg[i]->g_o.block(0,3,3,1) = robot->leg[i]->p_i; // framequat ref to CoM

        robot->leg[i]->g_o_world = robot->g_com*robot->leg[i]->g_o; //
    }

    for(int i = 0 ; i <  robot->n_legs ; i++)
    {
        int nv = m->nv;
        double jacp1[3*nv];
        // double point[3] = {d->sensordata[robot->leg[i]->tip_x__],d->sensordata[robot->leg[i]->tip_y__],d->sensordata[robot->leg[i]->tip_z__]};
        // mj_jac(m,d,jacp1,NULL,point,robot->leg[i]->body__);
        double point[3] = {robot->leg[i]->g_o_world(0,3),robot->leg[i]->g_o_world(1,3),robot->leg[i]->g_o_world(2,3)};
        mj_jac(m,d,jacp1,NULL,point,robot->leg[i]->body__);

        // mj_jacSite(m,d,jacp1,NULL,robot->leg[i]->site__);


        robot->leg[i]->J(0,0) = jacp1[6+3*i+0];      robot->leg[i]->J(0,1) = jacp1[6+3*i+1];      robot->leg[i]->J(0,2) = jacp1[6+3*i+2]; 
        robot->leg[i]->J(1,0) = jacp1[6+3*i+0+nv];   robot->leg[i]->J(1,1) = jacp1[6+3*i+1+nv];   robot->leg[i]->J(1,2) = jacp1[6+3*i+2+nv]; 
        robot->leg[i]->J(2,0) = jacp1[6+3*i+0+2*nv]; robot->leg[i]->J(2,1) = jacp1[6+3*i+1+2*nv]; robot->leg[i]->J(2,2) = jacp1[6+3*i+2+2*nv]; 
    
        // Probability of Contact Estimation
        robot->leg[i]->prob_stable = std::fmin(1.0,1.0); // TODO 

    }

}
void Wrapper::update_locomotion(const mjModel* m, mjData* d, double dt)
{

                /* Robot - CoM */
    // CoM position
    robot->p_c(0) = d->sensordata[robot->com_x__] ;
    robot->p_c(1) = d->sensordata[robot->com_y__] ;
    robot->p_c(2) = d->sensordata[robot->com_z__] ;

    // CoM orientation
    Eigen::Quaterniond cur_c(d->sensordata[robot->quat_w__], d->sensordata[robot->quat_x__], d->sensordata[robot->quat_y__], d->sensordata[robot->quat_z__]);
    cur_c.normalize();
    robot->R_c = cur_c.toRotationMatrix(); 

    // CoM velocity
    robot->dp_c(0) = d->sensordata[robot->vel_x__] ;
    robot->dp_c(1) = d->sensordata[robot->vel_y__] ;
    robot->dp_c(2) = d->sensordata[robot->vel_z__] ;

    robot->g_com.block(0,0,3,3) = robot->R_c;
    robot->g_com.block(0,3,3,1) = robot->p_c;

    if (once)
    {

        /* init CoM zero position */
        robot->p_c0(0) = d->sensordata[robot->com_x__] ;
        robot->p_c0(1) = d->sensordata[robot->com_y__] ;
        robot->p_c0(2) = d->sensordata[robot->com_z__] ;    
        robot->com_p_prev =  robot->p_c0; // set init value to prev CoM position

        Eigen::Quaterniond cur_c(d->sensordata[robot->quat_w__], d->sensordata[robot->quat_x__], d->sensordata[robot->quat_y__], d->sensordata[robot->quat_z__]); // init CoM zero orientation 
        cur_c.normalize();
        robot->R_c0 = cur_c.toRotationMatrix(); 
        robot->R_CoM_prev = robot->R_c0;

        once = false;
    }

    // CoM velocity 
    // robot->dCoM_p = get_dp_CoM(robot->com_p_prev, robot->p_c, dt);  //TODO
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

        // tip pose (x,y,z) world frame
        robot->leg[i]->p_i(0) = d->sensordata[robot->leg[i]->tip_x__]; // framepos ref to CoM
        robot->leg[i]->p_i(1) = d->sensordata[robot->leg[i]->tip_y__]; // framepos ref to CoM
        robot->leg[i]->p_i(2) = d->sensordata[robot->leg[i]->tip_z__]; // framepos ref to CoM   

        Eigen::Quaterniond Q_i(d->sensordata[robot->leg[i]->quat_w__], d->sensordata[robot->leg[i]->quat_x__], d->sensordata[robot->leg[i]->quat_y__], d->sensordata[robot->leg[i]->quat_z__]);
        Q_i.normalize();
        robot->leg[i]->R_i = Q_i.toRotationMatrix(); // framequat ref to CoM

        robot->leg[i]->g_o.block(0,0,3,3) = robot->leg[i]->R_i; // framepos ref to CoM
        robot->leg[i]->g_o.block(0,3,3,1) = robot->leg[i]->p_i; // framequat ref to CoM

        robot->leg[i]->g_o_world = robot->g_com*robot->leg[i]->g_o; //


    }

    for(int i = 0 ; i <  robot->n_legs ; i++)
    {
        int nv = m->nv;
        double jacp1[3*nv];
        // double point[3] = {robot->leg[i]->g_o_world(0,3),robot->leg[i]->g_o_world(1,3),robot->leg[i]->g_o_world(2,3)};
        // mj_jac(m,d,jacp1,NULL,point,robot->leg[i]->body__);
        // mj_jacBodyCom(m,d,jacp1,NULL,robot->leg[i]->body__);
        mj_jacSite(m,d,jacp1,NULL,robot->leg[i]->site__);

        robot->leg[i]->J(0,0) = jacp1[6+3*i+0];      robot->leg[i]->J(0,1) = jacp1[6+3*i+1];      robot->leg[i]->J(0,2) = jacp1[6+3*i+2]; 
        robot->leg[i]->J(1,0) = jacp1[6+3*i+0+nv];   robot->leg[i]->J(1,1) = jacp1[6+3*i+1+nv];   robot->leg[i]->J(1,2) = jacp1[6+3*i+2+nv]; 
        robot->leg[i]->J(2,0) = jacp1[6+3*i+0+2*nv]; robot->leg[i]->J(2,1) = jacp1[6+3*i+1+2*nv]; robot->leg[i]->J(2,2) = jacp1[6+3*i+2+2*nv]; 
    
        // Probability of Contact Estimation
        robot->leg[i]->prob_stable = std::fmin(1.0,1.0); // TODO 

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
void Wrapper::send_torque_pos(const mjModel* m, mjData* d)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        d->ctrl[robot->leg[l]->torque_hip__]   = robot->leg[l]->tau(0);
        d->ctrl[robot->leg[l]->torque_thigh__] = robot->leg[l]->tau(1);
        d->ctrl[robot->leg[l]->torque_calf__]  = robot->leg[l]->tau(2);
    }
    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_q_hip__]   = robot->leg[(int) robot->swingL_id]->q_out(0);
    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_q_thigh__] = robot->leg[(int) robot->swingL_id]->q_out(1);
    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_q_calf__]  = robot->leg[(int) robot->swingL_id]->q_out(2);

    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_dq_hip__]   = robot->leg[(int) robot->swingL_id]->dq_out(0);
    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_dq_thigh__] = robot->leg[(int) robot->swingL_id]->dq_out(1);
    d->ctrl[robot->leg[(int) robot->swingL_id]->cmd_dq_calf__]  = robot->leg[(int) robot->swingL_id]->dq_out(2);

}
void Wrapper::send_torque_pos_Dynamic(const mjModel* m, mjData* d, bool A_PD, bool B_PD)
{
    for(int l=0; l< robot->n_legs; l++)
    {
        d->ctrl[robot->leg[l]->torque_hip__]   = robot->leg[l]->tau(0);
        d->ctrl[robot->leg[l]->torque_thigh__] = robot->leg[l]->tau(1);
        d->ctrl[robot->leg[l]->torque_calf__]  = robot->leg[l]->tau(2);
    }

    // if (A_PD)
    // {
        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_hip__]   = robot->leg[(int) robot->swingL_id_a]->q_out(0);
        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_thigh__] = robot->leg[(int) robot->swingL_id_a]->q_out(1);
        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_calf__]  = robot->leg[(int) robot->swingL_id_a]->q_out(2);

        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_hip__]   = robot->leg[(int) robot->swingL_id_a]->dq_out(0);
        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_thigh__] = robot->leg[(int) robot->swingL_id_a]->dq_out(1);
        d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_calf__]  = robot->leg[(int) robot->swingL_id_a]->dq_out(2);
    // }
    // else
    // {
    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_hip__]   = robot->leg[(int) robot->swingL_id_a]->q_out(0);
    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_thigh__] = robot->leg[(int) robot->swingL_id_a]->q_out(1);
    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_q_calf__]  = robot->leg[(int) robot->swingL_id_a]->q_out(2);

    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_hip__]   = 0.0;
    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_thigh__] = 0.0;
    //     d->ctrl[robot->leg[(int) robot->swingL_id_a]->cmd_dq_calf__]  = 0.0;
    // }
    // if (B_PD)
    // {
        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_hip__]   = robot->leg[(int) robot->swingL_id_b]->q_out(0);
        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_thigh__] = robot->leg[(int) robot->swingL_id_b]->q_out(1);
        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_calf__]  = robot->leg[(int) robot->swingL_id_b]->q_out(2);

        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_hip__]   = robot->leg[(int) robot->swingL_id_b]->dq_out(0);
        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_thigh__] = robot->leg[(int) robot->swingL_id_b]->dq_out(1);
        d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_calf__]  = robot->leg[(int) robot->swingL_id_b]->dq_out(2);
    // }
    // else
    // {
    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_hip__]   = robot->leg[(int) robot->swingL_id_b]->q_out(0);
    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_thigh__] = robot->leg[(int) robot->swingL_id_b]->q_out(1);
    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_q_calf__]  = robot->leg[(int) robot->swingL_id_b]->q_out(2);

    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_hip__]   = 0.0;
    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_thigh__] = 0.0;
    //     d->ctrl[robot->leg[(int) robot->swingL_id_b]->cmd_dq_calf__]  = 0.0;
    // }
}
bool Wrapper::change_gains(const mjModel* m, mjData* d, bool A_PD, bool B_PD)
{

    for(int i = 0 ; i < robot->n_legs; i++)
    {
        m->actuator_gainprm[10*robot->leg[i]->cmd_q_hip__+0]    =  0;
        m->actuator_biasprm[10*robot->leg[i]->cmd_q_hip__+1]    = -0;
        // m->actuator_gainprm[10*robot->leg[i]->cmd_dq_hip__+0]   =  1.5;
        // m->actuator_biasprm[10*robot->leg[i]->cmd_dq_hip__+1]   = -1.5;
        m->actuator_gainprm[10*robot->leg[i]->cmd_q_thigh__+0]  =  0;
        m->actuator_biasprm[10*robot->leg[i]->cmd_q_thigh__+1]  = -0;
        // m->actuator_gainprm[10*robot->leg[i]->cmd_dq_thigh__+0] =  1.0;
        // m->actuator_biasprm[10*robot->leg[i]->cmd_dq_thigh__+1] = -1.0;
        m->actuator_gainprm[10*robot->leg[i]->cmd_q_calf__+0]   =  0;
        m->actuator_biasprm[10*robot->leg[i]->cmd_q_calf__+1]   = -0;
        // m->actuator_gainprm[10*robot->leg[i]->cmd_dq_calf__+0]  =  1.0;
        // m->actuator_biasprm[10*robot->leg[i]->cmd_dq_calf__+1]  = -1.0;
    }

    if (A_PD)
    {
        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_hip__+0] =  Kp_hip;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_hip__+1] = -Kp_hip;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_hip__+0] =  Kv_hip;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_hip__+1] = -Kv_hip;

        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_thigh__+0] =  Kp_thing;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_thigh__+1] = -Kp_thing;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_thigh__+0] =  Kv_thing;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_thigh__+1] = -Kv_thing;

        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_calf__+0] =  Kp_calf;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_q_calf__+1] = -Kp_calf;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_calf__+0] =  Kv_calf;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_a]->cmd_dq_calf__+1] = -Kv_calf;
    }
    if(B_PD)
    {
        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_hip__+0] =  Kp_hip;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_hip__+1] = -Kp_hip;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_hip__+0] =  Kv_hip;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_hip__+1] = -Kv_hip;

        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_thigh__+0] =  Kp_thing;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_thigh__+1] = -Kp_thing;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_thigh__+0] =  Kv_thing;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_thigh__+1] = -Kv_thing;

        m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_calf__+0] =  Kp_calf;
        m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_q_calf__+1] = -Kp_calf;

        // m->actuator_gainprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_calf__+0] =  Kv_calf;
        // m->actuator_biasprm[10*robot->leg[(int) robot->swingL_id_b]->cmd_dq_calf__+1] = -Kv_calf;
    }

    return false; // this will change the C 
}
