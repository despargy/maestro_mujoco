#include <Robot.h>
/* Constructor*/
Robot::Robot()
{
    // robot_kin = ; TODO

    this->R_c.resize(3,3);
    this->R_c0.resize(3,3);
    this->H_c.resize(6,6);
    this->C_c.resize(6,6);
    this->Gq.resize(6,12);
    this->Gq_sudo.resize(12,6);
    // init H_c
    this->H_c.block(0,0,3,3) = this->mass*Eigen::Matrix3d::Identity();
    this->H_c.block(3,3,3,3) = Eigen::Matrix3d::Zero(); // will later be set based on Rc*Ic*Rc.t() 
    // init C_c
    this->C_c.block(0,0,3,3) = Eigen::Matrix3d::Zero();
    this->C_c.block(3,3,3,3) = Eigen::Matrix3d::Zero();
    //init Gq
    this->Gq.block(0,0,3,3) =  Eigen::Matrix3d::Identity();
    this->Gq.block(0,3,3,3) =  Eigen::Matrix3d::Identity();
    this->Gq.block(0,6,3,3) =  Eigen::Matrix3d::Identity();
    this->Gq.block(0,9,3,3) =  Eigen::Matrix3d::Identity();

    this->F_a.resize(12);
    this->F_c.resize(6);
    this->gc.resize(6);

    leg = new Leg *[n_legs];
    for(int l=0;l<n_legs;l++)
        leg[l] = new Leg();   // allocate Leg pointers
  
    vvvv = leg[0]->w0*Eigen::VectorXd::Ones(12); //as Legs wv_leg init
    W_inv = (vvvv.asDiagonal()).inverse();


    this->com_p_prev = Eigen::Vector3d::Zero();
    this->R_CoM_prev = Eigen::Matrix3d::Zero();

    
}
/* De-Constructor*/
Robot::~Robot()
{
    delete leg;
}
