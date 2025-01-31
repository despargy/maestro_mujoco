#include <Robot.h>
/* Constructor*/
Robot::Robot()
{
    std::cout<<"Constractor Robot"<<std::endl;

    // this->R_c.resize(3,3);
    // this->R_c0.resize(3,3);
    this->H_c.resize(6,6);
    this->C_c.resize(6,6);
    this->Gq.resize(6,12);
    this->Gq_sudo.resize(12,6);
   
    this->H_c.block(0,0,3,3) = this->mass*Eigen::Matrix3d::Identity();  // init H_c
    this->H_c.block(3,3,3,3) = Eigen::Matrix3d::Zero(); // will later be set based on Rc*Ic*Rc.t() 
    
    this->C_c.block(0,0,3,3) = Eigen::Matrix3d::Zero(); // init C_c
    this->C_c.block(3,3,3,3) = Eigen::Matrix3d::Zero();
    
    this->Gq.block(0,0,3,3) =  Eigen::Matrix3d::Identity(); //init Gq
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

    g_com = Eigen::Matrix4d::Identity();
    swingL_id = 0; // do not play role

    theta_d = 0.0;

    if (n_legs==4)
    {
        leg[0]->pros = - 1;
        leg[1]->pros = + 1;
        leg[2]->pros = - 1;
        leg[3]->pros = + 1;

    //     leg[0]->TIP_EXT = Eigen::Vector3d(+l1, -d, 0.019);
    //     leg[2]->TIP_EXT = Eigen::Vector3d(-l2, -d, 0.019);
    //     leg[1]->TIP_EXT = Eigen::Vector3d(+l1, +d, 0.019);
    //     leg[3]->TIP_EXT = Eigen::Vector3d(-l2, +d, 0.019);

    }
}
/* De-Constructor*/
Robot::~Robot()
{
    delete leg;
}
