#include <Leg.h>
/* Constructor*/
Leg::Leg()
{
    std::cout<<"Constractor Leg"<<std::endl;

    n_superV_joints =3;
    q.resize(n_superV_joints);  // joint position
    dq.resize(n_superV_joints);   // joint velocity
    q_out.resize(n_superV_joints);

    J.resize(6, n_superV_joints);
    
    w0 = 50;
    wv_leg = w0*Eigen::Vector3d::Ones(); 
    
    g_0bo_init = Eigen::Matrix4d::Identity();
    g_o = Eigen::Matrix4d::Identity();
    g_o_world = Eigen::Matrix4d::Identity();

    dq_out = Eigen::Vector3d::Zero();
}

/* De-Constructor*/
Leg::~Leg(){}

void Leg::storeInitG()
{
    g_0bo_init = g_o_world;
}
void Leg::initQout()
{
    q_out(0) = q(0);
    q_out(1) = q(1);
    q_out(2) = q(2);

}