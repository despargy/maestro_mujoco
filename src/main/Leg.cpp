#include <Leg.h>
/* Constructor*/
Leg::Leg()
{
    n_superV_joints =3;
    q.resize(n_superV_joints);  // joint position
    dq.resize(n_superV_joints);   // joint velocity

    J.resize(6, n_superV_joints);
    
    w0 = 50;
    wv_leg = w0*Eigen::Vector3d::Ones(); 
     
}
// Leg::Leg(KDL::Tree robot_kin)
// {
//     n_superV_joints =3;
//     q.resize(n_superV_joints);  // joint position
//     dq.resize(n_superV_joints);   // joint velocity

//     if (!robot_kin.getChain("base", name, kdl_chain)) 
//     {
//         printf("Could not initialize chain object");
//     }
//     kdl_solver.reset(new KDL::ChainJntToJacSolver(kdl_chain));
//     kdl_solver_pos.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
//     jacobian_kdl.resize(n_superV_joints);

//     J.resize(6, n_superV_joints);
    
//     w0 = 50;
//     wv_leg = w0*Eigen::Vector3d::Ones();  

// }
/* De-Constructor*/
Leg::~Leg(){}
// void Leg::kdlSolver()
// {
//     if ( !(kdl_solver->JntToJac(q, jacobian_kdl) >=0 ))
//         printf("One leg unexpected Jacobian solution leg, '%d'", id);
//     kdl_solver_pos->JntToCart(q,p_frame);
//     J = jacobian_kdl.data; // jac KDL to Eigen 


//     // TODO 
//     // tf::transformKDLToEigen(p_frame, p); // p to eigen Affine
// }