#include <string.h>
#include <iostream>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <stdio.h>
// #include <eigen_conversions/eigen_msg.h>
// #include <eigen_conversions/eigen_kdl.h>
#include <Eigen/Dense>

#include <bits/stdc++.h>


#ifndef _MATH_H_
#define _MATH_H_


Eigen::Matrix3d scewSymmetric(Eigen::Vector3d t);
Eigen::Vector3d scewSymmetricInverse(Eigen::Matrix3d m);
Eigen::Vector3d deriv_RcRdTwd(Eigen::Vector3d RcRdTwd_prev,Eigen::Vector3d RcRdTwd_cur, double dt);
Eigen::Vector3d get_dp_CoM(Eigen::Vector3d com_p_prev,Eigen::Vector3d com_p_cur, double dt);
Eigen::Matrix3d get_dR_CoM(Eigen::Matrix3d R_CoM_prev,Eigen::Matrix3d R_CoM_cur, double dt);
std::pair<double, double> find_Centroid(std::vector<std::pair<double, double> >& v);
double sigmoid(double t, double c1, double c2);
double superGaussian(double A,double b,double r,double d, double n);

#endif