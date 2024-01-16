#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>

Eigen::Vector3d generateBezier()
{
    // dot_bCurveX.empty();
    // dot_bCurveY.empty();
    // dot_bCurveZ.empty();
    // bCurveX.empty();
    // bCurveY.empty();
    // bCurveZ.empty();
    double dt = 0.002;
    double freq_swing = 2.0;
    double step = freq_swing*dt/(1+dt*freq_swing);
    // std::cout<<step<<std::endl;
    // std::cout<<1/step<<std::endl;

    Eigen::Vector3d ofset = Eigen::Vector3d(0.05, 0.0, 0.02);
    Eigen::Vector3d p0 = Eigen::Vector3d(0.147791, -0.128676, 0.0192237);

    Eigen::Vector3d p3 = p0 + ofset;
    Eigen::Vector3d p1, p2;
    p1(0) = p0(0) + 0.5*ofset(0);   p2(0) = p0(0) + 0.8*ofset(0);
    p1(1) = p0(1) + 0*ofset(1);     p2(1) = p0(1) + 0*ofset(1);
    p1(2) = p0(2) + 2.5*ofset(2);   p2(2) = p0(2) + 2.8*ofset(2);

    std::vector<double> xX{p0(0), p1(0), p2(0), p3(0)}; 
    std::vector<double> yY{p0(1), p1(1), p2(1), p3(1)}; 
    std::vector<double> zZ{p0(2), p1(2), p2(2), p3(2)};

    double bCurveXt, dot_bCurveXt;
    double bCurveYt, dot_bCurveYt;
    double bCurveZt, dot_bCurveZt;
    for (double t = 0.0; t <= 1; t += step)
    {
        bCurveXt = std::pow((1 - t), 3) * xX[0] + 3 * std::pow((1 - t), 2) * t * xX[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * xX[2] + std::pow(t, 3) * xX[3];
        bCurveYt = std::pow((1 - t), 3) * yY[0] + 3 * std::pow((1 - t), 2) * t * yY[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * yY[2] + std::pow(t, 3) * yY[3];
        bCurveZt = std::pow((1 - t), 3) * zZ[0] + 3 * std::pow((1 - t), 2) * t * zZ[1] + 3 * std::pow((1 - t), 1) * std::pow(t, 2) * zZ[2] + std::pow(t, 3) * zZ[3];
        // bCurveX.push_back(bCurveXt);
        // bCurveY.push_back(bCurveYt);
        // bCurveZ.push_back(bCurveZt);
        dot_bCurveXt = 3 * std::pow((1 - t), 2) *(xX[1] - xX[0])  + 6 * (1 - t) * t * (xX[2] - xX[1]) + 3 * std::pow(t, 2) * ( xX[3] - xX[2] );
        dot_bCurveYt = 3 * std::pow((1 - t), 2) *(yY[1] - yY[0])  + 6 * (1 - t) * t * (yY[2] - yY[1]) + 3 * std::pow(t, 2) * ( yY[3] - yY[2] );
        dot_bCurveZt = 3 * std::pow((1 - t), 2) *(zZ[1] - zZ[0])  + 6 * (1 - t) * t * (zZ[2] - zZ[1]) + 3 * std::pow(t, 2) * ( zZ[3] - zZ[2] );
        // dot_bCurveX.push_back(dot_bCurveXt);
        // dot_bCurveY.push_back(dot_bCurveYt);
        // dot_bCurveZ.push_back(dot_bCurveZt);

        std::cout<<t<<" "<<bCurveXt<<" "<<bCurveYt<<" "<<bCurveZt<<" "<<std::endl;

    }
    return p3;
}

int main() 
{


    Eigen::Vector3d target = generateBezier();
    std::cout<<1.0<<" "<<target(0)<<" "<<target(1)<<" "<<target(2)<<" "<<std::endl;

    return 0;
}

