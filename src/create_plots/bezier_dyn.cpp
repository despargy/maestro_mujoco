#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>


double sigmoid(double t, double c1, double c2)
{
    return 1/(1 + exp( -c1*(t-c2)) );
}
double der_sigmoid(double t, double c1, double c2)
{
    return exp( -c1*(t-c2))/std::pow( (1 + exp( -c1*(t-c2)) ), 2);
} 
double inverse_sigmoid(double s_t, double c1, double c2)
{
    // returns the "Virutal time" of the current sigmoid value s_t
    return (1/c1*log((1-s_t)/s_t)+c2);
}

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
    Eigen::Vector3d p1, p2, p12;
    p1(0) = p0(0) + 0.5*ofset(0);   p2(0) = p0(0) + 0.8*ofset(0);
    p1(1) = p0(1) + 0*ofset(1);     p2(1) = p0(1) + 0*ofset(1);
    // p1(2) = p0(2) + 0.8*0.02;   p2(2) = p0(2) + 0.9*0.02; //2.5*ofset(2) ,  2.8*ofset(2)
    p1(2) = p0(2) + 2/3*(p3(2) - p0(2));   p2(2) = p3(2) - 2/3*(p3(2) - p0(2)); //2.5*ofset(2) ,  2.8*ofset(2)
    
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
        // bCurveZt = std::pow((1 - t), 2) * zZ[0] + 2*(1 - t) * t * zZ[1] + std::pow(t, 2) * zZ[2];
        // bCurveZt = sin(2*M_PI*freq_swing*t/4);
        // bCurveX.push_back(bCurveXt);
        // bCurveY.push_back(bCurveYt);
        // bCurveZ.push_back(bCurveZt);
        dot_bCurveXt = 3 * std::pow((1 - t), 2) *(xX[1] - xX[0])  + 6 * (1 - t) * t * (xX[2] - xX[1]) + 3 * std::pow(t, 2) * ( xX[3] - xX[2] );
        dot_bCurveYt = 3 * std::pow((1 - t), 2) *(yY[1] - yY[0])  + 6 * (1 - t) * t * (yY[2] - yY[1]) + 3 * std::pow(t, 2) * ( yY[3] - yY[2] );
        dot_bCurveZt = 3 * std::pow((1 - t), 2) *(zZ[1] - zZ[0])  + 6 * (1 - t) * t * (zZ[2] - zZ[1]) + 3 * std::pow(t, 2) * ( zZ[3] - zZ[2] );
        // dot_bCurveZt = 2 * t *(zZ[0] - 2*zZ[1] + zZ[2])  + 2 * (zZ[1] - zZ[0]) ;
        // dot_bCurveZt = cos(2*M_PI*freq_swing*t/4);

        // dot_bCurveX.push_back(dot_bCurveXt);
        // dot_bCurveY.push_back(dot_bCurveYt);
        // dot_bCurveZ.push_back(dot_bCurveZt);

        std::cout<<t<<" "<<bCurveXt<<" "<<bCurveYt<<" "<<bCurveZt<<" "<<dot_bCurveXt<<" "<<dot_bCurveYt<<" "<<dot_bCurveZt<<std::endl;

    }
    return p3;
}

Eigen::Vector3d generateBezier(Eigen::Vector3d p0, Eigen::Vector3d p3, int i)
{

    double dt = 0.002;
    double freq_swing = 2.0;
    double step = freq_swing*dt/(1+dt*freq_swing);


    Eigen::Vector3d ofset = p3 - p0;//Eigen::Vector3d(0.05, 0.0, 0.02);
    ofset(2) = 0.02;
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
    for (double t = 0.0 ; t <= 1 ; t += step)
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

        std::cout<<t + i<<" "<<bCurveXt<<" "<<bCurveYt<<" "<<bCurveZt<<" "<<std::endl;

    }
    return p3;
}
void generateSteps()
{
    Eigen::Vector3d p0(0.147791, -0.128676 ,0.0192237);
    Eigen::Vector3d p3(-0.225242, 0.127784, 0.019249);
    Eigen::Vector3d p1(0.149468, 0.128349 ,0.0184109);
    Eigen::Vector3d p2(-0.223524, -0.128083 ,0.0184252);

    std::vector<Eigen::Vector3d> steps_FR, steps_RR, steps_FL, steps_RL;


                   
    double ofset = 0.1;
    // double a  = p1(0) - p0(0); 
    // double b = p1(1) - p0(1);
    // double front_tip_dist = sqrt( pow((double)(p1(0) - p0(0)),2) + pow((double)(p1(1) - p0(1)),2) );
    // std::cout<<front_tip_dist/2<<std::endl;

    // double rear_tip_dist = sqrt( pow((double)(p3(0) - p2(0)),2) + pow((double)(p3(1) - p2(1)),2) );
    // std::cout<<rear_tip_dist/2<<std::endl;

    // double right_tip_dist = sqrt( pow((double)(p2(0) - p0(0)),2) + pow((double)(p2(1) - p0(1)),2) );
    // std::cout<<right_tip_dist/2<<std::endl;

    // double left_tip_dist = sqrt( pow((double)(p1(0) - p3(0)),2) + pow((double)(p1(1) - p3(1)),2) );
    // std::cout<<left_tip_dist/2<<std::endl;


    Eigen::Vector3d pc0 = Eigen::Vector3d(-0.022416, 2.0279e-05, 0.310678);
    // double mc = 0.00351716; // m = tan(ea(0)) EULER ANGLE
    // double bc = pc0(1) - mc*pc0(0);
    // double yc, xc;

    // double bR = bc - 0.128/cos(atan(mc));
    // double bL = bc + 0.128/cos(atan(mc));
    // double Dx = cos(atan(mc))*ofset;
    // double Dy = sin(atan(mc))*ofset;
    // double front_ = 0.17;
    // double rear_ = -0.17;

    double d = 0.128, l1 = 0.17, l2 = 0.19;
    Eigen::Vector2d A0(pc0(0) + l1, pc0(1) - d);
    Eigen::Vector2d A2(pc0(0) - l2, pc0(1) - d);
    Eigen::Vector2d A1(pc0(0) + l1, pc0(1) + d);
    Eigen::Vector2d A3(pc0(0) - l2, pc0(1) + d);

    Eigen::Matrix3d R;
    R(0,0) = 1;  R(0,1) = -9.13051e-05; R(0,2) = -0.000124806;
    R(1,0) = 9.14191e-05;  R(1,1) =  1; R(1,2) =  0.000913243;
    R(2,0) = 0.000124723;  R(2,1) = -0.000913254 ; R(2,2) =  1;
    
        // std::cout<<0<<" "<< pc0(0)<<" "<<pc0(1)<<" "<<
        //                     p0(0)<<" "<<p0(1)<<" "<<
        //                     p2(0)<<" "<<p2(1)<<" "<<
        //                     p1(0)<<" "<<p1(1)<<" "<<
        //                     p3(0)<<" "<<p3(1)<< std::endl;

    int num_steps = 20;
    Eigen::Vector3d start, goal, current;
    Eigen::Vector2d help_vect = R.block(0,0,2,2)*A3;
    Eigen::Vector3d vect_A0(help_vect(0),help_vect(1), 0.019);
    std::cout<<vect_A0(0)<<" "<<vect_A0(1)<<" "<<vect_A0(2)<<std::endl;
    // start = p0;
    // for(int i=1; i <= num_steps; i++)
    // {
    //     // Virtual CoM ref
    //     // yc = mc*( pc0(0) + i*Dx) + bc;

    //     // std::cout<<yc<<" "<<pc0(1) + i*Dy<<" "
    //     // std::cout<<i<<" "<<pc0(0) + i*Dx<<" "<<yc<<" "//<<pc0(1) + i*Dy<<" "
    //     // <<( pc0(0) + cos(atan(mc))*front_ ) + i*Dx<<" "<< mc*( ( pc0(0) + cos(atan(mc))*front_ ) + i*Dx) + bR <<" "
    //     // <<( pc0(0) + cos(atan(mc))*rear_  ) + i*Dx<<" "<< mc*( ( pc0(0) + cos(atan(mc))*rear_ ) + i*Dx) + bR <<" "
        
    //     // <<( pc0(0) + cos(atan(mc))*front_ ) + i*Dx<<" "<< mc*( ( pc0(0) + cos(atan(mc))*front_ ) + i*Dx) + bL <<" "
    //     // <<( pc0(0) + cos(atan(mc))*rear_  ) + i*Dx<<" "<< mc*( ( pc0(0) + cos(atan(mc))*rear_ ) + i*Dx) + bL <<std::endl;
    //     // goal = Eigen::Vector3d(( pc0(0) + cos(atan(mc))*rear_  ) + i*Dx, mc*( ( pc0(0) + cos(atan(mc))*rear_ ) + i*Dx) + bR, 0.019);
    //     std::cin>>current(0);
    //     std::cin>>current(1);
    //     current(2) = 0.019;

    //     Eigen::Vector3d vect_A0(A0*R.block(0,0,2,2), 0.019);
    //     generateBezier(start, vect_A0, i-1);
    //     // start = goal;
    // }

    
}

void sigmoid_goaltracker()
{
    double t_down = 0.4 + 0.9*1/2.0;
    double t_phase = 0.85;
    double c1 = 20,c2 = 0.2;

    double ampli = 0.22 - 0.019 ;

    for(double t = t_down; t < t_down + 1; t += 0.002)
    {
        std::cout<<t<<" "<< 0.019 + ampli*(1 - sigmoid( t - t_down , c1, c2)) <<" "<< ampli*(-der_sigmoid(t - t_down , c1, c2) )<<" "<<   ampli*(- (sigmoid(t - t_down, c1, c2)*(1-sigmoid(t - t_down, c1, c2))) )<<std::endl;
    }

}


void generateStepsOld()
{
    Eigen::Vector3d p0(0.147791, -0.128676 ,0.0192237);
    Eigen::Vector3d p3(-0.225242, 0.127784, 0.019249);
    Eigen::Vector3d p1(0.149468, 0.128349 ,0.0184109);
    Eigen::Vector3d p2(-0.223524, -0.128083 ,0.0184252);

    double mR, mL, m, bR, bL, b;
    double ofset = 0.1;
    int num = 5;

//////////////////////////////////////////////////////////

    // Right line connecting FR - RR
    mR = ( abs(p0(0) - p2(0)) > 0.0001) ? (p0(1) - p2(1))/(p0(0) - p2(0)) : 0.0;
    bR = p0(1) -  mR*p0(0);
    // Left line connecting FL - RL
    mL = ( abs(p1(0) - p3(0)) > 0.0001) ? (p1(1) - p3(1))/(p1(0) - p3(0)) : 0.0;
    bL = p1(1) -  mL*p1(0);

    double Dx_R = cos(atan(mR))*ofset;
    double Dy_R = sin(atan(mR))*ofset;
    double Dx_L = cos(atan(mL))*ofset;
    double Dy_L = sin(atan(mL))*ofset;

//////////////////////////////////////////////////////////

    m = 0.0;//(mR+mL)/2;
    b = 0.128;
    double Dx = cos(atan(m))*ofset;
    double Dy = sin(atan(m))*ofset;
    // std::cout<<mR<<std::endl;
    // std::cout<<mL<<std::endl;
    // std::cout<<m<<std::endl;
    // std::cout<<bR<<std::endl;
    // std::cout<<bL<<std::endl;
    // std::cout<<b<<std::endl;
    // std::cout<<Dx<<std::endl;
    // std::cout<<Dy<<std::endl;



    std::vector<Eigen::Vector3d> steps_FR, steps_RR, steps_FL, steps_RL;

        std::cout<<p0(0)<<" "<<p0(1)<<" "<<
                   p2(0)<<" "<<p2(1)<<" "<<
                   p1(0)<<" "<<p1(1)<<" "<<
                   p3(0)<<" "<<p3(1)<< std::endl;

    for(int i = 1 ; i <= num ; i++)
    {
        // steps_FR.push_back(Eigen::Vector3d( p0(0)+i*Dx_R, mR*(p0(0)+i*Dx_R) + bR, p0(2)));
        // steps_RR.push_back(Eigen::Vector3d( p2(0)+i*Dx_R, mR*(p2(0)+i*Dx_R) + bR, p2(2)));
        // // std::cout<<"********** START    ************" <<std::endl;
        // // std::cout<<"y case a" <<p0(1)+i*Dy<<std::endl;
        // // std::cout<<"y case b" <<mR*(p0(0)+i*Dx) + bR<<std::endl;
        // // std::cout<<"********** END   ************" <<std::endl;
        // steps_FL.push_back(Eigen::Vector3d( p1(0)+i*Dx_L,mL*(p1(0)+i*Dx_L) + bL, p1(2)));
        // steps_RL.push_back(Eigen::Vector3d( p3(0)+i*Dx_L,mL*(p3(0)+i*Dx_L) + bL, p3(2)));

        // std::cout<<p0(0)+i*Dx_R<<" "<< mR*(p0(0)+i*Dx_R) + bR<<" "<<
        //            p2(0)+i*Dx_R<<" "<<mR*(p2(0)+i*Dx_R) + bR<<" "<<
        //            p1(0)+i*Dx_L<<" "<<mL*(p1(0)+i*Dx_L) + bL<<" "<<
        //            p3(0)+i*Dx_L<<" "<<mL*(p3(0)+i*Dx_L) + bL<< std::endl;

        steps_FR.push_back(Eigen::Vector3d( p0(0)+i*Dx, m*(p0(0)+i*Dx) + b, p0(2)));
        steps_RR.push_back(Eigen::Vector3d( p2(0)+i*Dx, m*(p2(0)+i*Dx) + b, p2(2)));
        // std::cout<<"********** START    ************" <<std::endl;
        // std::cout<<"y case a" <<p0(1)+i*Dy<<std::endl;
        // std::cout<<"y case b" <<mR*(p0(0)+i*Dx) + bR<<std::endl;
        // std::cout<<"********** END   ************" <<std::endl;
        steps_FL.push_back(Eigen::Vector3d( p1(0)+i*Dx,m*(p1(0)+i*Dx) + b, p1(2)));
        steps_RL.push_back(Eigen::Vector3d( p3(0)+i*Dx,m*(p3(0)+i*Dx) + b, p3(2)));

        std::cout<<p0(0)+i*Dx<<" "<<m*(p0(0)+i*Dx) + b<<" "<<
                   p2(0)+i*Dx<<" "<<m*(p2(0)+i*Dx) + b<<" "<<
                   p1(0)+i*Dx<<" "<<m*(p1(0)+i*Dx) + b<<" "<<
                   p3(0)+i*Dx<<" "<<m*(p3(0)+i*Dx) + b<< std::endl;
    }
    

}

int main() 
{


    // Eigen::Vector3d target = generateBezier();
    // std::cout<<1.0<<" "<<target(0)<<" "<<target(1)<<" "<<target(2)<<" "<<std::endl;

    // generateSteps();
    sigmoid_goaltracker();
    return 0;
}

