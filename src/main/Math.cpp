#include <Math.h>

Eigen::Matrix3d scewSymmetric(Eigen::Vector3d t)
{
    Eigen::Matrix3d t_hat;
    t_hat << 0, -t(2), t(1),
        t(2), 0, -t(0),
        -t(1), t(0), 0;
    return t_hat;
}
Eigen::Vector3d scewSymmetricInverse(Eigen::Matrix3d m)
{
    return Eigen::Vector3d( (m(2,1)-m(1,2))/2.0, (m(0,2) - m(2,0))/2.0, (m(1,0) - m(0,1))/2.0 );
}
Eigen::Vector3d deriv_RcRdTwd(Eigen::Vector3d RcRdTwd_prev, Eigen::Vector3d RcRdTwd_cur, double dt)
{
    return  (RcRdTwd_cur - RcRdTwd_prev)/dt;
}
Eigen::Vector3d get_dp_CoM(Eigen::Vector3d com_p_prev,Eigen::Vector3d com_p_cur,double dt)
{
    return (com_p_cur - com_p_prev )/dt;
}
Eigen::Matrix3d get_dR_CoM(Eigen::Matrix3d R_CoM_prev,Eigen::Matrix3d R_CoM_cur, double dt)
{
    return (R_CoM_cur - R_CoM_prev)/dt;
}

double sigmoid(double t)
{
    return 1/(1 + exp( -1000*(t-0.03)) );
}
 
std::pair<double, double> find_Centroid(std::vector<std::pair<double, double> >& v)
{
    std::pair<double, double> ans = { 0, 0 };
     
    int n = v.size();
    double signedArea = 0;
     
    // For all vertices
    for (int i = 0; i < v.size(); i++) {
         
        double x0 = v[i].first, y0 = v[i].second;
        double x1 = v[(i + 1) % n].first, y1 =
                            v[(i + 1) % n].second;
                             
        // Calculate value of A
        // using shoelace formula
        double A = (x0 * y1) - (x1 * y0);
        signedArea += A;
         
        // Calculating coordinates of
        // centroid of polygon
        ans.first += (x0 + x1) * A;
        ans.second += (y0 + y1) * A;
    }
 
    signedArea *= 0.5;
    ans.first = (ans.first) / (6 * signedArea);
    ans.second = (ans.second) / (6 * signedArea);
 
    return ans;
}

double superGaussian(double A,double b,double r,double d, double n)
{
    return A*pow( b, -pow(pow(d,2)/pow(r,2),n*r) );
}
