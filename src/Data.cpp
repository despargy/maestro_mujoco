#include <Data.h>
/* Constructor*/
Data::Data()
{
    std::filesystem::path currentPath = std::filesystem::current_path();
    currentPath = currentPath.parent_path();
    currentPath = currentPath.parent_path();
    currentPath = currentPath.parent_path();
    currentPath /= "data/data.csv";
    datafile = currentPath.c_str(); // datafile to save data
    fid = fopen(datafile,"w"); // open it for write
}
Data::Data(char* file_)
{
    datafile = file_; // datafile to save data
    fid = fopen(datafile,"w"); // open it for write
}
/* De-Constructor*/
Data::~Data(){}

/* ************************* Trajectory Tracking ****************************** */
/* ************************* ******************** ****************************** */

/* Init function of Data to write headers*/
void Data::init_save_data()
{
    //write name of the variable here (header)
    fprintf(fid,"CoM_x,CoM_y,CoM_z,ep_x,ep_y,ep_z");

    //move to a newline
    fprintf(fid,"\n");
}

/* Function which defines the data/variables we want to save to the csv file*/
void Data::save(double time, double pc0, double pc1, double pc2, double ep0, double ep1, double ep2, double d_p0, double d_p1, double d_p2, double eo0, double eo1, double eo2)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f ",time, pc0, pc1, pc2, ep0, ep1, ep2,d_p0,d_p1,d_p2, eo0, eo1, eo2); 
    //move to a newline
    fprintf(fid,"\n");
}
void Data::save_error(double time, double ep0, double ep1, double ep2)
{
    fprintf(fid, "%f %f %f %f",time, ep0, ep1, ep2);
    //move to a newline
    fprintf(fid,"\n");
}




/* *************************                      ****************************** */
/* ************************* ******************** ****************************** */


/* *************************     Locomotion  ****************************** */
/* ************************* ******************** ****************************** */

void Data::init_save_data_locomotion()
{
    //write name of the variable here (header)
    fprintf(fid,"time, CoM_x,CoM_y,CoM_z,ep_x,ep_y,ep_z");

    //move to a newline
    fprintf(fid,"\n");
}
void Data::init_save_data_dynamic()
{
    //write name of the variable here (header)
    fprintf(fid,"time, CoM_x,CoM_y,CoM_z,ep_x,ep_y,ep_z");

    //move to a newline
    fprintf(fid,"\n");
}
/* *************************                      ****************************** */
/* ************************* ******************** ****************************** */


/* Function which defines the data/variables we want to save to the csv file*/
void Data::save_loc(double time, double p0x, double p0y, double p0z, double w0, double w1, double w2, double w3, double p0xd, double p0yd, double p0zd,double x1, double x2, double x3,double x1_d, double x2_d, double x3_d, double f)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",time,p0x,p0y,p0z,w0, w1, w2, w3,p0xd,p0yd,p0zd,x1,x2,x3,x1_d,x2_d,x3_d,f); 
    //move to a newline
    fprintf(fid,"\n");
}
/* Function which defines the data/variables we want to save to the csv file*/
void Data::save_dyna(double time, double pcx, double pcy, double pcz, double w0, double w1, double w2, double w3, double f0x, double f0y, double f0z,double f1x, double f1y, double f1z, double f2x, double f2y, double f2z,double f3x, double f3y, double f3z, double tip0_x,double tip0_y,double tip0_z,double tip1_x,double tip1_y,double tip1_z ,double tip2_x,double tip2_y,double tip2_z,double tip3_x,double tip3_y,double tip3_z , double bA_x,double bA_y,double bA_z,double bB_x,double bB_y,double bB_z, double ev_x, double ev_y, double ev_z )
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",time,pcx,pcy,pcz,w0, w1, w2, w3,f0x, f0y, f0z, f1x, f1y, f1z, f2x, f2y, f2z, f3x, f3y, f3z, tip0_x, tip0_y, tip0_z, tip1_x, tip1_y, tip1_z, tip2_x, tip2_y, tip2_z, tip3_x, tip3_y, tip3_z, bA_x,bA_y,bA_z,bB_x,bB_y,bB_z, ev_x, ev_y, ev_z ); 
    //move to a newline
    fprintf(fid,"\n");
}


/* Init function of Data to write headers*/
void Data::init_save_data_exp()
{
    //write name of the variable here (header)
    fprintf(fid,"Time,Wx0,Wx1,Wx2,Wx3, Bx,By, Bz, Tipx, Tipy,Tipz, px, py, pz, pTx, pTy, pTz");

    //move to a newline
    fprintf(fid,"\n");
}
/* Function which defines the data/variables we want to save to the csv file*/
void Data::save_exp(double time, double w0, double w1, double w2, double w3, float bx, float by, float bz, float tipx, float tipy, float tipz, double pcx, double pcy, double pcz, double pTx, double pTy, double pTz)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",time,w0, w1, w2, w3, bx, by, bz, tipx, tipy, tipz, pcx, pcy, pcz, pTx, pTy, pTz); 
    //move to a newline
    fprintf(fid,"\n");
}

void Data::save_slip(double time, double pcx, double pcy, double pcz, double w0, double w1, double w2, double w3, double prob0, double prob1, double prob2, double prob3, double wStanceA, double wStanceB, double probStanceA, double probStanceB,double wSwingA, double wSwingB, double probSwingA, double probSwingB, double ev_x, double ev_y, double ev_z, double fAz, double fBz, double fAz_stance, double fBz_stance, double vx, double vy, double vz)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",time,pcx,pcy,pcz,w0, w1, w2, w3, prob0, prob1, prob2, prob3,   wStanceA,  wStanceB,  probStanceA,  probStanceB, wSwingA,  wSwingB,  probSwingA,  probSwingB, ev_x, ev_y, ev_z,  fAz, fBz, fAz_stance, fBz_stance, vx, vy, vz); 
    //move to a newline
    fprintf(fid,"\n");
}

void Data::save_compare(double time, double Vcx, double Vcy, double pcz, double w0, double w1, double w2, double w3,  double eo_x, double eo_y, double eo_z, double ev_x, double ev_y, double ev_z, double t1, double t2, double t3, double t4)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",time,Vcx,Vcy,pcz,w0, w1, w2, w3,eo_x, eo_y, eo_z, ev_x, ev_y, ev_z, t1, t2, t3, t4); 
    //move to a newline
    fprintf(fid,"\n");
}