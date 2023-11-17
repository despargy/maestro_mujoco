#include <Data.h>
/* Constructor*/
Data::Data()
{

    datafile = "/home/despinar/mujoco_ws/maestro_mujoco/data/data.csv"; // datafile to save data
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
    fprintf(fid,"CoM_x,CoM_y,CoM_z,ep_x,ep_y,ep_z");

    //move to a newline
    fprintf(fid,"\n");
}

/* *************************                      ****************************** */
/* ************************* ******************** ****************************** */


/* Function which defines the data/variables we want to save to the csv file*/
void Data::save_loc(double time, double p0x, double p0y, double p0z, double w0, double w1, double w2, double w3, double p0xd, double p0yd, double p0zd)
{
    fprintf(fid, "%f %f %f %f %f %f %f %f %f %f %f",time,p0x,p0y,p0z,w0, w1, w2, w3,p0xd,p0yd,p0zd); 
    //move to a newline
    fprintf(fid,"\n");
}

