#include <stdio.h>
#include <stdlib.h>
#include <string>

#ifndef _DATA_H_
#define _DATA_H_

class Data
{
    public:
        FILE *fid;
        char *datafile;

        Data();
        Data(char* file_);
        ~Data();
        void init_save_data();
        void init_save_data_locomotion();
        void init_save_data_dynamic();

        void save_error(double time, double ep0, double ep1, double ep2) ;
        void save(double time,double pc0, double pc1, double pc2, double ep0, double ep1, double ep2,double d_p0, double d_p1, double d_p2, double eo0, double eo1, double eo2) ;
        void save_loc(double time, double p0x, double p0y, double p0z, double w0, double w1, double w2, double w3, double p0xd, double p0yd, double p0zd,double x1, double x2, double x3,double x1_d, double x2_d, double x3_d, double f);
        void save_dyna(double time, double pcx, double pcy, double pcz, double w0, double w1, double w2, double w3, double f0x, double f0y, double f0z,double f1x, double f1y, double f1z, double f2x, double f2y, double f2z,double f3x, double f3y, double f3z, double tip0_x,double tip0_y,double tip0_z,double tip1_x,double tip1_y,double tip1_z ,double tip2_x,double tip2_y,double tip2_z,double tip3_x,double tip3_y,double tip3_z , double bA_x,double bA_y,double bA_z,double bB_x,double bB_y,double bB_z,double ev_x, double ev_y, double ev_z );

        void save_exp(double time, double w0, double w1, double w2, double w3, float bx, float by, float bz, float tipx, float tipy, float tipz, double pcx, double pcy, double pcz, double pTx, double pTy, double pTz);
        void init_save_data_exp();

};

#endif