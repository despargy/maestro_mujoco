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
        void save_error(double time, double ep0, double ep1, double ep2) ;
        void save(double time,double pc0, double pc1, double pc2, double ep0, double ep1, double ep2,double d_p0, double d_p1, double d_p2, double eo0, double eo1, double eo2) ;
        void save_loc(double time, double p0x, double p0y, double p0z, double w0, double w1, double w2, double w3, double p0xd, double p0yd, double p0zd);

};

#endif