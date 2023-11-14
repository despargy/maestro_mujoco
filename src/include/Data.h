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
        void save_loc(double time, double pc0, double pc1, double pc2, double ep0, double ep1, double ep2, double d_p0, double d_p1, double d_p2, double pT0, double pT1, double pT2);

};

#endif