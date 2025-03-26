#include <string.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <mujoco/mujoco.h>



#ifndef _PERCEPTION_H_
#define _PERCEPTION_H_



class Perception
{
    public:

        // Set resolution
        const int WIDTH = 640;
        const int HEIGHT = 480;

        cv::Mat depth_gray;
        cv::Mat depth_normalized;
        cv::Mat depth_img;
        cv::Mat rgb_img;

        Perception();
        ~Perception();
        void get_rgbd(const mjModel* model, mjData* data, mjrContext& context);
};

#endif