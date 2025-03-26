#include <Perception.h>
/* Constructor*/
Perception::Perception()
{

    // // depth_gray.create(HEIGHT, WIDTH, CV_32F); 
    // depth_gray.create(HEIGHT, WIDTH, CV_8UC3); 
    // depth_img.create(HEIGHT, WIDTH, CV_32F); 
    // depth_normalized.create(HEIGHT, WIDTH, CV_32F); 
    // rgb_img.create(HEIGHT, WIDTH, CV_8UC3);

}
/* De-Constructor*/
Perception::~Perception()
{
    // depth_gray.release();
    // depth_img.release();
    // depth_normalized.release();
    // rgb_img.release();

}
void Perception::get_rgbd(const mjModel* model, mjData* data, mjrContext& context)
{
    // Create buffers for RGB and Depth data
    std::vector<unsigned char> rgb(WIDTH * HEIGHT * 3);
    std::vector<float> depth(WIDTH * HEIGHT);

    // Initialize camera
    mjvCamera cam;
    mjv_defaultCamera(&cam);
    cam.type = mjCAMERA_FIXED;
    cam.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "realsense_cam");  // Attach to robot camera

    // Visualization settings
    mjvScene scene;
    mjvOption opt;
    mjv_defaultScene(&scene);
    mjv_defaultOption(&opt);
    mjv_makeScene(model, &scene, 1000);  // Allocate resources for the scene

    // Update the scene based on camera position
    mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);

    // Now, render the camera image (this is the crucial part)
    mjrRect viewport = {0, 0, WIDTH, HEIGHT}; // Set the viewport dimensions
    mjr_render(viewport, &scene, &context);   // Render using context (this updates the visuals)

    // Capture RGB and depth data from the simulation using mjr_readPixels
    mjr_readPixels(rgb.data(), depth.data(), viewport, &context);


    // Convert depth to grayscale
    cv::Mat depth_img(HEIGHT, WIDTH, CV_32F, depth.data());
    cv::Mat depth_gray;

    // Normalize depth for visualization (adjust the max depth if needed)
    cv::Mat depth_normalized;
    cv::normalize(depth_img, depth_normalized, 0, 255, cv::NORM_MINMAX);
    depth_normalized.convertTo(depth_gray, CV_8UC1);


    // Convert RGB data to OpenCV format
    cv::Mat rgb_img(HEIGHT, WIDTH, CV_8UC3, rgb.data());
    cv::cvtColor(rgb_img, rgb_img, cv::COLOR_RGB2BGR);  // Convert RGB to BGR for OpenCV display

    // Rotate the image if necessary (correct the 90-degree rotation)
    cv::rotate(rgb_img, rgb_img, cv::ROTATE_90_COUNTERCLOCKWISE);
    cv::rotate(depth_gray, depth_gray, cv::ROTATE_90_COUNTERCLOCKWISE);


    // Show the images in OpenCV windows
    cv::imshow("Robot Camera (RGB)", rgb_img);
    cv::imshow("Robot Camera (Depth)", depth_gray);
    cv::waitKey(1);  // Display in a loop with slight delay

    // Clean up the scene after rendering
    mjv_freeScene(&scene);
}

// void Perception::get_rgbd(const mjModel* model, mjData* data, mjrContext& context)
// {
//     // Create buffers for RGB and Depth data
//     std::vector<unsigned char> rgb(WIDTH * HEIGHT * 3);
//     std::vector<float> depth(WIDTH * HEIGHT);

//     // Initialize camera
//     mjvCamera cam;
//     mjv_defaultCamera(&cam);
//     cam.type = mjCAMERA_FIXED;
//     cam.fixedcamid = mj_name2id(model, mjOBJ_CAMERA, "realsense_cam");  // Attach to robot camera

//     // Visualization settings
//     mjvScene scene;
//     mjvOption opt;
//     mjv_defaultScene(&scene);
//     mjv_defaultOption(&opt);
//     mjv_makeScene(model, &scene, 1000);  // Allocate resources for the scene

//     // Update the scene based on camera position
//     mjv_updateScene(model, data, &opt, nullptr, &cam, mjCAT_ALL, &scene);

//     // Now, render the camera image (this is the crucial part)
//     mjrRect viewport = {0, 0, WIDTH, HEIGHT}; // Set the viewport dimensions
//     mjr_render(viewport, &scene, &context);   // Render using context (this updates the visuals)

//     // Capture RGB and depth data from the simulation using mjr_readPixels
//     mjr_readPixels(rgb.data(), depth.data(), viewport, &context);


//     // // Convert depth to grayscale
//     // cv::Mat depth_img(HEIGHT, WIDTH, CV_32F, depth.data());

//     // // Normalize depth for visualization (adjust the max depth if needed)
//     // cv::normalize(depth_img, depth_normalized, 0, 255, cv::NORM_MINMAX);
//     // depth_normalized.convertTo(depth_gray, CV_8UC1);


//     // // Convert RGB data to OpenCV format
//     // cv::Mat rgb_img(HEIGHT, WIDTH, CV_8UC3, rgb.data());
//     // cv::cvtColor(rgb_img, rgb_img, cv::COLOR_RGB2BGR);  // Convert RGB to BGR for OpenCV display

//     // // Rotate the image if necessary (correct the 90-degree rotation)
//     // cv::rotate(rgb_img, rgb_img, cv::ROTATE_90_COUNTERCLOCKWISE);
//     // cv::rotate(depth_gray, depth_gray, cv::ROTATE_90_COUNTERCLOCKWISE);


//     // // Show the images in OpenCV windows
//     // cv::imshow("Robot Camera (RGB)", rgb_img);
//     // cv::imshow("Robot Camera (Depth)", depth_gray);
//     // cv::waitKey(1);  // Display in a loop with slight delay

//     // // Clean up the scene after rendering
//     // mjv_freeScene(&scene);


// }