#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <LocomotionTopLevelControl.h>

//simulation end time
// double simend = 30;

//related to writing data to a file
int loop_index = 0;

//Change the xml file
char path[] = "../../../xml/";
char xmlfile[] = "unitree_go2/scene_multi.xml"; //

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context
mjvFigure fig;

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// holders of one step history of time and position to calculate dertivatives
mjtNum position_history = 0;
mjtNum previous_time = 0;

// controller related variables
// float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

/* Maestro TopLevelController global pointer definition*/
LocomotionTopLevelControl* topController;
bool FIND_PARAMS = true; 

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}


// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}
bool ONCE_SAVE = true;
void my_controller_walk(const mjModel* m, mjData* d)
{

    if( (d->time - topController->t_last_c) >= topController->controller->dt )
    {
    
        topController->wrapper->update_locomotion(m,d,topController->controller->dt);
        //Probabilistic Contact Estimation DO NOT NEED TO RUN, ONLY FOR FIRST TIME of your model
        // if(topController->fsm->state == S3)
        //     topController->wrapper->find_params_PCE();
        // else if(FIND_PARAMS & topController->fsm->state == DYNA_GAIT)
        // {
        //     for(int i = 0; i<topController->controller->robot->n_legs; i++)
        //     {
        //         std::cout<<"LEG "<<i<<std::endl;
        //         topController->wrapper->pce_obj[i].compute_mean_std();
        //     }
        //     FIND_PARAMS = false;
        // }
        if(topController->fsm->state == S3)
        {
            // std::cout<<"G: \n"<<topController->controller->robot->Gq<<std::endl;
            topController->wrapper->init_PCE();
            // topController->wrapper->pce_obj[0].save_csv();
        }
        // Uncomment  those to store IMU data in .csv of one leg before start locomotion
        // else if(topController->fsm->state == DYNA_GAIT & ONCE_SAVE)
        // {
        //     topController->wrapper->pce_obj[3].save_csv();
        //     ONCE_SAVE = false;
        // }
        else if(topController->fsm->state == DYNA_GAIT or topController->fsm->state == INCLINED_DYNA_GAIT)
        {
            // topController->wrapper->update_PCE();
            // topController->wrapper->update_PCE_forces(topController->controller->f_applied_a(2), topController->controller->f_applied_b(2));
            topController->wrapper->update_PCE_onlystance();
            // topController->wrapper->pce_obj[0].save_csv();
        }

        topController->computeDynamic(d->time); // call once
        topController->wrapper->set_gains(m,d,topController->controller->A_PD,topController->controller->B_PD); 
        topController->wrapper->send_torque_pos_Dynamic(m,d,topController->controller->A_PD,topController->controller->B_PD); 

    }
}


// main function
int main(int argc, const char** argv)
{

    char xmlpath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    // load and compile model
    char error[1000] = "Could not load binary model";

    // check command-line arguments
    if( argc<2 )
        m = mj_loadXML(xmlpath, 0, error, 1000);

    else
        if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
            m = mj_loadModel(argv[1], 0);
        else
            m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1244, 700, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    // mjv_defaultFigure(&fig);
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context
    
    // mjrRect fig_rec{0,0,120,300};
    // mjr_figure(fig_rec, &fig, &con);

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {-129.477651, -3.102665, 2.209726, -0.047404, -0.001591, 0.330533}; //view the left side (for ll, lh, left_side)
    cam.azimuth = arr_view[0];
    cam.elevation = arr_view[1];
    cam.distance = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];


    /* Maestro - new TopLevelControl of the global pointer */
    topController = new LocomotionTopLevelControl("Mujoco") ; // later define gait for walk : gait type 2
    topController->init_topControlDynamic(m,d); // call once

    // install control callback
    mjcb_control = my_controller_walk;  // set the myTopLevel instead of the default Mujoco's one


    // use the first while condition if you want to simulate for a period.
    while( !glfwWindowShouldClose(window))
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 )
        {
            mj_step(m, d);
        }    

        // if (d->time>=topController->fsm->t_End)
        // {
        //    fclose(topController->data->fid);  
        //    break;
        //  }

       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
        // opt.frame = mjFRAME_WORLD; // Remove this one HERE
        cam.lookat[0] = d->qpos[0] ;
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        // printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

        // swap OpenGL buffers (blocking call due to v-sync)
        glfwSwapBuffers(window);

        // process pending GUI events, call GLFW callbacks
        glfwPollEvents();

    }

    // free visualization storage
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    // mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}