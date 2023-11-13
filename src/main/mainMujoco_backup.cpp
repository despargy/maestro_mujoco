#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep
//#include <math.h>


#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

//simulation end time
double simend = 15;

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 10; //frequency at which data is written to a file

//Change the path <template_writeData>
//Change the xml file
char path[] = "/home/despinar/mujoco_ws/maestro_mujoco/";
char xmlfile[] = "xml/go1/xml/extra_scene.xml";

char datafile[] = "data/data.csv";

int ii = 0;
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
float_t ctrl_update_freq = 100;
mjtNum last_update = 0.0;
mjtNum ctrl;

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


//****************************
//This function is called once and is used to get the headers
void init_save_data()
{
  //write name of the variable here (header)
   fprintf(fid,"t,");
   fprintf(fid,"FR_foot_x,FR_foot_y,FR_foot_z,");
   fprintf(fid,"FL_foot_x,FL_foot_y,FL_foot_z,");
   fprintf(fid,"RR_foot_x,RR_foot_y,RR_foot_z,");
   fprintf(fid,"RL_foot_x,RL_foot_y,RL_foot_z,");

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  fprintf(fid,"%f,",d->time);
  fprintf(fid,"%f,%f,%f,", d->sensordata[89], d->sensordata[90], d->sensordata[91]);
  fprintf(fid,"%f,%f,%f,", d->sensordata[92], d->sensordata[93], d->sensordata[94]);
  fprintf(fid,"%f,%f,%f,", d->sensordata[95], d->sensordata[96], d->sensordata[97]);
  fprintf(fid,"%f,%f,%f,", d->sensordata[98], d->sensordata[99], d->sensordata[100]);

  //Don't remove the newline
  fprintf(fid,"\n");
}

/******************************/
void set_torque_control(const mjModel* m,int actuator_no,int flag)
{
  if (flag==0)
    m->actuator_gainprm[10*actuator_no+0]=0;
  else
    m->actuator_gainprm[10*actuator_no+0]=1;
}
/******************************/


/******************************/
void set_position_servo(const mjModel* m,int actuator_no,double kp)
{
  m->actuator_gainprm[10*actuator_no+0]=kp;
  m->actuator_biasprm[10*actuator_no+1]=-kp;
}
/******************************/

/******************************/
void set_velocity_servo(const mjModel* m,int actuator_no,double kv)
{
  m->actuator_gainprm[10*actuator_no+0]=kv;
  m->actuator_biasprm[10*actuator_no+2]=-kv;
}
/******************************/

//**************************
void init_controller(const mjModel* m, mjData* d)
{
    while(d->time<1)
    {
        printf("wait\n");
        mj_step(m,d);
    }
}
void nextQdes(int i, double duration, double* targetPos)
{
    double percent = (double)i/duration;
    double lastPos[12];
    for(int j=0; j<12; j++)
    {
        lastPos[j] = d->sensordata[j];
    }  
    for(int j = 0; j < 12 ; j++)
    {
       d->ctrl[j] = -100*(d->sensordata[j]-(lastPos[j]*(1-percent) + targetPos[j]*percent))-10*d->sensordata[j+12];
       
    }
}
//**************************
void fake(const mjModel* m, mjData* d)
{
}

void mycontroller(const mjModel* m, mjData* d)
{
    double targetPos[12] = {0.0, 0.67, -1.3, -0.0, 0.67, -1.3, 
                                            0.0, 0.67, -1.3, -0.0, 0.67, -1.3};
        


    if(d->time<6)
    {
        /* Configure init sit down pose for robot */
        d->ctrl[0] = -70*(d->sensordata[0]-0.0)-10*d->sensordata[0+12];
        d->ctrl[3] = -70*(d->sensordata[3]+0.0)-10*d->sensordata[3+12];
        d->ctrl[6] = -70*(d->sensordata[6]-0.0)-10*d->sensordata[6+12];
        d->ctrl[9] = -70*(d->sensordata[9]+0.0)-10*d->sensordata[9+12];

        d->ctrl[1 ] = -180*(d->sensordata[1 ]-1.13)-10*d->sensordata[1 +12];
        d->ctrl[4 ] = -180*(d->sensordata[4 ]-1.13)-10*d->sensordata[4 +12];
        d->ctrl[7 ] = -180*(d->sensordata[7 ]-1.13)-10*d->sensordata[7 +12];
        d->ctrl[10] = -180*(d->sensordata[10]-1.13)-10*d->sensordata[10+12];

        d->ctrl[2 ] = -300*(d->sensordata[2 ]+2.7)-10*d->sensordata[2 +12];
        d->ctrl[5 ] = -300*(d->sensordata[5 ]+2.7)-10*d->sensordata[5 +12];
        d->ctrl[8 ] = -300*(d->sensordata[8 ]+2.7)-10*d->sensordata[8 +12];
        d->ctrl[11] = -300*(d->sensordata[11]+2.7)-10*d->sensordata[11+12];

    }
    else
    {
        

        ii += 1;
        nextQdes(ii, 5000, targetPos);
        // d->ctrl[0] = -70*(d->sensordata[0]-0.2)-3*d->sensordata[0+12];
        // d->ctrl[3] = -70*(d->sensordata[3]-0.2)-3*d->sensordata[3+12];
        // d->ctrl[6] = -70*(d->sensordata[6]-0.2)-3*d->sensordata[6+12];
        // d->ctrl[9] = -70*(d->sensordata[9]-0.2)-3*d->sensordata[9+12];

        // d->ctrl[1 ] = -180*(d->sensordata[1 ]-1.5)-8*d->sensordata[1 +12];
        // d->ctrl[4 ] = -180*(d->sensordata[4 ]-1.5)-8*d->sensordata[4 +12];
        // d->ctrl[7 ] = -180*(d->sensordata[7 ]-1.5)-8*d->sensordata[7 +12];
        // d->ctrl[10] = -180*(d->sensordata[10]-1.5)-8*d->sensordata[10+12];

        // d->ctrl[2 ] = -300*(d->sensordata[2 ]+M_PI)-15*d->sensordata[2 +12];
        // d->ctrl[5 ] = -300*(d->sensordata[5 ]+M_PI)-15*d->sensordata[5 +12];
        // d->ctrl[8 ] = -300*(d->sensordata[8 ]+M_PI)-15*d->sensordata[8 +12];
        // d->ctrl[11] = -300*(d->sensordata[11]+M_PI)-15*d->sensordata[11+12];
    }
    printf("****************\n");
  //write data here (dont change/dete this function call; instead write what you need to save in save_data)
  if ( loop_index%data_frequency==0)
    {
      save_data(m,d);
    }
  loop_index = loop_index + 1;
}


//************************
// main function
int main(int argc, const char** argv)
{

    // activate software
    mj_activate("mjkey.txt");

    char xmlpath[100]={};
    char datapath[100]={};

    strcat(xmlpath,path);
    strcat(xmlpath,xmlfile);

    strcat(datapath,path);
    strcat(datapath,datafile);


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

    // install control callback
    mjcb_control = mycontroller;


    fid = fopen(datapath,"w");
    init_save_data();
    init_controller(m,d);


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

        if (d->time>=simend)
        {
           fclose(fid);
           break;
         }


       // get framebuffer viewport
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

          // update scene and render
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
    mj_deactivate();

    // terminate GLFW (crashes with Linux NVidia drivers)
    #if defined(__APPLE__) || defined(_WIN32)
        glfwTerminate();
    #endif

    return 1;
}