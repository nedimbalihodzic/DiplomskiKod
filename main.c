

#include<stdbool.h> //for bool
//#include<unistd.h> //for usleep

#include <math.h>
#include "mujoco/mujoco.h"
#include "GLFW/glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"

//simulation end time
double simend = 10;

#define ndof 6
#define na 4
#define M_PI 3.14159265358979323846

//related to writing data to a file
FILE *fid;
int loop_index = 0;
const int data_frequency = 50; //frequency at which data is written to a file


char xmlpath[] = "../myproject/quadrotor/quadrotor_x.xml";
char datapath[] = "../myproject/quadrotor/data.csv";


//Change the path <template_writeData>
//Change the xml file
// char path[] = "../myproject/dbpendulum/";
// char xmlfile[] = "doublependulum.xml";


char datafile[] = "data.csv";


// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

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
   fprintf(fid,"t, ");
   fprintf(fid,"PE, KE, TE, ");
   fprintf(fid,"q1, q2, ");

   //Don't remove the newline
   fprintf(fid,"\n");
}

//***************************
//This function is called at a set frequency, put data here
void save_data(const mjModel* m, mjData* d)
{
  //data here should correspond to headers in init_save_data()
  //seperate data by a space %f followed by space
  fprintf(fid,"%f, ",d->time);
  fprintf(fid,"%f, %f, %f, ",d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);
  fprintf(fid,"%f, %f ",d->qpos[0],d->qpos[1]);
  //Don't remove the newline
  fprintf(fid,"\n");
}

//**************************

//**************************
void crossProduct(double vect_A[], double vect_B[], double cross_P[])
 
{
    cross_P[0] = vect_A[1] * vect_B[2] - vect_A[2] * vect_B[1];
    cross_P[1] = vect_A[2] * vect_B[0] - vect_A[0] * vect_B[2];
    cross_P[2] = vect_A[0] * vect_B[1] - vect_A[1] * vect_B[0];
}
//**************************

//**************************
void computeMassMatrix(const mjModel* m, mjData* d, double M[ndof][ndof]){
 double dense_M[ndof*ndof] = {0};
  mj_fullM(m,dense_M, d->qM);
   int i,j,k=0;
   for(int i=0; i<ndof; i++){
      for(int j=0; j<ndof; j++){
        M[i][j] = dense_M[k++]; 
      }
    }
}

//**************************
//***************************
// Jednacina: T = J*w_dot+ (w)x(J*w) 
void torqueVector(const mjModel* m, mjData* d, double* T){
  double M[ndof][ndof]={0};  
  computeMassMatrix(m,d,M); 
  // inertia Matrix 
  double J[9] = {0}, res1[3]={0}, w_dot[3]={0},res2[3]={0},w[3]={0}, cross_P[3]={0};
  int k=0,i,j; 
  for(i=0; i<3; i++){
      for( j=0;j<3;j++){
        J[k++] = M[i+3][j+3];    
     }
  }
  for(i=0; i<3; i++) 
  {
    w_dot[i] = d->qacc[i+3]; // ugaona ubrzanja
    w[i] = d->qvel[i+3]; // ugaone brzine
  } 
 
  mju_mulMatVec(res1, J, w_dot, 3, 3); // clan J*w_dot
  mju_mulMatVec(res2, J, w, 3, 3); // clan J*w
  crossProduct(w, res2,cross_P);
  for(i=0;i<3;i++){
    T[i] = res1[i] + cross_P[i];
   }
}
//***************************

//****************************
// Model aktuatora kao sistema prvog reda
 void actuatorModel(const mjModel* m, mjData* d, double* ctr){
 int i=0; 
 double tau[na] = {1,1,1,1}, ctrl_0[na]={0}, h=0.01; 
 for(i=0;i<na;i++){
  ctrl_0[i] = d->ctrl[i]; 
 }
 for(i=0; i<na;i++){
 //d->act[i] = ctr[i]-tau[i]*d->act_dot[i];
 //d->ctrl[i] = ctr[i]*(1-exp(-d->time/tau[i]));
  d->ctrl[i] = (h/tau[i])*(ctr[i]-ctrl_0[i])+ctrl_0[i];
 // printf("%f ", d->ctrl[i]);
 }
// printf("\n");
// printf("******\n"); 
}
//****************************
// Funkcija za odredjivanje Eulerovih uglova iz Quaterniona koji je dostupan kroz mjData.qpos
  void ToEulerAngles(const mjModel* m, mjData* d, double eulerAngles[]) {
    double w, x, y, z;
    w = d->qpos[3]; 
    x = d->qpos[4]; 
    y = d->qpos[5]; 
    z = d->qpos[6];
    // roll (x-axis rotation)
    double sinr_cosp = 2 * (w * x + y * z);
    double cosr_cosp = 1 - 2 * (x * x + y * y);
    eulerAngles[0] = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = sqrt(1 + 2 * (w * y - x * z));
    double cosp = sqrt(1 - 2 * (w * y - x * z));
     eulerAngles[1] = 2 * atan2(sinp, cosp) - M_PI / 2;

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
     eulerAngles[2] = atan2(siny_cosp, cosy_cosp);
}

//****************************
//****************************
// Jednacina translacije: qddot_world = (1/md)*Rot*Force + g_vec
 void calculateTranslation(const mjModel* m, mjData* d, double acc_vec[]){
 double eulerAngles[3] = {0};
 ToEulerAngles(m, d,eulerAngles);
 double alpha = eulerAngles[0], beta = eulerAngles[1], gama = eulerAngles[2]; 
// Rotx
   double rotx[9] = {1,0,0,0,cos(alpha), -sin(alpha), 0, sin(alpha), cos(alpha)};  
// Roty
   double roty[9] = {cos(beta), 0, sin(beta), 0, 1, 0, -sin(beta), 0, cos(beta)};
// Rotz 
  double rotz[9] = {cos(gama), -sin(gama), 0, sin(gama), cos(gama), 0, 0, 0, 1};
// Rot = Rz*Ry*Rx 
 double rot[9] =  {cos(beta)*cos(gama), -cos(alpha)*sin(gama)+sin(alpha)*sin(beta)*cos(gama), sin(alpha)*sin(gama)+cos(alpha)*sin(beta)*cos(gama), 
                   cos(beta)*sin(gama), cos(alpha)*cos(gama)+sin(alpha)*sin(beta)*sin(gama), -sin(alpha)*cos(gama)+cos(alpha)*sin(beta)*sin(gama), 
                   -sin(beta), sin(alpha)*cos(beta), cos(alpha)*cos(beta)};

// 1/md * Rot 
double md = mj_getTotalmass(m);
int i; 
for(i=0; i<9;i++){
  rot[i] = rot[i]/md; 
}
// model sile propelera 
double Fm = 1, F; 
F = Fm*(d->ctrl[0]+d->ctrl[1]+d->ctrl[2]+d->ctrl[3]); 
// vektor sile 
double force[3] = {0, 0, F};
// vektor gravitacijskog ubrzanja 
double g_vec[3] = {0, 0, -9.81}; 
double res[3]= {0}; 
mju_mulMatVec(res, rot, force, 3, 3); // clan 1/md*R*force
for(i=0; i<3;i++){
 acc_vec[i] = res[i] + g_vec[i];
}

}
//****************************

void mycontroller(const mjModel* m, mjData* d)
{
  //write control here
  mj_energyPos(m,d);
  mj_energyVel(m,d);
  //printf("%f %f %f %f \n",d->time,d->energy[0],d->energy[1],d->energy[0]+d->energy[1]);

  double qddot[ndof]={0};
  qddot[0]=d->qacc[0];  // ubrzanje pozicije
  qddot[1]=d->qacc[1];
  qddot[2]=d->qacc[2];
  qddot[3]=d->qacc[3];  // ugaono ubrzanje 
  qddot[4]=d->qacc[4];
  qddot[5]=d->qacc[5];
  double T[3] = {0}; 
 torqueVector( m, d, T);
  int i; 
 // for(i=0; i<3; i++) 
 // {
 //   printf("%f ", qddot[i]);
  //} 
  //printf("\n");
 // printf("******\n"); 
 // printf("%f %f %f %f %f %f %f \n", d->qpos[0],d->qpos[1] ,d->qpos[2], d->qpos[3], d->qpos[4], d->qpos[5], d->qpos[6]);
 // printf("******\n"); 
// ukupna sika  
//   double F, Fm = 100;
//   F = (d->ctrl[0] + d->ctrl[1] + d->ctlr[2] + d->ctrl[3])*Fm; 
  double crt = 0.72970, act_old=0, act_new=0;
 double timestep = 1.0/60.0, tau = 2.7; 
 act_new = crt*(1-exp(-d->time/tau)); 
 d->ctrl[0] = crt; 
 d->ctrl[1] = crt; 
 d->ctrl[2] = crt; 
  d->ctrl[3] = crt;
  act_old = act_new; 
  double ctr[na] = {0.75, 0.75, 0.75, 0.75};
  double acc_vec[3] = {0}; 
  calculateTranslation(m,d,acc_vec);  
  printf("%f %f %f %f\n", acc_vec[0],  acc_vec[1] , acc_vec[2], d->qacc[2]);
  printf("******\n"); 
 // actuatorModel(m,d,ctr);

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

    // char xmlpath[100]={};
    // char datapath[100]={};
    //
    // strcat(xmlpath,path);
    // strcat(xmlpath,xmlfile);
    //
    // strcat(datapath,path);
    // strcat(datapath,datafile);


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
    mjv_makeScene(m, &scn, 2000);                // space for 2000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_150);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    double arr_view[] = {89.608063, -11.588379, 5, 0.000000, 0.000000, 2.000000};
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

   // d->qpos[0] = 0.5;
    //d->qpos[1] = 0;
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
        //printf("{%f, %f, %f, %f, %f, %f};\n",cam.azimuth,cam.elevation, cam.distance,cam.lookat[0],cam.lookat[1],cam.lookat[2]);

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
