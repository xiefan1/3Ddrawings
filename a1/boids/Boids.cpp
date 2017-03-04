/***********************************************************
             CSC418 - St George , Winter 2017 version

                     Boids.cpp

	This assignment will help you become familiar with
	the basic structure and shape of an OpenGL program.

	Please take time to read through the code and note
	how the viewing parameters, viewing volume, and
	general OpenGL options are set. You will need to
	change those in future assignments.

	You should also pay attention to the way basic
	OpenGL drawing commands work and what they do.
        You should check the OpenGL reference manual
        for the different OpenGL functions you will
        find here. In particular, those that set up
        the viewing parameters.

	Note that this program is intended to display
	moving objects in real time. As such, it is
	strongly recommended that you run this code locally,
	on one of the machines at the CS lab. Alternately,
	install the OpenGL libraries on your own computer.

	Working remotely over ssh, or working on a non-
	Linux machine will give you headaches.

    Instructions:

	The assignment handout contains a detailed
	description of what you need to do. Please be
	sure to read it carefully.

	You must complete all sections marked
        // TO DO

	In addition to this, you have to complete
	all information requested in the file called
	REPORT.TXT. Be sure to answer in that
	report any
	// QUESTION:
	parts found in the code below.

	Sections marked
	// CRUNCHY:
	Are worth extra credit. How much bonus you get
	depends on the quality of your extensions
	or enhancements. Be sure to indicate in your
	REPORT.TXT any extra work you have done.

	The code is commented, and the comments provide
	information about what the program is doing and
	what your task will be.

	As a reminder. Your code must compile and run
	on 'mathlab.utsc.utoronto.ca' under Linux. We will
	not grade code that fails to compile or run
	on these machines.

Written by: F. Estrada, Jun 2011.
            Main loop/init derived from older 418
            OpenGL assignments
            Updated, Jan. 2017
***********************************************************/

/*
  Headers for 3DS management - model loading for point clouds
*/
#include<lib3ds/types.h>
#include<lib3ds/mesh.h>
#include<lib3ds/file.h>

/*
   Headers for OpenGL libraries. If you want to run this
   on your computer, make sure you have installed OpenGL,
   GLUT, and GLUI

   NOTE: The paths below assume you're working on mathlab.
   On your system the libraries may be elsewhere. Be sure
   to check the correct location for include files and
   library files (you may have to update the Makefile)
*/
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glut.h>
#include <GL/glui.h>

/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "util.h"

#define numTrace 100
#define MaxLeaders 2
#define NUM_OBST 0

// *************** GLOBAL VARIABLES *************************
int nBoids;				// Number of boids to dispay
int nLeaders;
int leader_id[MaxLeaders];
Point3D Boid_Location[MAX_BOIDS];	// Pointers to dynamically allocated
Point3D Boid_Velocity[MAX_BOIDS];	// Boid position & velocity data
Point3D Boid_trace[MAX_BOIDS][numTrace];
int Boid_clock[MAX_BOIDS];
float Boid_scaleY[MAX_BOIDS];
float Boid_Color[MAX_BOIDS][3];	 	// RGB colour for each boid

Point3D Obstacle[NUM_OBST];
// 1 -- postive y
//-1 -- negative y
int Obstacle_dir[NUM_OBST];
int Obstacle_clock[NUM_OBST];

float *modelVertices;                   // Imported model vertices
int n_vertices;                         // Number of model vertices

// *************** USER INTERFACE VARIABLES *****************
int windowID;               // Glut window ID (for display)
GLUI *glui;                 // Glui window (for controls)
int Win[2];                 // window (x,y) size
float r_rule1;		    // Parameters of the boid update function.
float r_rule2;		    // see updateBoid()
float r_rule3;
float k_rule1;
float k_rule2;
float k_rule3;
float k_rule0;
float shapeness;
float global_rot[16]; //4 by 4 matrix
float zoomf = 1.0;
float clock_scale;

float speed_ratio1 = 0.1;

// ***********  FUNCTION HEADER DECLARATIONS ****************
// Initialization functions
void initGlut(char* winName);
void initGlui();
void GL_Settings_Init();
float *read3ds(const char *name, int *n);

// Callbacks for handling events in glut
void WindowReshape(int w, int h);
void WindowDisplay(void);
void WindowClick(int button, int state, int x, int y);

bool mouse_click;
clock_t click_start;
Point3D click_world;

// Callback for handling events in glui
void GLUI_Control(int id);

// Return the current system clock (in seconds)
double getTime();

//functions of drawing boids
void updateBoid(int i);
void drawBoid(int i);
void drawTrace(int i);

//functions of drawing obstacles
void drawObstacle(int i);
void updateObst(int i);

void HSV2RGB(float H, float S, float V, float *R, float *G, float *B);
void draw_halfSphere();
void draw_jellyFish(int i);
void draw_revolution(float *r, float *y, int num, GLenum, struct Color* color_base, float, float);
void draw_circle();


/* color hsv.h */
float AquaH = 0.49;
float PinkH = 0.84;
float YellowH = 0.16;
float Green = 0.35;
float YGP_H[3] = {0.65, 0.1, 0.3};


// ******************** FUNCTIONS ************************

/*
   main()

   Read command line parameters, initialize boid positions
   and velocities, and initialize all OpenGL data needed
   to set up the image window. Then call the GLUT main loop
   which handles the actual drawing
*/
int main(int argc, char** argv)
{
    // Process program arguments
    if(argc < 4 || argc > 5) {
        fprintf(stderr,"Usage: Boids width height nBoids [3dmodel]\n");
        fprintf(stderr," width & height control the size of the graphics window\n");
        fprintf(stderr," nBoids determined the number of Boids to draw.\n");
        fprintf(stderr," [3dmodel] is an optional parameter, naming a .3ds file to be read for 3d point clouds.\n");
        exit(0);
    }
    Win[0]=atoi(argv[1]); //width
    Win[1]=atoi(argv[2]); //height
    nBoids=atoi(argv[3]);

    //initialize leader id
    if (nBoids < 4)
	nLeaders = MaxLeaders-1;
    else nLeaders = MaxLeaders;
    
    for(int i=0; i<nBoids && i<nLeaders; ++i){
	leader_id[i] = i;
    }

    if (nBoids>MAX_BOIDS)
    {
     fprintf(stderr,"Too Many Boids! Max=%d\n",MAX_BOIDS);
     exit(0);
    }

    // If a model file is specified, read it, normalize scale
    n_vertices=0;
    modelVertices=NULL;
    if (argc==5)
    {
     float mx=0;
     n_vertices=nBoids;
     modelVertices=read3ds(argv[4],&n_vertices);
     if (n_vertices>0)
     {
      fprintf(stderr,"Returned %d points\n",n_vertices);
      for (int i=0; i<n_vertices*3; i++)
       if (fabs(*(modelVertices+i))>mx) mx=fabs(*(modelVertices+i));
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)/=mx;
      for (int i=0; i<n_vertices*3; i++) *(modelVertices+i)*=(SPACE_SCALE*.5);
     }
    }

    // Initialize Boid positions and velocity
    // Mind the SPEED_SCALE. You may need to change it to
    // achieve smooth animation - increase it if the
    // animation is too slow. Decrease it if it's too
    // fast and choppy.
    srand48(1522); //seed the random number generator
    for (int i=0; i<nBoids; i++)
    {
     // Initialize Boid locations and velocities randomly
     Boid_Location[i].set_x((-.5+drand48())*SPACE_SCALE);
     Boid_Location[i].set_y((-.5+drand48())*SPACE_SCALE);
     Boid_Location[i].set_z((-.5+drand48())*SPACE_SCALE);
     Boid_Velocity[i].set_x((-.5+drand48())*SPEED_SCALE);
     Boid_Velocity[i].set_y((-.5+drand48())*SPEED_SCALE);
     Boid_Velocity[i].set_z((-.5+drand48())*SPEED_SCALE);

     // Initialize Boid trace to its location
     for( int j = 0; j <numTrace; ++j){ 
	     Boid_trace[i][j] = Boid_Location[i];
     }

     // Initialize boid colour to solid blue-ish
     // You may want to change this
     Boid_clock[i] = clock();
     Boid_scaleY[i] = 1.0;
     Boid_Color[i][0]=.15;
     Boid_Color[i][1]=.15;
     Boid_Color[i][2]=1;
    }

    for(int i=0; i<NUM_OBST; ++i){
     //initialize obstacle positions randomly
     Obstacle[i].set_x((-.5+drand48())*SPACE_SCALE);
     Obstacle[i].set_y((-.5+drand48())*SPACE_SCALE);
     Obstacle[i].set_z((-.5+drand48())*SPACE_SCALE);
     Obstacle_dir[i] = (i%2)?1:-1;
     Obstacle_clock[i] = clock();
    }

    //initialize clock scale
    //to slow the animation as the number of boids increase
    clock_scale = (float)nBoids/100.0;
    if(clock_scale>1) clock_scale = 1.0;

    // Initialize glut, glui, and opengl
    glutInit(&argc, argv);
    initGlut(argv[0]);
    initGlui();
    GL_Settings_Init();

    // Initialize variables that control the boid updates
    r_rule1=15;
    r_rule2=8;
    r_rule3=25;
    k_rule1=.15;
    k_rule2=.5;
    k_rule3=.15;
    k_rule0= 0.5;
    shapeness=0;

    mouse_click = 0;

    // Invoke the standard GLUT main event loop
    glutMainLoop(); //event processing loop
    exit(0);         // never reached
}

// Initialize glut and create a window with the specified caption
void initGlut(char* winName)
{
    // Set video mode: double-buffered, color, depth-buffered
    glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    // We will learn about all of these later, for now, what you
    // need to know is that with the settings above, the graphics
    // window will keep track of the depth of objects so that
    // objects in the back can be properly obscured by objects
    // in front of them. The double buffer is used to ensure
    // smooth-looking animation.

    // Create window
    glutInitWindowPosition (0, 0);
    glutInitWindowSize(Win[0],Win[1]);
    windowID = glutCreateWindow(winName);

    // Setup callback functions to handle window-related events.
    // In particular, OpenGL has to be informed of which functions
    // to call when the image needs to be refreshed, and when the
    // image window is being resized.
    glutReshapeFunc(WindowReshape);   // Call WindowReshape whenever window resized
    glutDisplayFunc(WindowDisplay);   // Call WindowDisplay whenever new frame needed
    glutMouseFunc(WindowClick);
}

// Quit button handler.  Called when the "quit" button is pressed.
void quitButton(int)
{
  if (modelVertices!=NULL && n_vertices>0) free(modelVertices);
  exit(0);
}

// Initialize GLUI and the user interface
void initGlui()
{
    GLUI_Master.set_glutIdleFunc(NULL);

    // Create GLUI window
    glui = GLUI_Master.create_glui("Boid CSC 418 Window", 0, Win[0]+10, 0);

    ////////////////////////////////////////
    // LEARNING OBJECTIVES:
    //
    // This part of the assignment is meant to help
    // you learn about:
    //
    // - How to create simple user interfaces for your
    //   OpenGL programs using GLUI.
    // - Defining and using controllers to change
    //   variables in your code.
    // - Using the GUI to change the behaviour of
    //   your program and the display parameters
    //   used by OpenGL.
    //
    // Be sure to check on-line references for GLUI
    // if you want to learn more about controller
    // types.
    //
    // See: http://www.eng.cam.ac.uk/help/tpl/graphics/using_glui.html
    //
    //   Add controls to change the values of the variables
    //
    //   - k_rule1, k_rule2, k_rule3, k_rule0
    //   - r_rule2, r_rule3
    //
    //   Ranges for these variables are given in the updateBoid()
    //   function. Make sure to set the increments to a reasonable
    //   value.
    //
    ///////////////////////////////////////////////////////////

      float spin_speed = 1.0;
      GLUI_Spinner *r1_spinner
          = glui->add_spinner("r_rule1", GLUI_SPINNER_FLOAT, &r_rule1);
      GLUI_Spinner *r2_spinner
          = glui->add_spinner("r_rule2", GLUI_SPINNER_FLOAT, &r_rule2);
      GLUI_Spinner *r3_spinner
          = glui->add_spinner("r_rule3", GLUI_SPINNER_FLOAT, &r_rule3);

      GLUI_Spinner *k0_spinner
          = glui->add_spinner("k_rule0", GLUI_SPINNER_FLOAT, &k_rule0);
      GLUI_Spinner *k1_spinner
          = glui->add_spinner("k_rule1", GLUI_SPINNER_FLOAT, &k_rule1);
      GLUI_Spinner *k2_spinner
          = glui->add_spinner("k_rule2", GLUI_SPINNER_FLOAT, &k_rule2);
      GLUI_Spinner *k3_spinner
          = glui->add_spinner("k_rule3", GLUI_SPINNER_FLOAT, &k_rule3);

      r1_spinner->set_speed(spin_speed);
      r2_spinner->set_speed(spin_speed);
      r3_spinner->set_speed(spin_speed);
      k0_spinner->set_speed(spin_speed);
      k1_spinner->set_speed(spin_speed);
      k2_spinner->set_speed(spin_speed);
      k3_spinner->set_speed(spin_speed);
      r1_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
      r2_spinner->set_float_limits(1, 15, GLUI_LIMIT_CLAMP);
      r3_spinner->set_float_limits(10, 100, GLUI_LIMIT_CLAMP);
      k0_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
      k1_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
      k2_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);
      k3_spinner->set_float_limits(0, 1, GLUI_LIMIT_CLAMP);



//    Add rotation
      GLUI_Rotation * rot = glui->add_rotation("rotation", global_rot);
      rot->reset(); //reset to identity matrix
      float damping_factor = 0.98;
      rot->set_spin(damping_factor);

    // Add "Quit" button
    glui->add_separator();
    glui->add_button("Quit", 0, quitButton);

    // Set the main window to be the "active" window
    glui->set_main_gfx_window(windowID);
}

/*
  Reshape callback function. Takes care of handling window resizing
  events.
  Parameters are the new window size in pixels.
*/
void WindowReshape(int w, int h)
{
    // Setup projection matrix for new window

    // We will learn about projections later on. The projection mode
    // determines how 3D objects are 'projected' onto the 2D image.

    // Most graphical operations in OpenGL are performed through the
    // use of matrices. Below, we let OpenGL know that we will be
    // working with the GL_PROJECTION matrix, which controls the
    // projection of objects onto the image.
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();			// Initialize with identity matrix

    // We will use perspective projection. This simulates a simple
    // pinhole camera.

    // The line below specifies the general shape and properties of
    // the viewing volume, that is, the region of space that is
    // visible within the image window.
    gluPerspective(45,1,15,500);
    //              ^ ^  ^  ^
    // FOV ---------| |  |  |		// See OpenGL reference for
    // Aspect Ratio --|  |  |		// more details on using
    // Near plane -------|  |		// gluPerspective()
    // Far plane -----------|

    // Set the OpenGL viewport - this corresponds to the 2D image
    // window. It uses pixels as units. So the instruction below
    // sets the image window to start at pixel coordinate (0,0)
    // with the specified width and height.
    glViewport(0,0,w,h);
    Win[0] = w;
    Win[1] = h;

    // glutPostRedisplay()	// Is this needed?
}

void GL_Settings_Init()
{
 // Initialize OpenGL parameters to be used throughout the
 // life of the graphics window

    // Set the background colour
    glClearColor(0.01f,0.01f,0.01f,1.0f);

    // Enable alpha blending for transparency
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

/**** Illumination set up start ****/

    // The section below controls the illumination of the scene.
    // Nothing can be seen without light, so the section below
    // is quite important. We will discuss different types of
    // illumination and how they affect the appearance of objecs
    // later in the course.
    glClearDepth(1);
    glEnable(GL_DEPTH_TEST);    // Enable depth testing
    glEnable(GL_LIGHTING);      // Enable lighting
    glEnable(GL_LIGHT0);        // Enable LIGHT0 for diffuse illumination
    glEnable(GL_LIGHT1);        // Enable LIGHT1 for ambient illumination

    // Set up light source colour, type, and position
    GLfloat light0_colour[]={1.0,1.0,1.0};
    GLfloat light1_colour[]={.25,.25,.25};
    GLfloat light0_pos[]={500,0,500,0};
    glLightfv(GL_LIGHT0,GL_DIFFUSE,light0_colour);
    glLightfv(GL_LIGHT1,GL_AMBIENT,light1_colour);
    glLightfv(GL_LIGHT0,GL_POSITION,light0_pos);
    glShadeModel(GL_SMOOTH);

    // Enable material colour properties
    glEnable(GL_COLOR_MATERIAL);

/***** Illumination setup end ******/
}

/*
   Display callback function.
   This has to be called whenever the image needs refreshing,
   either as a result of updates to the graphical content of
   the window (e.g. animation is taking place), or as a
   result of events outside this window (e.g. other programs
   may create windows that partially occlude the OpenGL window,
   when the occlusion ends, the obscured region has to be
   refreshed)
*/
void WindowDisplay(void)
{
    // Clear the screen and depth buffers
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

/***** Scene drawing start *********/

    // Here is the section where shapes are actually drawn onto the image.
    // In this case, we call a function to update the positions of boids,
    // and then draw each boid at the updated location.

    // Setup the model-view transformation matrix
    // This is the matrix that determines geometric
    // transformations applied to objects. Typical
    // transformations include rotations, translations,
    // and scaling.
    // Initially, we set this matrix to be the identity
    // matrix.
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    
    // The line below specifies the position and orientation of the
    // camera, as well as the direction it's pointing at.
    // The first three parameters are the camera's X,Y,Z location
    // The next three specify the (x,y,z) position of a point
    // the camera is looking at.
    // The final three parameters specify a vector that indicates
    // what direction is 'up'
    gluLookAt(125,75,100,0,0,0,0,1,0);

    //Rotate the world by user input angle
    //Important: the transpose of rotation matrix is the same
    //except that the direction reverses.
    glMultMatrixf(global_rot);
    //zoom factor specified by user
    glScalef(zoomf, zoomf, zoomf);

    // Draw box bounding the viewing area
    glColor4f(.95,.95,.95,.95);
    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,-50,50);
     glVertex3f(-50,50,50);
     glVertex3f(-50,50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,-50,50);
     glVertex3f(50,-50,50);
     glVertex3f(50,-50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(-50,-50,-50);
     glVertex3f(-50,50,-50);
     glVertex3f(50,50,-50);
     glVertex3f(50,-50,-50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,50,-50);
     glVertex3f(50,-50,-50);
     glVertex3f(50,-50,50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,50,-50);
     glVertex3f(-50,50,-50);
     glVertex3f(-50,50,50);
    glEnd();

    glBegin(GL_LINE_LOOP);
     glVertex3f(50,50,50);
     glVertex3f(50,-50,50);
     glVertex3f(-50,-50,50);
     glVertex3f(-50,50,50);
    glEnd();

    for(int i=0; i<NUM_OBST; i++){
      updateObst(i);
      drawObstacle(i);
    }

    for (int i=0; i<nBoids; i++)
    {
     updateBoid(i);		// Update position and velocity for boid i
     drawTrace(i);
     drawBoid(i);		// Draw this boid
    }

    // Make sure all OpenGL commands are executed
    glFlush();

    // Swap buffers to enable smooth animation
    glutSwapBuffers();
/***** Scene drawing end ***********/

  // update user interface to synchronize variable changes
  glui->sync_live();

  // Tell glut window to update itself
  glutSetWindow(windowID);
  //This call queues the redisplay event therefore enable animation
  glutPostRedisplay();
}


//This function updates the position and velocity of Boid i
// Reference: http://www.vergenet.net/~conrad/boids/pseudocode.html
void updateBoid(int i)
{	
	/* Extra feature */
	/* Clicking on the screen to scatter the flock by making k1 negative */
	float k1 = k_rule1;
	float k2 = k_rule2;
	//rule3 doesn't apply to the leader boid

	if(mouse_click){
		if(50 > abs(click_world.x() - Boid_Location[i].x()) &&
			50 > abs(click_world.y() - Boid_Location[i].y())){
			clock_t time = clock();
	
			if((time - click_start) <= CLOCKS_PER_SEC)
				k1 = - k_rule1;
	
			//gradually come back to the mass center
			else if((time - click_start) <= 2*CLOCKS_PER_SEC)
				k1 = 0.5*k_rule1;
			else mouse_click = false;
		}
	}
	
	
	Point3D new_velocity;



	if( i > leader_id[nLeaders-1]){

		//rule 1: Boids fly towards the center of mass
		Point3D mass_center;
		vector_to_massCenter(i, mass_center, r_rule1);
		new_velocity += (mass_center*k1);


		//Crunchy: the nearest leader has larger weight
		float nearest_leader = 10000.0;
		int leader=0;

		//find the nearest leader
		for(int j = 0; j < nLeaders; j++){
			//leader index
			int k = leader_id[j];
			float disL = Boid_Location[i].distance_to(Boid_Location[k]);
			if(disL < nearest_leader){
				nearest_leader = disL;
				leader = k;
			}
		}
			
	
		//rule 3: Boids try to match speed with neighbours
		Point3D avg_vec;
		for(int j = 0; j < nBoids; ++j){
			if(j==i) continue;
			
			float dis = Boid_Location[i].distance_to(Boid_Location[j]);
			if(dis > r_rule3) continue;
		
			avg_vec += Boid_Velocity[j];
		}
		if(nBoids != 1)
		avg_vec /= (nBoids-1);

		//leader has larger weght
		avg_vec += Boid_Velocity[leader];
		avg_vec /=2; 
		new_velocity += (avg_vec*k_rule3);



		//fly towards the nearest leader
		Point3D vector_to_leader = Boid_Location[leader] - Boid_Location[i];
		new_velocity += vector_to_leader*0.1;;

	}else{
		//this is a leader boid
		//apply rule 1: Boids fly towards the center of mass
		Point3D mass_center;
		vector_to_massCenter(i, mass_center, r_rule1*0.5);
		new_velocity += (mass_center*k1);
		k2 = k2/2;
	}
	

	//rule 2: Boids steer to avoid collision.
	Point3D repel_vec;
	for(int j = 0; j < nBoids; ++j){
		if(j==i) continue;
		
		float dis = Boid_Location[i].distance_to(Boid_Location[j]);
		//each boid has a radius around 3
		// minus 2r from the distance between 2 boids
		dis = dis - 6.0;
		if(dis > r_rule2) continue;
	
		repel_vec += (Boid_Location[j] - Boid_Location[i]);
	}
	new_velocity -= (repel_vec*k2);


	 
	//  Enforcing bounds on motion
	//
	//  This is already implemented: The goal
	// is to ensure boids won't stray too far
	// from the viewing area.
	if (Boid_Location[i].x()<-50) Boid_Velocity[i].addx(1.0);
	if (Boid_Location[i].x()>50) Boid_Velocity[i].addx(-1.0);
	if (Boid_Location[i].y()<-50) Boid_Velocity[i].addy(1.0);
	if (Boid_Location[i].y()>50) Boid_Velocity[i].addy(-1.0);
	if (Boid_Location[i].z()<-50) Boid_Velocity[i].addz(1.0);
	if (Boid_Location[i].z()>50) Boid_Velocity[i].addz(-1.0);
	
	///////////////////////////////////////////
	// CRUNCHY: Add a 'shapeness' component.
	//  this should give your Boids a tendency
	//  to hover near one of the points of a
	//  3D model imported from file. Evidently
	//  each Boid should hover to a different
	//  point, and you must figure out how to
	//  make the void fly toward that spot
	//  and hover more-or-less around it
	//  (depending on all Boid parameters
	//   for the above rules). 
	//
	//  3D model data is imported for you when 
	//  the user specifies the name of a model
	//  file in .3ds format from the command
	//  line. 
	//
	//  The model data
	//  is stored in the modelVertices array
	//  and the number of vertices is in
	//  n_vertices (if zero, there is no model
	//  and the shapeness component should
	//  have no effect whatsoever)
	//
	//  The coordinates (x,y,z) of the ith
	//  mode, vertex can be accessed with
	//  x=*(modelVertices+(3*i)+0);
	//  y=*(modelVertices+(3*i)+1);
	//  z=*(modelVertices+(3*i)+2);
	//
	//  Evidently, if you try to access more
	//  points than there are in the array
	//  you will get segfault (don't say I
	//  didn't warn you!). Be careful with
	//  indexing.
	//
	//  shapeness should be in [0,1], and
	//  there is already a global variable
	//  to store it. You must add a slider
	//  to the GUI to control the amount of
	//  shapeness (i.e. how strong the shape
	//  constraints affect Boid position).
	//
	//  .3ds models can be found online, you
	//  *must ensure* you are using a freely
	//  distributable model!
	//
	//////////////////////////////////////////
	
	// Velocity Limit:
	Boid_Velocity[i].sqrt_xyz();
	
	// Paco's Rule Zero,
	Boid_Velocity[i] += new_velocity*k_rule0;
	
	///////////////////////////////////////////
	// QUESTION: Why add inertia at the end and
	//  not at the beginning?
	///////////////////////////////////////////

	//update the trace
	for(int j=numTrace-1; j>0; --j){
		Boid_trace[i][j] = Boid_trace[i][j-1];
	}
	Boid_trace[i][0] = Boid_Location[i];

	
	// Finally (phew!) update the position
	// of this boid.
	//Each boid has at least base_r = 3.0
	//Distance between boids >=  2*base_r

	Boid_Location[i] += (Boid_Velocity[i]*speed_ratio1);
	
	#ifdef DEBUG
	//std::cout<<Boid_Velocity[i];
	#endif
	
	
	 ///////////////////////////////////////////
	 // CRUNCHY:
	 //
	 //  Things you can add here to make the behaviour
	 // more interesting. Be sure to note in your
	 // report any extra work you have done.
	 //
	 // - Add a few obstacles (boxes or something like it)
	 //   and add code to have boids avoid these
	 //   obstacles
	 //
	 // - Follow the leader: Select a handful
	 //   (1 to 5) boids randomly. Add code so that
	 //   nearby boids tend to move toward these
	 //   'leaders'
	 //
	 // - Make the updates smoother: Idea, instead
	 //   of having hard thresholds on distances for
	 //   the update computations (r_rule1, r_rule2,
	 //   r_rule3), use a weighted computation
	 //   where contributions are weighted by
	 //   distance and the weight decays as a
	 //   function of the corresponding r_rule
	 //   parameter.
	 //
	 // - Add a few 'predatory boids'. Select
	 //   a couple of boids randomly. These become
	 //   predators and the rest of the boids
	 //   should have a strong tendency to
	 //   avoid them. The predatory boids should
	 //   follow the standard rules. However,
	 //   Be sure to plot the predatory boids
	 //   differently so we can easily see
	 //   who they are.
	 //
	 // - Make it go FAST. Consider and implement
	 //   ways to speed-up the boid update. Hint:
	 //   good approximations are often enough
	 //   to give the right visual impression.
	 //   What and how to approximate? that is the
	 //   problem.
	 //
	 //   Thoroughly describe any crunchy stuff in
	 //   the REPORT.
	 //
	 ///////////////////////////////////////////
	
	 return;
}


/*  float shearMatrix[16] = {1, 0, 0, 0,
			  -0.1, 3, -0.1, 0,
			  0, 0, 1, 0,
			  0, 0, 0, 1}; //column major matrix
*/

void updateObst(int i){
	float time_scale = clock_scale + 0.01*(float)i;
	if(clock() - Obstacle_clock[i] > time_scale*CLOCKS_PER_SEC){
		Obstacle[i].addy(Obstacle_dir[i]);
		//reverse the direction
		Obstacle_dir[i]*= -1;
		Obstacle_clock[i] = clock();
	}
}


void drawObstacle(int i){
	glPushMatrix();
	glTranslatef(Obstacle[i].x() , Obstacle[i].y(), Obstacle[i].z());

/*	float rgb[4];
	rgb[3] = 0.6; //alpa value
	float hsv[3] = {338, 100, 63};
	HSV2RGB(hsv[0], hsv[1], hsv[2], &rgb[0], &rgb[1], &rgb[2]);
	glColor4fv(rgb);
*/
	glColor4f((float)13/256, (float)94/256, (float)5/256, 1.0);
	GLUquadric* obst = gluNewQuadric();
	if(!obst) return;

	float r = (i%3+1)*3;;
	gluSphere(obst, r, 4, 4);

	gluDeleteQuadric(obst);
	glPopMatrix();

}

//This function draws a boid i at the specified location.
void drawBoid(int i){

	// Save current transformation matrix
	glPushMatrix();

	//update Boid location
	glTranslatef(Boid_Location[i].x(), Boid_Location[i].y(), Boid_Location[i].z());

	draw_jellyFish(i);

	// Restore transformation matrix so it's
	// ready for the next boid.
	glPopMatrix();			
}

//Add trails that show the last
//few positions of each boid
void drawTrace(int id){
	/* draw trace with trangles */
	float angley =10; //degree
	float p1[3] = {-0.1, 0.0, 0.0};
	float p2[3] = {0.2, 0.0, 0.1};
	float p3[3] = {0.0, -0.4, 0.0};

	//glBegin(GL_LINE_STRIP);


	for(int i= 3; i<numTrace; i+=2){
		float f = i;
		glColor4f(1.0, 1.0, 25.0/f, 0.8 - f*0.016);
		//glVertex3fv(Boid_trace[id][i].get());

		//draw trace with trangles -- not anymore
		glPushMatrix();

		glTranslatef(Boid_trace[id][i].x(), Boid_trace[id][i].y(),
				Boid_trace[id][i].z());
		glRotatef(angley, 0, 1,0);
		glScalef(2.0, 2.0, 2.0);

		//draw traingle
		glBegin(GL_TRIANGLES);
		glVertex3fv(p1);
		glVertex3fv(p2);
		glVertex3fv(p3);
		glEnd();

		glPopMatrix();
	}
}




void HSV2RGB(float H, float S, float V, float *R, float *G, float *B)
{
 // Handy function to convert a colour specified as an HSV triplet
 // to RGB values used by OpenGL. You can use this function to
 // set the boids' colours in a more intuitive way. To learn
 // about HSV colourspace, check the Wikipedia page.
 float c,x,hp,r1,g1,b1,m;

 hp=H*6;
 c=V*S;
 x=c*(1.0-fabs(fmod(hp,2)-1.0));
 if (hp<1){r1=c;g1=x;b1=0;}
 else if (hp<2){r1=x;g1=c;b1=0;}
 else if (hp<3){r1=0;g1=c;b1=x;}
 else if (hp<4){r1=0;g1=x;b1=c;}
 else if (hp<5){r1=x;g1=0;b1=c;}
 else{r1=c;g1=0;b1=x;}

 m=V-c;
 *R=r1+m;
 *G=g1+m;
 *B=b1+m;

 if (*R>1) *R=1;
 if (*R<0) *R=0;
 if (*G>1) *G=1;
 if (*G<0) *G=0;
 if (*B>1) *B=1;
 if (*B<0) *B=0;
}

float *read3ds(const char *name, int *n)
{
 /*
   Read a model in .3ds format from the specified file.
   If the model is read successfully, a pointer to a
   float array containing the vertex coordinates is
   returned.

   Input parameter n is used to specify the maximum
   number of vertex coordinates to return, as well
   as to return the actual number of vertices read.


   Vertex coordinates are stored consecutively
   so each vertex occupies in effect 3 consecutive
   floating point values in the returned array
 */
 Lib3dsFile *f;
 Lib3dsMesh *mesh;
 Lib3dsPoint *pt;
 int n_meshes, n_points;
 Lib3dsVector *vertex_data;
 float *vertices, *v_return;
 float inc_step;
 int idx;

 f=lib3ds_file_load(name);
 if (f==NULL)
 {
  fprintf(stderr,"Unable to load model data\n");
  *n=0;
  return(NULL);
 }

 // Count meshes and faces
 n_points=0;
 n_meshes=0;
 mesh=f->meshes;
 while(mesh!=NULL)
 {
  n_meshes++;
  n_points+=mesh->points;
  mesh=mesh->next;
 }
 fprintf(stderr,"Model contains %d meshes, %d points, %d coordinates\n",n_meshes,n_points,3*n_points);

 // Allocate data for vertex array and put all input points (from all meshes) in the array
 vertex_data=(Lib3dsVector *)calloc(n_points,sizeof(Lib3dsVector));
 mesh=f->meshes;
 while(mesh!=NULL)
 {
  pt=mesh->pointL;
  for (int i=0; i < mesh->points; i++)
    memcpy((vertex_data+i),(pt+i),sizeof(Lib3dsVector));
  mesh=mesh->next;
 }

 vertices=(float *)vertex_data;
 // Release memory allocated to the file data structure and return the vertex array
 lib3ds_file_free(f);

 if (n_points<(*n)) *(n)=n_points;                      // Less points than expected!
 v_return=(float *)calloc((*(n))*3,sizeof(float));      // Allocate space for n points
 inc_step=(n_points-1)/(*n);                            // Sampling step
 for (int i=0; i<(*(n)); i++)
 {
  idx=floor(inc_step*i);
  *(v_return+(3*i)+0)=*(vertices+(3*idx)+0);            // Mind the ordering! it's shuffled
  *(v_return+(3*i)+1)=*(vertices+(3*idx)+2);            // to match our coordinate frame
  *(v_return+(3*i)+2)=*(vertices+(3*idx)+1);
 }

 free(vertex_data);
 return(v_return);
}

void WindowClick(int button, int state, int x, int y){
	//click window to scatter the flock
	if(button == GLUT_LEFT_BUTTON){
		static clock_t press_start;
		static int old_x, old_y;
		if(state != GLUT_DOWN){
			press_start = clock();
			old_x = x, old_y = y;

			if(x>Win[0] | y>Win[1]) return;
			//convert screen coords to world coords
			GLdouble modelview_matrix[16], projection_matrix[16];
			GLint viewport[4];
			glGetDoublev(GL_MODELVIEW_MATRIX, modelview_matrix);
			glGetDoublev(GL_PROJECTION_MATRIX, projection_matrix);
			glGetIntegerv(GL_VIEWPORT, viewport);
		
			//get the near depth
			GLfloat depth[2];
			GLdouble world_coords[3];
			glGetFloatv(GL_DEPTH_RANGE, depth);
		
			gluUnProject(x, y, depth[0], modelview_matrix, projection_matrix, viewport,
				&world_coords[0], &world_coords[1], &world_coords[2]);
		
			click_world.set((float)world_coords[0], world_coords[1], world_coords[2]);
			mouse_click = true;	
			click_start = clock();
	#ifdef DEBUG
			std::cout<<"click "<<click_world;
	#endif
		}else{  //mouse released
			clock_t press_end = clock();
			if(press_end - press_start > 500){
				


			}
		}

	}else if(button == 3){//scrolling up
		if(zoomf<5) zoomf +=0.08;
	}
	else if(button == 4) //scrolling down
		if(zoomf>0.09) zoomf -=0.08;
}



// id is used to select the color
void draw_jellyFish(int id){
	int i, num = 50;
	float base_r = 3.0;
	/* the base radius r (before any scaling)
	   y = Boid_scaleY[i]*base_r^2
	   0 < Boid_scaleY[i] < 1
	*/

	//update scale factor wrt time to get Boid animation
	if(clock() - Boid_clock[id] > clock_scale*CLOCKS_PER_SEC){
	       if(Boid_scaleY[id] <= 0.7) Boid_scaleY[id] = 1.0;
	       else Boid_scaleY[id] -= 0.2;
	       Boid_clock[id] = clock();
	}

	//array to store y, r coordinates
	float r[num], y[num];
	r[0] = y[0] = 0.0;
	for(i=1; i<num; ++i){
		r[i] = r[i-1] + 3.0/(float)num;
		y[i] = -Boid_scaleY[id]*r[i]*r[i];
	}


	glPushMatrix();
	glTranslatef(0, 9.0, 0);
	
	//shape update -- squishing animation
	glTranslatef(0, 3*Boid_scaleY[id], 0);
	glScalef(2.0-Boid_scaleY[id], 1.0, 2.0-Boid_scaleY[id]);


	//draw the legs in dots
	int start_index = num*0.8;
	glPushMatrix();
	glTranslatef(0, y[30],0); 
	glScalef(0.6, 1.0, 0.6);

	float rgb[3];
	struct Color color_base(0.54, 0.72, 1.0); //auqa
	if(id <= leader_id[nLeaders-1]) color_base.h = YGP_H[id];
	HSV2RGB(color_base.h, color_base.s, color_base.v, rgb, rgb+1, rgb+2); 
	glColor4f(rgb[0], rgb[1], rgb[2], 1.0);

	draw_revolution(&r[start_index], &y[start_index], num, GL_POINTS, NULL, 0, 0);
	glPopMatrix();


	//draw the inner cap/layer
	struct Color inner_base(color_base.h+0.4, 0.79, 1.0);
	draw_revolution(r, y, num, GL_QUAD_STRIP, &inner_base, -46, 0.13);

	//draw the outter cap
	glTranslatef(0, -y[num-1]*0.2,0); 
	glScalef(1.4, 1.3, 1.4);
	draw_revolution(r, y, num, GL_QUAD_STRIP, &color_base, 250, 0.11);

	//draw the shell in dots
	/*glTranslatef(0, -y[num-1]*0.1,0); 
	glColor4f(0.2, 1, 1, 0.5);
	draw_revolution(r, y, num, GL_POINTS, NULL, 0, 0);
	*/

	glPopMatrix();

}

// pass y and r coordinates into array
// this function draws a solid of revolution
// if color_base == 0, the caller already set the color
// h_scale: scale the chnaging of hsv.h
// note: this function assumes y is negative
void draw_revolution(float *r, float *y, int num, GLenum mode, struct Color* color_base, float h_scale, float alpha){
	float theta=0, delta=10;
	// change in y-dir
	float delta_i = 1;
	if(mode == GL_POINTS) {delta_i = 8; delta = 20;}
	for(int i=0; i<(num-1); i+= delta_i){
		glBegin(mode);
		//rotation in x-z plane
		for(theta=0;theta<=360;theta+=delta){
			float x1, x2, z1, z2;
			float _rx = cos(theta*D2R);
			float _rz = sin(theta*D2R);

			if(color_base != NULL){
				float rgb[3];
				HSV2RGB((color_base->h + y[i]*_rx/h_scale), color_base->s, (color_base->v - y[i]*_rx/25.0), rgb, rgb+1, rgb+2);
				glColor4f( rgb[0], rgb[1], rgb[2], 1.0 +  alpha*y[i]);
			}

			x1 = r[i]*_rx;
			x2 = r[i+1]*_rx;	
			z1 = r[i]*_rz;
			z2 = r[i+1]*_rz;
			glVertex3f(x1, y[i], z1);
			glVertex3f(x2, y[i+1], z2);
		}
		glEnd();
	}

}




void draw_circle(){
	glPushMatrix();
	float delta = 10, r = 10;
	float x = r, z = 0, theta;
	glBegin(GL_POLYGON);
	for(theta = 0; theta <= 360; theta+= delta){
		x = r*cos((theta*2*PI/360.0));
		z = r*sin(theta*2*PI/360);
		glVertex3f(x, 0, z);
	}
	glEnd();
	glPopMatrix();
}
