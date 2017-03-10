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

/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <time.h>
#include "util.h"

float r_rule2;		    // see updateBoid()
float r_rule3;
float k_rule2;
float k_rule3;
float k_rule0;
float v_gravity;



void updateBoid(int i){
	Point3D new_velocity;

	/*
	//note rule1 (fly towards the mass center) is not applied
	//rule 2: Boids steer to avoid collision.
	Point3D repel_vec;
	for(int j = 0; j < nBoids; ++j){
		if(j==i) continue;
		
		float dis = Boid_Location[i].distance_to(Boid_Location[j]);
		//each boid has a radius around 3
		// minus 2r from the distance between 2 boids
		//dis = dis - 6.0;
		if(dis > r_rule2) continue;
	
		repel_vec += (Boid_Location[j] - Boid_Location[i]);
	}
	new_velocity -= (repel_vec*k2);

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
	new_velocity += (avg_vec*k_rule3);

	//my rule: the boid (snow) has a constant -z velocity due to the gravity
	new_velocity.addz(v_gravity);

	// Paco's Rule Zero,
	//Boid_Velocity[i] += new_velocity*k_rule0;

	//update the position
	Boid_Location[i] += (Boid_Velocity[i]*speed_ratio1);
	*/
	new_velocity.addz(v_gravity);
	Boid_Location[i] += Boid_Velocity[i];
}


//This function draws a boid i at the specified location.
void drawBoid(int i){
	// Save current transformation matrix
	glPushMatrix();

	//update Boid location
	glTranslatef(Boid_Location[i].x(), Boid_Location[i].y(), Boid_Location[i].z());

	GLUquadric * my_quad = gluNewQuadric();
	gluSphere(my_quad, 1, 10, 10);

	gluDeleteQuadric(my_quad);
	glPopMatrix();			
}
