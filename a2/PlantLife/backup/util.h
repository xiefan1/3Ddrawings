/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <iostream>

// *************** GLOBAL VARIABLES *************************
#define MAX_BOIDS 2000
#define SPACE_SCALE 75
#define SPEED_SCALE 0.01
#define DEBUG
const float PI = 3.14159;
const float E = 2.71828182846;
const float D2R = 2*PI/360;
class Point3D;
struct Color;

// *************** GLOBAL VARIABLES *************************
extern Point3D Boid_Location[MAX_BOIDS];	// Pointers to dynamically allocated
extern Point3D Boid_Velocity[MAX_BOIDS];	// Boid position & velocity data


struct Color {
	float h, s, v;

	Color(float _h, float _s, float _v){
		h = _h;
		s = _s;
		v = _v;
	}
	Color(){
		h=s=v=0.0;
	}
};



//struct Color const RoyalBlue = {0.255, 0.412, 0.882};
//struct Color const DeepSkyBlue = {0.000, 0.749, 1.000};
//struct Color const Purple = {0.502, 0.000, 0.502};



// Functions for handling Boids
static float sign(float x){if (x>=0) return(1.0); else return(-1.0);}

/*  define 3d vector class */
class Point3D{
	float p[3];

	public:
		//construtors
		Point3D(){
			p[0] = p[1] = p[2] = 0;
		}
		Point3D(float x, float y, float z){
			p[0] = x;
			p[1] = y;
			p[2] = z;
		}
		Point3D(float* array){
			p[0] = array[0];
			p[1] = array[1];
			p[2] = array[2];
		}


		/*** access methods ****/
		float x() {return p[0];}
		float y() {return p[1];}
		float z() {return p[2];}
		float* get() {return p;}
		void set_x(float x) { p[0] = x;}
		void set_y(float y) { p[1] = y;}
		void set_z(float z) { p[2] = z;}
		void set(float x, float y, float z){
			p[0] = x;
			p[1] = y;
			p[2] = z;
		}
		void zero() { p[0] = p[1] = p[2] = 0;}

		/*** methods ***/
		void normalise(){
			float magnitude = sqrt(p[0]*p[0]+p[1]*p[1]+p[2]*p[2]);
			if(magnitude == 0) return;
			p[0] /= magnitude;
			p[1] /= magnitude;
			p[2] /= magnitude;
		}

		void addx(float dx){ p[0] += dx;}
		void addy(float dy){ p[1] += dy;}
		void addz(float dz){ p[2] += dz;}
		void sqrt_xyz(){
 			p[0]=sign(p[0])*sqrt(fabs(p[0]));
 			p[1]=sign(p[1])*sqrt(fabs(p[1]));
 			p[2]=sign(p[2])*sqrt(fabs(p[2]));
		}

		float distance_to(Point3D & a){
			float dx = p[0] - a.x();
			float dy = p[1] - a.y();
			float dz = p[2] - a.z();
			return sqrt(dx*dx+dy*dy+dz*dz);
		}

		/**** operator overload *****/
		Point3D & operator=(Point3D rhs){
			p[0] = rhs.x();
			p[1] = rhs.y();
			p[2] = rhs.z();
			return *this;
		}

/*		Point3D & operator+=(Point3D& rhs){
			p[0] += rhs.x();
			p[1] += rhs.y();
			p[2] += rhs.z();
			return *this;
		}
*/
		Point3D & operator/=(float factor){
			p[0]/=factor;
			p[1]/=factor;
			p[2]/=factor;
			return *this;
		}

		friend Point3D operator +(Point3D a, Point3D b){
			return Point3D(a.x()+b.x(),
				a.y()+b.y(), a.z()+b.z());
		}
		friend Point3D operator -(Point3D a, Point3D b){
			return Point3D(a.x()-b.x(),
				a.y()-b.y(), a.z()-b.z());
		}

		friend Point3D& operator +=(Point3D& a, Point3D b){
			a.p[0] += b.x();
			a.p[1] += b.y();
			a.p[2] += b.z();
			return a;
		}
		friend Point3D& operator -=(Point3D& a, Point3D b){
			a.p[0] -= b.x();
			a.p[1] -= b.y();
			a.p[2] -= b.z();
			return a;
		}

		friend Point3D operator *(Point3D a, float k){
			return Point3D(a.x()*k,
				a.y()*k, a.z()*k);
		}
		friend Point3D operator *(float k, Point3D a){
			return Point3D(a.x()*k,
				a.y()*k, a.z()*k);
		}

		//dot product
		friend float operator *(Point3D & a, Point3D & b){
			return (a.x()*b.x()+
				a.y()*b.y()+ a.z()*b.z());
		}

		//For debugging
		friend std::ostream & operator<<(std::ostream& out, Point3D a){
			out << "("<<a.p[0]<<", "<<a.p[1]<<", "<<a.p[2]<<")\n";
			return out;
		}
		

};

void updateBoid(int i);
void drawBoid(int i);


//compute the vector from Boid_Location[i] to its preceived mass center
Point3D& vector_to_massCenter(int k, Point3D& vec3, float r1);
