#include "util.h"
#include <cmath>


//k -- Boid identifier
//vec3 -- where to store
//r1 is the effective radius 
Point3D& vector_to_massCenter(int k, Point3D& vec3, float r1){
	if(k<0 | k> MAX_BOIDS) return vec3;

	Point3D mass_center;
	
	for(int i=0; i< nBoids; ++i){
		if(i != k){
			float dis = Boid_Location[k].distance_to(Boid_Location[i]);
			if(dis <= r1)
				mass_center += Boid_Location[i];
		}
	}
	if(nBoids != 1)
	mass_center /= (nBoids-1);
	vec3 = mass_center-Boid_Location[k];

	return vec3;
}


