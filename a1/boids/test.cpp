/* Standard C libraries */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <iostream>

#include "speed.h"
using namespace std;

void change_array(int *array){
array[0] = 99;
cout<<"in function array[0] = "<<array[0]<<endl;

}

int main(int argc, char** argv){

	Point3D vec1(4, 3, 5), vec2(10,10,10), vec3;
	cout<<"vec1 "<<vec1<<"vec2 "<<vec2;
	vec3 = vec2*(-102.4);
	cout<<vec3;
	vec3.sqrt_xyz();
	cout<<vec3<<vec1<<vec2;

	int array[2];
	bzero(array,2*sizeof(int));
	
cout<<"in main array[0] = "<<array[0]<<endl;
	change_array(array);	
cout<<"in main array[0] = "<<array[0]<<endl;


/*	float x, y, z,k;
	cout<<"Please enter x1 y1 z1\n";
	cin>>x>>y>>z;
	Point3D p1(x,y,z);

	cout<<"Please enter k\n";
	cin>>k;
	cout<<"p1*k: "<<p1;
	cout<<(k*p1);

	cout<<"p1: "<<p1<<"p2: "<<p2;
	cout<<"p1*p2: ";
	Point3D p3 = (p1*p2);
	cout<<p3;
	cout<<"p1-p2: ";
	p3 = p1-p2;
	cout<<p3;
	//Problem: can't do cout<<(p1-p2)
	//WHY!!!??
*/

	return 0;
}
