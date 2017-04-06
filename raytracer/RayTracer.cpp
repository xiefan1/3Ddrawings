/*
  CSC418 - RayTracer code - Winter 2017 - Assignment 3&4

  Written Dec. 9 2010 - Jan 20, 2011 by F. J. Estrada
  Freely distributable for adacemic purposes only.

  Uses Tom F. El-Maraghi's code for computing inverse
  matrices. You will need to compile together with
  svdDynamic.c

  You need to understand the code provided in
  this file, the corresponding header file, and the
  utils.c and utils.h files. Do not worry about
  svdDynamic.c, we need it only to compute
  inverse matrices.

  You only need to modify or add code in sections
  clearly marked "TO DO"
*/

#include "utils.h"
#include "assert.h"
#define maxlight 10
//#define DEBUGTEXT
#define DEBUGRGB

// A couple of global structures and data: An object list, a light list, and the
// maximum recursion depth
struct object3D *object_list;
struct object3D *light_list;
double light_radius[maxlight];
struct object3D *backgroundObj;
int MAX_DEPTH;
int antialiasing;	// Flag to determine whether antialiaing is enabled or disabled
FILE *debugUV;

//generate weights from Gaussian normal function
//size is always odd
void gen_Gaussian_weight(double *table,int center){
    if(!table) return;
    double coe = 1/(2*PI);
    int size = center*2+1;
    for(int i=0;i<size;++i){
	for(int j=0;j<size;++j){
	    double x = j - center;
	    double y = i - center;
	    double p = -(x*x+y*y)/2;
	    *(table+i*size+j) = coe*pow(E,p);
	}
    }
}


//accumulate the top transformation ONE level down to its children
void setChildT(struct object3D* top){
 struct object3D* cur = top->children;
 while(cur!=NULL){
    matMult(top->T,cur->T);
    invert(&cur->T[0][0],&cur->Tinv[0][0]);
 }
}


//The top level object is a bounding box that holds all parts, which is returned.
//User can apply transformations directly onto this box object, which will be
//accumulated to its child nodes by calling setChildT(obj);.
struct object3D* buildAvator(void){
 
 struct object3D *top, *o;

 top=newBox(1,1,1,1,1,1,1,1,1,10); //top level bounding box
 invert(&top->T[0][0],&top->Tinv[0][0]);
 insertObject(top,&object_list);

 //an opague cone (crown)
 o=newCone(.2,.8,.1,.8,.8,.8,1,1,1.52,10);
 RotateZ(o,PI/8);
 Translate(o,0,5,-2);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 //insert this object into the boudning box object list
 insertObject(o,&(top->children));

 //an opague refractive sphere (head)
 o=newSphere(.2,.95,.95,1,.94,.5,.5,1,1.52,10);
 Translate(o,0,4,-2);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&(top->children));


 //an opague paraboloid (body)
 o=newParaboloid(.2,.5,.1,.3,.8,.8,1,1,1.52,10);
 Translate(o,0,1.5,-2);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&(top->children));

 //legs

 return o;
}



void buildScene(void)
{
 // Sets up all objects in the scene. This involves creating each object,
 // defining the transformations needed to shape and position it as
 // desired, specifying the reflectance properties (albedos and colours)
 // and setting up textures where needed.
 // Light sources must be defined, positioned, and their colour defined.
 // All objects must be inserted in the object_list. All light sources
 // must be inserted in the light_list.
 //
 // To create hierarchical objects:
 //   Copy the transform matrix from the parent node to the child, and
 //   apply any required transformations afterwards.
 //
 // NOTE: After setting up the transformations for each object, don't
 //       forget to set up the inverse transform matrix!


 //set up the backgroud as a huge sphere
 backgroundObj = newSphere(0,0,0,0,0,0,0,0,0,0);
 Scale(backgroundObj,20,10,20);
 invert(&backgroundObj->T[0][0],&backgroundObj->Tinv[0][0]);
 //loadTexture(backgroundObj,"texture/starSphere.ppm");


 struct object3D *o;

 // Note the parameters: ra, rd, rs, rg, R, G, B, alpha, r_index, and shinyness)

 o=newPlane(.1,.75,.05,.35,.55,.8,.75,1,1.33,2);	// Note the plane is highly-reflective (rs=rg=.75) so we
						// should see some reflections if all is done properly.
						// Colour is close to cyan, and currently the plane is
						// completely opaque (alpha=1). The refraction index is
						// meaningless since alpha=1
// Scale(o,10,10,1);				// Do a few transforms...
 Scale(o,40,60,1);				// Do a few transforms...
 RotateZ(o,PI/1.20);
 RotateX(o,PI/2.25);
 Translate(o,0,-3,10);
// loadTexture(o,"texture/medium_check.ppm");
 loadTexture(o,"texture/lake1.ppm");
 invert(&o->T[0][0],&o->Tinv[0][0]);		// Very important! compute
						// and store the inverse
						// transform for this object!
 insertObject(o,&object_list);			// Insert into object list

 //an opague ellipse
 o=newSphere(.05,.95,.35,.35,.5,1,.83,1,1,10);
 Scale(o,.75,.5,1.5);
 RotateY(o,PI/3);
 Translate(o,-3,0.1,5);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);

 //a tranparent ellipse
 o=newSphere(.3,.5,.95,1,1,1,.2,1,1.52,10);
 Scale(o,.5,2.0,1.0);
 RotateZ(o,PI/1.8);
 Translate(o,-3.5,-2,1.5);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);

 //a mirror
 o=newPlane(.05,.75,.05,1,1,1,1,1,1,2);
 o->isMirror = 1; 			//for mirror, specify all rgb to 1,1,1
 Scale(o,3.3,2.5,1);
 RotateY(o,PI/3.5);
 RotateZ(o,-PI/19);
 Translate(o,3,-1,3);
 invert(&o->T[0][0],&o->Tinv[0][0]);	
 insertObject(o,&object_list);		

/* //an transparent plane
 o=newPlane(.05,.1,.7,1,1,1,1,.5,1.5,2);
 Translate(o,-1,1.5,-1);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);
*/

 //an transparent sphere
 o=newSphere(.1,.1,.6,.8,1,1,1,.2,1.42,10);
 Scale(o,1.3,1.3,1.3);
 Translate(o,0,3,1);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);



 o=newBox(.3,.95,.95,.5,.94,.5,.5,1,1.52,10);
 Scale(o,.5,.5,.5);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);


/*
 //an opague refractive sphere
 o=newSphere(.3,.95,.95,.5,.94,.5,.5,1,1.52,10);
 Translate(o,-1.9,0,1);
 invert(&o->T[0][0],&o->Tinv[0][0]);
 insertObject(o,&object_list);

 //an opaque plane
 o=newPlane(.05,.75,.05,1,.8,.5,.3,1,1,10);
 Scale(o,3.3,2.5,1);
 RotateY(o,-PI/3.5);
 Translate(o,0,0,0);
 invert(&o->T[0][0],&o->Tinv[0][0]);	
 insertObject(o,&object_list);		
*/

 // Insert a single point light source as sphere
 double r1=3;
 light_radius[0]=r1;
 o=newSphere(0,0,0,0,.95,.95,.95,1,0,0);
 o->isLightSource=1;
 Scale(o,r1,r1,r1);
 Translate(o,0,14.5,-9.5);
 insertObject(o,&light_list);

 // End of simple scene for Assignment 3
 // Keep in mind that you can define new types of objects such as cylinders and parametric surfaces,
 // or, you can create code to handle arbitrary triangles and then define objects as surface meshes.
 //
 // Remember: A lot of the quality of your scene will depend on how much care you have put into defining
 //           the relflectance properties of your objects, and the number and type of light sources
 //           in the scene.
}





int main(int argc, char *argv[])
{
 // Main function for the raytracer. Parses input parameters,
 // sets up the initial blank image, and calls the functions
 // that set up the scene and do the raytracing.
 struct image *im;	// Will hold the raytraced image
 struct view *cam;	// Camera and view for this scene
 int sx;		// Size of the raytraced image
 char output_name[1024];	// Name of the output file for the raytraced .ppm image
 struct point3D e;		// Camera view parameters 'e', 'g', and 'up'
 struct point3D g;
 struct point3D up;
 double du, dv;			// Increase along u and v directions for pixel coordinates
 struct colourRGB background;   // Background colour
 unsigned char *rgbIm;
 srand(1522);

 if (argc<5)
 {
  fprintf(stderr,"RayTracer: Can not parse input parameters\n");
  fprintf(stderr,"USAGE: RayTracer size rec_depth softshadow output_name\n");
  fprintf(stderr,"   size = Image size (both along x and y)\n");
  fprintf(stderr,"   rec_depth = Recursion depth\n");
  fprintf(stderr,"   softshadow = A single digit, 0 disables softshadow. Anything else enables softshadow\n");
  fprintf(stderr,"   output_name = Name of the output file, e.g. MyRender.ppm\n");
  exit(0);
 }
 sx=atoi(argv[1]);
 MAX_DEPTH=atoi(argv[2]);
 if (atoi(argv[3])==0) antialiasing=0; else antialiasing=1;
 strcpy(&output_name[0],argv[4]);

 fprintf(stderr,"Rendering image at %d x %d\n",sx,sx);
 fprintf(stderr,"Recursion depth = %d\n",MAX_DEPTH);
 if (!antialiasing) fprintf(stderr,"Softshadow is off\n");
 else fprintf(stderr,"Softshadow is on\n");
 fprintf(stderr,"Anti-aliasing is always on\n");
 fprintf(stderr,"Output file name: %s\n",output_name);

 object_list=NULL;
 light_list=NULL;

 // Allocate memory for the new image
 im=newImage(sx, sx);
 if (!im)
 {
  fprintf(stderr,"Unable to allocate memory for raytraced image\n");
  exit(0);
 }
 else rgbIm=(unsigned char *)im->rgbdata; //Fan: char *rgb[im->sx][im->sy][3]

 buildScene();		// Create a scene. This defines all the
			// objects in the world of the raytracer

 // Mind the homogeneous coordinate w of all vectors below. DO NOT
 // forget to set it to 1, or you'll get junk out of the
 // geometric transformations later on.

 // Camera center is at (0,0,-1)
 e.px=0;
 e.py=5;
 e.pz=-10;
 e.pw=1;

 // To define the gaze vector, we choose a point 'pc' in the scene that
 // the camera is looking at, and do the vector subtraction pc-e.
 // Here we set up the camera to be looking at the origin, so g=(0,0,0)-(0,0,-1)
 g.px=0;
 g.py=-5;
 g.pz=10;
 g.pw=0;
 normalize(&g);

 // Define the 'up' vector to be the Y axis
 up.px=0;
 up.py=1;
 up.pz=0;
 up.pw=0;

 // Set up view with given the above vectors, a 4x4 window. Fan: size 4 is in distance units, not pixels
 // and a focal length of -1 (why? where is the image plane?)
 // Note that the top-left corner of the window is at (-2, 2)
 // in camera coordinates.
 cam=setupView(&e, &g, &up, -2, -2, 2, 4);

 if (cam==NULL)
 {
  fprintf(stderr,"Unable to set up the view and camera parameters. Our of memory!\n");
  cleanup(object_list);
  cleanup(light_list);
  deleteImage(im);
  exit(0);
 }

 // Set up background colour here
 background.R=0;
 background.G=0;
 background.B=0;
 du=cam->wsize/(sx-1);		// dv is negative since y increases downward in pixel
 dv=-cam->wsize/(sx-1);		// coordinates and upward in camera coordinates.
				//Fan: cam->wsize is in distance unit, sx is the resolution
				
 fprintf(stderr,"View parameters:\n");
 fprintf(stderr,"Left=%f, Top=%f, Width=%f, f=%f\n",cam->wl,cam->wt,cam->wsize,cam->f);
 fprintf(stderr,"Camera to world conversion matrix (make sure it makes sense!):\n");
 printmatrix(cam->C2W);
 fprintf(stderr,"World to camera conversion matrix\n");
 printmatrix(cam->W2C);
 fprintf(stderr,"\n");

 fprintf(stderr,"Rendering rows ");
 
 int center = 1;
 int ns=2*center+1; //[ns x ns] subcells per pixel
 double num = ns*ns;
 double dsu = du/(ns-1);
 double dsv = dv/(ns-1); //note dsy is negative
 double weightG[ns][ns];
 //compute weight from Gaussian function (low-pass filter)
 gen_Gaussian_weight(&weightG[0][0],center);

 //initialize points and vectors in the camera space
 struct point3D origin;
 origin.px=0;
 origin.py=0;
 origin.pz=0;
 origin.pw=1;
 #ifdef DEBUGRGB
FILE *debugRGB=fopen("rgb.txt","wb+");
#endif
 #ifdef DEBUGTEXT
debugUV=fopen("uv.txt","wb+");
#endif


 //openmp multi-threaded
 #pragma omp parallel for
 for (int j=0;j<sx;j++)		// For each of the pixels in the image
 {
  //direction vector: pixel coordinate-origin
  struct point3D ps;
  ps.py=cam->wt+j*dv; //note: dv is negative
  ps.pz=cam->f;
  ps.pw=0;

   for (int i=0;i<sx;i++)
  {
    //update to the current pixel position
    ps.px=cam->wl+i*du;

    struct colourRGB col_avg={0,0,0};
    struct point3D copyP;
    copyPoint(&ps,&copyP);
    //anti-aliasing by supersampling
    //divide per pixel into nsxns cells and randomly shoot rays
    for(int su=0;su<ns;++su){
	for(int sv=0;sv<ns;++sv){
	    struct colourRGB col={0,0,0};

	    //for each subcell
	    //construct the primary ray
	    struct ray3D *ray = newRay(&origin,&copyP);

	    //transform the ray into the world space
	    matRayMult(cam->C2W,ray);
	    rayTrace(ray,0,&col,NULL);

	    //average the col with Gaussian weight
	    mult_col(weightG[su][sv],&col);
	    add_col(&col,&col_avg);

	    //update to the next subcell position
	    copyP.px+=dsu;
	    free(ray);
	    ray = NULL;
	}
	copyP.px=ps.px;
	copyP.py+=dsv;
    }

    //set color of this pixel
    //printf("(%f, %f, %f)",col.R,col.G,col.B);
    *(rgbIm+j*sx*3+i*3+0) = col_avg.R*255;
    *(rgbIm+j*sx*3+i*3+1) = col_avg.G*255;
    *(rgbIm+j*sx*3+i*3+2) = col_avg.B*255;

 #ifdef DEBUGRGB
    fprintf(debugRGB,"(%d %d %d) ",(int)(col_avg.R*255),(int)(col_avg.G*255),(int)(col_avg.B*255));
/*    if(j>200 && col_avg.R*255<10)
	printf("wrong!\n");
*/	
 #endif

  } // end of this row
 #ifdef DEBUGRGB
  fprintf(debugRGB,"\n\n");
 #endif
 } // end for j

 #ifdef DEBUGRGB
 fclose(debugRGB);
 #endif
 #ifdef DEBUGTEXT
 fclose(debugUV);
 #endif

 fprintf(stderr,"\nDone!\n");

 // Output rendered image
 imageOutput(im,output_name);

 // Exit section. Clean up and return.
 cleanup(object_list);		// Object and light lists
 cleanup(light_list);
 deleteImage(im);				// Rendered image
 free(cam);					// camera view
 exit(0);
}


// Find the closest intersection between the ray and any objects in the scene.
// It returns:
//   - The lambda at the intersection (or < 0 if no intersection)
//   - The pointer to the object at the intersection (so we can evaluate the colour in the shading function)
//   - The location of the intersection point (in p)
//   - The normal at the intersection point (in n)
//
// Os is the 'source' object for the ray we are processing, can be NULL, and is used to ensure we don't 
// return a self-intersection due to numerical errors for recursive raytrace calls.
// note: ray is in the world coords
void findFirstHit(struct ray3D *ray, double *lambda, struct object3D *Os,
		  	struct object3D **obj, struct point3D *p, 
			struct point3D *n, double *a, double *b, int depth){
    *lambda = -1;
    *obj = NULL;
    int initial=1;
    double temp=0; //temporary lambda
    struct point3D _n,_p;
    double _a,_b;

    struct object3D *cur_obj=object_list;
    while(cur_obj!=NULL){

	cur_obj->intersect(cur_obj,ray,&temp,&_p,&_n,&_a,&_b);

	//Q1:should it compare with 1 instead??
	if(temp>0){
    	    if(initial==1 || *lambda>temp)
    	    {
		initial = 0;
		*lambda = temp;
		*obj = cur_obj;

		/* Transform n and p back to the world coords */
		// transform normal vectors n
		double Tinv_trans[4][4]={0};
		transpose(&(cur_obj->Tinv[0][0]),&(Tinv_trans[0][0]));
		matVecMult(Tinv_trans,&_n);
		normalize(&_n);

		// transform the intersect point p
		matVecMult(cur_obj->T,&_p);

		memcpy(n,&_n,sizeof(struct point3D));
		memcpy(p,&_p,sizeof(struct point3D));

		*a=_a;
		*b=_b;
	    }
	}

	cur_obj=cur_obj->next;
    }
}


// generate unit refraction ray
// alpha will be recalculated by this function, as transmittance T
// n - normal unit vector
// b - intersection to eye unit vector
// p - intersection point
struct ray3D* gen_refractionRay(struct object3D* obj, struct point3D* n, struct point3D* b, struct point3D* p){
    struct point3D d;
    d.px=-(b->px);
    d.py=-(b->py);
    d.pz=-(b->pz);
    d.pw=1;

    double ni, nt;
    double cosCritical=-1;//critical angle for total internal reflection
    if(!obj->goingOut){
	nt = obj->r_index;
	ni = 1.0;
    }else{
	ni = obj->r_index;
	nt = 1.0;
    	//critical angle cosine
	if(ni>nt){
	    double temp = nt/ni;
    	    cosCritical = sqrt(1-temp*temp);
	}
    }

    double cosTheta = dot(n,b);
    //if theta > critical angle, no reflection
    if(cosTheta < cosCritical)
	return 0;

    struct point3D temp;
    double cosPhi = (1-cosTheta*cosTheta)*(ni*ni)/(nt*nt);
    assert(cosPhi<=1);
    cosPhi = sqrt(1-cosPhi);
    temp.px = ((d.px-n->px*cosTheta)*ni/nt) - n->px*cosPhi;
    temp.py = ((d.py-n->py*cosTheta)*ni/nt) - n->py*cosPhi;
    temp.pz = ((d.pz-n->pz*cosTheta)*ni/nt) - n->pz*cosPhi;
    temp.pw = 0;
    normalize(&temp);
    
    //recalculate alpha, i.e. tranmittance
    //t is fresnel (transmission) coefficient, TE mode
    double t = 2*ni*cosTheta/(ni*cosTheta+nt*cosPhi);
    obj->alpha = nt*cosPhi*t*t/(ni*cosTheta);
    assert(obj->alpha<1);
    assert(obj->alpha>0);

    return(newRay(p,&temp));
}


// generate unit reflection ray
// n - normal unit vector
// b - intersection to eye unit vector
// p - intersection point
struct ray3D* gen_reflectionRay(struct point3D* n, struct point3D* b, struct point3D* p){
    struct point3D r;
    copyPoint(n,&r);
    double up=2*dot(n,b);
    multVector(up,&r);
    subVectors(b,&r);
    normalize(&r);
    r.pw=0;

    return(newRay(p,&r)); //r is normalized
}


// Ray-Tracing function. It finds the closest intersection between
// the ray and any scene objects, calls the shading function to
// determine the colour at this intersection, and returns the
// colour.
//
// Os is needed for recursive calls to ensure that findFirstHit will
// not simply return a self-intersection due to numerical
// errors. For the top level call, Os should be NULL. And thereafter
// it will correspond to the object from which the recursive
// ray originates.
void rayTrace(struct ray3D *ray, int depth, struct colourRGB *col,
			struct object3D *Os)
{
	assert(ray);
	if (depth>MAX_DEPTH)	// Max recursion depth reached
	    return;

	double lambda=0, a=0,b=0; //a,b are texture coords
    	struct object3D* hitObj=NULL;
        struct point3D p,n;
        //find the first intersection
        //return lambda, hit object(next object source), hit point and normal
        findFirstHit(ray,&lambda,Os,&hitObj,&p,&n,&a,&b,depth);

	//color shading
        if(hitObj){
            //if hit an object
	    //Phong illumination
	    rtShade(hitObj,&p,&n,ray,depth,a,b,col);

	    #ifdef DEBUGTEXT
	    if(hitObj->texImg){
		fprintf(debugUV,"%.3f %.3f    ",a,b);
	    }
	    #endif
	}else{
	    //environment mapping
	    //get color from the background
	    if(backgroundObj->texImg!=NULL)
    		bgMap(ray,col);
	}
}

void bgMap(struct ray3D* ray, struct colourRGB* col){
	double t=0;
	struct point3D _n,_p;
	double u,v;
	backgroundObj->intersect(backgroundObj,ray,&t,&_p,&_n,&u,&v);
	assert(u>=0 && u<1 && v>=0 && v<1);
	//fill in col with the texture RGB colour
	backgroundObj->textureMap(backgroundObj->texImg,u,v,&col->R,&col->G,&col->B);
}


// This function implements the shading model as described in lecture. It takes
// - A pointer to the first object intersected by the ray (to get the colour properties)
// - The coordinates of the intersection point (in world coordinates)
// - The normal at the point
// - The ray (needed to determine the reflection direction to use for the global component, as well as for
//   the Phong specular component)
// - The current racursion depth
// - The (a,b) texture coordinates (meaningless unless texture is enabled)
//
// Returns:
// - The colour for this ray (using the col pointer)
void rtShade(struct object3D *obj, struct point3D *p, struct point3D *n, struct ray3D *ray,
				int depth, double _a, double _b, struct colourRGB *col)
{
//ray shoot on the back face 
int backface = 0;
if(dot(n,&ray->d)>=0)
	if(obj->frontAndBack)
	    backface=1;
	else return; 


 if(!obj->frontAndBack && dot(n,&ray->d)>=0) return; //ray shoot on the back face ( of plane object)
 if(col->R==1 && col->G==1 && col->B==1) return;

 double R,G,B;			// Colour for the object in R G and B

 if (obj->texImg==NULL)		// Not textured, use object colour
 {
  R=obj->col.R;
  G=obj->col.G;
  B=obj->col.B;
 }
 else
 {
  // Get object colour from the texture given the texture coordinates (a,b), and the texturing function
  // for the object. Note that we will use textures also for Photon Mapping.
  obj->textureMap(obj->texImg,_a,_b,&R,&G,&B);
 }

 //compute the unit p->OS(eye) vector
 struct point3D b;
 copyPoint(&(ray->d),&b);
 normalize(&b);
 multVector(-1,&b);
 b.pw=0;

 //note: alpha and ra,rd,rs,rg will be recalculated if this object is refractive
 //alpha will be set to the transmittance T, ra+rd+rs will be set to reflectance R
 // T+R = 1
 double alpha = obj->alpha;
 double ra,rd,rs,rg;
 ra=obj->alb.ra;
 rd=obj->alb.rd;
 rs=obj->alb.rs;
 rg=obj->alb.rg;

 struct ray3D* rRay;

 /*refraction*/
 if(depth<MAX_DEPTH && alpha<1){

	struct colourRGB col_refract={0,0,0};
	struct point3D n_copy;
	copyPoint(n,&n_copy);

	if(backface){
	    multVector(-1,&n_copy);
	}

	//alpha will be recalculated by this function
	rRay = gen_refractionRay(obj,&n_copy,&b,p);

	if(rRay != NULL){
		//reset alpha, ra-rg
		alpha = obj->alpha;
		ra *=(1-alpha);
		rd *=(1-alpha);
		rs *=(1-alpha);
		rg *=(1-alpha);
	
		rayTrace(rRay,depth+1,&col_refract,obj);
	    	free(rRay);
	      	rRay=NULL;
		//note: alpha is set to the transmittance by the above function.
		//i.e. the larger the alpha, the more transparent this object is
		col_refract.R*=alpha*R;
		col_refract.G*=alpha*G;
		col_refract.B*=alpha*B;
		add_col(&col_refract,col);

		if(col->R>=1 && col->G>=1 && col->B>=1){
		     col->R=1;
		     col->G=1;
		     col->B=1;
		     return;
		}
	 }
 }

 //if this object is not a mirror, compute the local illumination (Phong model).
 if(obj->isMirror==0){
    
     //for all the light sources
     struct object3D *cur;
     int num_light=0;
     cur=light_list;

     while(cur!=NULL){
        double lr,lg,lb;
        lr=cur->col.R;
        lg=cur->col.G;
        lb=cur->col.B;
     
        //compute the unit p->light vector
        struct point3D s={0,0,0,1}; //the p->light vector
        matVecMult(cur->T,&s);
        subVectors(p,&s);
        //normalize the p->light vector
        normalize(&s);
        s.pw=0;
 
        //compute the unit reflection vector
        struct point3D r;
        copyPoint(n,&r);
        double up=2*dot(n,&s);
        multVector(up,&r);
        subVectors(&s,&r);
        normalize(&r);
        r.pw=0;
    
    
        /* ambient */
        add_col(ra*lr*R,ra*lg*G,ra*lb*B,col);
    
        /* shadow (diffuse and specular) */

	//if soft-shadoe is enabled,
	//shoot multiple rays towards the light source
	int numRays=1;
	if(antialiasing==1) numRays = 10;

	for(int light_i=0;light_i<numRays;++light_i){
            //create ray from hitObj to a random point on light source
	    double lightItensity = 1;
    	    struct point3D shadowRay={0,0,0,1}; //it's still a point for now
	    double theta = 2*PI*drand48();
	    double phi = 2*PI*drand48();
      	    double rxyz = light_radius[num_light]*drand48();
	    double rxy = rxyz*sin(theta);
	    shadowRay.px = rxy*cos(phi);
	    shadowRay.py = rxy*sin(phi);
            shadowRay.pz = rxyz*cos(theta);
            matVecMult(cur->T,&shadowRay); //transform to object world
            subVectors(p,&shadowRay);
            shadowRay.pw=0; //now it's a vector
        
	    //note shadow ray shall not be normalized
            struct ray3D *ray_to_light = newRay(p,&shadowRay);
            double shadow_t=0, a_temp, b_temp;
            struct object3D* hitObj=NULL;
            struct point3D _p,_n;
            findFirstHit(ray_to_light,&shadow_t,obj,&hitObj,&_p,&_n,&a_temp,&b_temp,depth);
            free(ray_to_light);
            ray_to_light=NULL;
           
        
            //if any opaque object blocks the light,
            //no diffuse and specular components.
	    //if the hitObj is transparent,
	    //a simple model for now, it should be recursively factored by all the alpha
            if(hitObj != NULL && shadow_t>0 && shadow_t < 1){
		if(hitObj->alpha<1)
		    lightItensity *= alpha;
		else
		    lightItensity = 0;
	    }

	    if(lightItensity>0){
		struct colourRGB col_ds={0,0,0};
                /* diffuse */
                double dim = dot(n,&s);
                if(dim<0){
                	if(obj->frontAndBack) dim=-dim;
            	else dim=0;
                }
                add_col(rd*lr*R*dim,rd*lg*G*dim,rd*lb*B*dim,&col_ds);
            
            
                /* specular */
                dim = dot(&b,&r);
                if(dim<0){
                	if(obj->frontAndBack) dim=-dim;
            	else dim=0;
                }
                dim = pow(dim,obj->shinyness);
                add_col(rs*lr*dim,rs*lg*dim,rs*lb*dim,&col_ds);

		mult_col(lightItensity*((double)1/numRays),&col_ds);
		add_col(&col_ds,col);
        
            }
	}//end of shadow
    
        //next light source
        cur=cur->next;
	num_light+=1;
     }    
 } 
 if(col->R>=1 && col->G>=1 && col->B>=1){
      col->R=1;
      col->G=1;
      col->B=1;
      return;
 }


 /* reflection */
 if(depth<MAX_DEPTH && !backface){
    struct colourRGB col_ref={0,0,0};

    //generate the reflection ray
    rRay = gen_reflectionRay(n,&b,p);
    //recursive call of rayTrace
    rayTrace(rRay,depth+1,&col_ref,obj);
    free(rRay);
    rRay=NULL;
    col_ref.R*=rg*R;
    col_ref.G*=rg*G;
    col_ref.B*=rg*B;
    add_col(&col_ref,col);
 }    

 if(col->R>1) col->R=1;
 if(col->G>1) col->G=1;
 if(col->B>1) col->B=1;
}     


