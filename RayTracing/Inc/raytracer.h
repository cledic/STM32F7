/*------------------------------------------------------------------------
  This is based on Paul Heckbert's "business card raytracer".
  His site is: http://www.cs.cmu.edu/~ph/

  I modified it a lot adding colors/materials, arbitrary sphere
  positions, etc.
  
  I also added some comments and made it readable. The original
  code was designed to be small enough to print on the back of
  a business card (hence the name) so it was very hard to read.
  
  FTB.
------------------------------------------------------------------------*/

/*------------------------------------------------------------------------
  (For non-arduino platforms)
------------------------------------------------------------------------*/
#ifndef PROGMEM
#define PROGMEM
float pgm_read_float(const float *f) {  return *f;  }
#endif

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

/*
  File include per i programmi 3D
*/
#include "stm32f7xx_hal.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_sd.h"
#include "stm32f7xx_hal_i2s.h"
#include "stm32f7xx_hal_lptim.h"
#include "stm32f7xx_hal_tim.h"
#include "stm32746g_discovery.h"
#include "stm32746g_discovery_audio.h"
#include "stm32746g_discovery_lcd.h"
#include "stm32746g_discovery_ts.h"

#define RGBTO565(_r, _g, _b) ((((_r) & B11111000)<<8) | (((_g) & B11111100)<<3) | ((_b) >>3))
#define RGBTO888(_r, _g, _b) (0xFF000000 | ((_r<<16) | (_g <<8) | (_b)))

/* http://www.gamedev.net/topic/75474-fast-floor-and-ceil/ */
#define auxCeil(x) ((float)(uint32_t)((x)+1))
	
float my_sqrt_asm( float f);

/*------------------------------------------------------------------------
  Values you can play with...
------------------------------------------------------------------------*/

// Position of the camera (nb. 'Z' is up/down)
static float cameraX = 0.0;
static float cameraY = 0.0;
static float cameraZ = 3.0;

// What the camera is pointing at
static float targetX = 1.0;
static float targetY = 8.0;
static float targetZ = 4.0;

// We cast this many rays per pixel for stochastic antialiasing and soft-shadows. 
//
// Large numbers produce a nicer image but it runs a lot slower
//static const int raysPerPixel = 4;

// The camera's field of view, smaller=>zoom, larger=>wide angle
static float fov = 0.45f;

// The size of the soft shadow, larger=>wider area
static const float shadowRegion = 0.125f;

/*------------------------------------------------------------------------
  Materials
------------------------------------------------------------------------*/
static const float ambient = 0.05f;
static const float materials[] PROGMEM = {
// R,    G,    B,   REFLECTIVITY
  0.8f, 0.8f, 0.8f,   0.5f,    // Mirror
//  1.0f, 0.0f, 0.0f,   0.3f,    // Red
  0.0f, 1.0f, 0.0f,   0.2f,    // Green
//  0.0f, 0.0f, 0.1f,   0.3f     // Dark blue
  0.0f, 0.8f, 0.8f,   0.3f     // Cyan
};

/*------------------------------------------------------------------------
  The spheres in the world
------------------------------------------------------------------------*/
#define NUM_SPHERES 4
static const float spheres[] PROGMEM = {
// center  radius material
   5,15,8,   5,     0,
  -6,12,4,   3,     0,
   1,10,2,   2,     1,
   0, 9,5,   1,     2
};

/*------------------------------------------------------------------------
  A 3D vector class
------------------------------------------------------------------------*/
struct vec3 {
//public:
  float x,y,z;  // Vector has three float attributes.
  vec3(){}
  vec3(float a, float b, float c){x=a;y=b;z=c;}
  vec3 operator+(const vec3& v) const { return vec3(x+v.x,y+v.y,z+v.z);  }    // Vector add
  vec3 operator-(const vec3& v) const { return (*this)+(v*-1);           }    // Vector subtract
  vec3 operator*(float s)       const { return vec3(x*s,y*s,z*s);        }    // Vector scale
  float operator%(const vec3& v)const { return x*v.x+y*v.y+z*v.z;        }    // Scalar product
  vec3 operator^(const vec3& v) const { return vec3(y*v.z-z*v.y, z*v.x-x*v.z, x*v.y-y*v.x);  } // Vector product
  vec3 operator!()              const { return *this*(1.0f/(float)(my_sqrt_asm(*this%*this)));  }  // Normalized vector
  void operator+=(const vec3& v)      { x+=v.x;  y+=v.y;  z+=v.z;        }
  void operator*=(float s)            { x*=s;    y*=s;    z*=s;          }
};

// A ray...
struct ray {
	public:
  // This occupies 24 bytes - you could only fit 20 of these into
  // a Tiny85 even if you could use the entire RAM (which you can't...)
  vec3 o;  // Origin
  vec3 d;  // Direction
};

/*------------------------------------------------------------------------
  Intersect a ray with the world
  Return 'SKY' if no hit was found but ray goes upward
  Return 'FLOOR' if no hit was found but ray goes downward towards the floor
  Return a material index if a hit was found

  Distance to the hit is returned in 'distance'. 
  The surface normal at the hit is returned in 'normal'
------------------------------------------------------------------------*/
// Values for 'SKY' and 'FLOOR'
static const uint8_t SKY=255;
static const uint8_t FLOOR=254;

uint8_t trace(const ray& r, float& distance, vec3& normal)
{
  // Assume we didn't hit anything
  uint8_t result = SKY;
	vec3 oc;

  // Does the ray go downwards?
  float d = -r.o.z/r.d.z;
  if (d > 0.01f) {
    // Yes, assume it hits the floor
    result = true;
    distance = d;
    result = FLOOR;
    normal = vec3(0,0,1);
  }

  // Test the objects in the scene to see if there's anything in the way
	#pragma unroll 
  for (uint8_t i=0; i<NUM_SPHERES; ++i) {
    //vec3 oc = r.o;
		oc = r.o;
    // Read a sphere from progmem, calculate vector 'oc'
    const float *n = spheres+(i*5);
    oc.x -= *(n++);
    oc.y -= *(n++);
    oc.z -= *(n++);
    d = *(n++);  // Radius

    // Ray-sphere intersection test
    // Math is here: http://en.wikipedia.org/wiki/Line%E2%80%93sphere_intersection
    const float b = r.d%oc;        // I.(o-c)
    const float c = (oc%oc)-(d*d); // (o-c).(o-c) - r^2

    // Does the ray hit the sphere?
    d = (b*b)-c;
    if (d > 0) {
      // Yes, compute the distance to the hit
      d = (-b)-my_sqrt_asm(d);

      // Is it the closest hit so far?
      if ((d > 0.01f) and ((result==SKY) or (d<distance))) {
        // Yes, save results
        distance = d;
        normal = !(oc+r.d*d);
        result = uint8_t(*(n));  // The sphere's material
      }
    }
  }
  return result;
}
float raise(float p, uint8_t n)
{
  while (n--) {
    p = p*p;
  }
  return p;
}

float raise2(float p, uint8_t n)
{
	p = p*p;
	p = p*p;	

  return p;
}

float raise5(float p, uint8_t n)
{
	p = p*p;
	p = p*p;
	p = p*p;
	p = p*p;
	p = p*p;
	
  return p;
}

/*----------------------------------------------------------
  Small, fast pseudo-random number generator
  
  I found this in a forum and I'm not sure who originally
  wrote it. It works very well though....
 
  If you wrote this then get in touch and I'll put
  your name here. :-)                              FTB.
----------------------------------------------------------*/
uint8_t rngA, rngB, rngC, rngX;
uint8_t randomByte()
{
  ++rngX;                        // X is incremented every round and is not affected by any other variable
  rngA = (rngA^rngC^rngX);       // note the mix of addition and XOR
  rngB = (rngB+rngA);            // And the use of very few instructions
  rngC = (rngC+(rngB>>1)^rngA);  // the right shift is to ensure that high-order bits from B can affect  
  return rngC;
}

// A random float in the range [-0.5 ... 0.5]  (more or less)
float randomFloat()
{
  uint8_t r = randomByte();
  return (float)(r)/256.0f;
}
#define RF randomFloat()
#define SH (RF*shadowRegion)

/*------------------------------------------------------------------------
  Sample the world and return the pixel color for a ray
------------------------------------------------------------------------*/
float sample(ray& r, vec3& color)
{
  // See if the ray hits anything in the world
  float t;  vec3& n = color;      // RAM is tight, use 'color' as temp workspace
  const uint8_t hit = trace(r,t,n);

  // Did we hit anything
  if (hit == SKY) {
    // Generate a sky color if the ray goes upwards without hitting anything
    color = vec3(0.1f,0.0f,0.3f) + vec3(.7f,.2f,0.5f)*raise2(1.0f-r.d.z, 2);
    return 0.0f;
  }

  // New ray origin
  r.o += r.d*t;

  // Half vector
  const vec3 half = !(r.d+n*((n%r.d)*-2));

// Vector that points towards the light
  r.d = vec3(9+SH, 6+SH,16); // Where the light is
  r.d = !(r.d-r.o);          // Normalized light vector

  // Lambertian factor
  float d = r.d%n;    // Light vector % surface normal

  // See if we'rez  in shadow
  if ((d<0) or (trace(r,t,n)!=SKY)) {
    d = 0;
  }

  // Did we hit the floor?
  if (hit == FLOOR) {
    // Yes, generate a floor color
    d=(d*0.2f)+0.1f;   t=d*3.0f;  // d=dark, t=light
    color = vec3(t,t,t);       // Assume grey color
    t = 1.0f/5.0f;     // Floor tiles are 5m across
//    int fx = int(ceil(r.o.x*t));
//    int fy = int(ceil(r.o.y*t));
//    bool dark = ((fx+fy)&1)!=0;  // Light or dark color?
    bool dark = (((int)(auxCeil(r.o.x*t)+auxCeil(r.o.y*t)))&1);  // Light or dark color? -> fix for AVR compiler
    if (dark) { color.y = color.z = d; }        // g+b => dark => 'red'
    return 0;
  }

  // No, we hit the scene, read material color from progmem
  const float *mat = materials+(hit*4);
  color.x = *(mat++);
  color.y = *(mat++);
  color.z = *(mat++);
 
  // Specular light in 't'
  t = d;
  if (t > 0) {
    t = raise5(r.d%half, 5);
  }

  // Calculate total color using diffuse and specular components
  color *= d*d+ambient;  // Ambient+diffuse
  color += vec3(t,t,t);  // Specular

  // We need to trace a reflection ray...need to modify 'r' for the recursion
  r.d = half;
  return *(mat);    // Reflectivity of this material
}

/*------------------------------------------------------------------------
  Raytrace the entire image
------------------------------------------------------------------------*/
void doRaytrace(int raysPerPixel = 4, int dw = 320, int dh = 240, int q = 1)
{
  // Trace it
  int dw2=dw/2;
  int dh2=dh/2;
  const float pixel =  fov/float(dh2);    // Size of one pixel on screen

  // Position/target of camera
  const vec3 camera = vec3(cameraX,cameraY,cameraZ);
  const vec3 target = vec3(targetX,targetY,targetZ);

  for (int y=0; y<dh; y+=q) {
    for (int x=0; x<dw; x+=q) {
      vec3 acc(0,0,0);     // Color accumulator
      for (int p=raysPerPixel; p--;) {
        ray r;  vec3 temp;
        float xpos = float(x-dw2), ypos=float(dh2-y);
        //if (raysPerPixel>1) { xpos+=RF; ypos+=RF; }       // Stochastic antialiasing when RPP > 1

        // Calculate a ray through this pixel
        temp = !(target-camera);
        vec3& right = r.o;   right = !(temp^vec3(0,0,1));
        vec3& up = r.d;      up = !(right^temp);
        r.d = !(temp + ((right*xpos)+(up*ypos))*pixel);  // Ray direction
        r.o = camera;                                    // Ray starts at the camera
        
        // Sample the world, accumulate the color returned
        vec3& color = temp;
        float reflect1 = sample(r,color);
        acc += color;
        // 'sample()' would normally be recursive but there's not enough RAM to do that on a Tiny85...
        if (reflect1 > 0) {
          // ...so we do the 'recursion' manually
          float reflect2 = sample(r,color);
          acc += color*reflect1;
          if (reflect2 > 0) {
            // ...3 levels deep
            sample(r,color);
            acc += color*(reflect1*reflect2);
          }
        }
      }
      
      // Output the pixel
      acc = acc*(255.0f/float(raysPerPixel));
      int r = acc.x;    if (r>255) { r=255; }
      int g = acc.y;    if (g>255) { g=255; }
      int b = acc.z;    if (b>255) { b=255; }
			
 			//__asm { usat r,#8,r}
			//__asm { usat g,#8,g}
			//__asm { usat b,#8,b}
			
			BSP_LCD_DrawPixel(x, y, RGBTO888(r,g,b));			
			
#if 0			
			if ( q==1) {
				BSP_LCD_DrawPixel(x, y, RGBTO888(r,g,b));
			} else {
				BSP_LCD_SetTextColor( RGBTO888(r,g,b));
				BSP_LCD_FillRect(x, y, q, q);
			}
#endif
						
    }
  }
}

void setCameraPos( float x=0.0f, float y=0.0f, float z=3.0f)
{
	cameraX = x;
	cameraY = y;
	cameraZ = z;
}

void getCameraPos( float*x, float*y, float*z)
{
	*x=cameraX;
	*y=cameraY;
	*z=cameraZ;
}

void setCameraTarget( float x=1.0f, float y=8.0f, float z=4.0f)
{
	targetX = x;
	targetY = y;
	targetZ = z;
}

void getCameraTarget( float*x, float*y, float*z)
{
	*x = targetX;
	*y = targetY;
	*z = targetZ;
}

void setFieldOfView( float f=0.45f)
{
	fov = f;
}

void getFieldOfView( float*f)
{
	*f = fov;
}

