#include "math.h"
#include "geom.h"
#include "main.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/*
*
*   geometry(..
*
*   This routine performs geometric
*   transformations on the pixels in an
*   image array.  It performs basic
*   displacement, stretching, and rotation.
*
*   The basic equations are:
*
*   new x = x.cos(a) + y.sin(a) + x_displace
*           + x.x_stretch +x.y.x_cross
*
*   new y = y.cos(a) - x.sin(a) + y_displace
*           + y.y_stretch +x.y.y_cross
*
*/
void geometry2(GEOMDATA *gd)
{
   float cosa, sina, radian_angle, tmpx, tmpy;
	 float tmp_xcross;
   float  fi, fj, x_div, y_div, x_num, y_num;
   int    i, j, new_i, new_j;


	/******************************
	*
	*   Load the terms array with
	*   the correct parameters.
	*
	*******************************/

      /* the following magic number is from
         180 degrees divided by pi */
   radian_angle = (float)gd->x_angle/57.29577951f;
   //radian_angle = (x_angle*3.1415)/180;
   cosa  = (float)cos(radian_angle);
   sina  = (float)sin(radian_angle);

      /************************************
      *
      *   NOTE: You divide by the
      *   stretching factors. Therefore, if
      *   they are zero, you divide by 1.
      *   You do this with the x_div y_div
      *   variables. You also need a
      *   numerator term to create a zero
      *   product.  You do this with the
      *   x_num and y_num variables.
      *
      *************************************/

   if(gd->x_stretch < 0.00001f){
      x_div = 1.0f;
      x_num = 0.0f;
   }
   else{
      x_div = gd->x_stretch;
      x_num = 1.0f;
   }

   if(gd->y_stretch < 0.00001f){
      y_div = 1.0f;
      y_num = 0.0f;
   }
   else{
      y_div = gd->y_stretch;
      y_num = 1.0f;
   }

      /**************************
      *
      *   Loop over image array
      *
      **************************/

   for(i=0; i<gd->rows; i++){
      // 
      for(j=0; j<gd->cols; j++){

         fi = i;
         fj = j;
				
         tmpx = (float)(j)*cosa         +
                (float)(i)*sina         +
                (float)(gd->x_displace) +
                (float)(x_num*fj/x_div) +
                (float)(gd->x_cross*i*j);

         tmpy = (float)(i)*cosa         -
                (float)(j)*sina         +
                (float)(gd->y_displace) +
                (float)(y_num*fi/y_div) +
								(float)(gd->y_cross*i*j);

         if(gd->x_stretch != 0.0f)
            tmpx = tmpx - (float)(fj*cosa + fi*sina);
         if(gd->y_stretch != 0.0f)
            tmpy = tmpy - (float)(fi*cosa - fj*sina);

         new_j = (int)tmpx;
         new_i = (int)tmpy;

				 if(new_j < 0       ||
						new_j >= gd->cols   ||
						new_i < 0       ||
						new_i >= gd->rows)
						gd->out_image[(i*gd->cols)+j] = FILL;
         else
            gd->out_image[(i*gd->cols)+j] =
                gd->the_image[(new_i*gd->cols)+new_j];								
								
         //if(gd->bilinear == 0){
         //   //if(new_j < 0       ||
         //   //   new_j >= gd->cols   ||
         //   //   new_i < 0       ||
         //   //   new_i >= gd->rows)
         //   //   gd->out_image[(i*gd->cols)+j] = FILL;
         //   //else
         //      gd->out_image[(i*gd->cols)+j] =
         //       gd->the_image[(new_i*gd->cols)+new_j];
         //}  /* ends if bilinear */
         //else{
         //   gd->out_image[(i*gd->cols)+j] =
         //      bilinear_interpolate(gd->the_image,
         //                           tmpx, tmpy);
         //}  /* ends bilinear if */

      }  /* ends loop over j */
   }  /* ends loop over i */

}  /* ends geometry2 */


/*
*
*   arotate(..
*
*   This routine performs rotation about
*   any point m,n.
*
*   The basic equations are:
*
*   new x = x.cos(a) - y.sin(a)
*           -m.cos(a) + m + n.sin(a)
*
*   new y = y.cos(a) + x.sin(a)
*           -m.sin(a) - n.cos(a) + n
*
*/
void arotate2(ROTDATA *rd)
{
   float cosa, sina, radian_angle, tmpx, tmpy;
   int    i, j, new_i, new_j;


      /* the following magic number is from
         180 degrees divided by pi */
   radian_angle = (float)rd->angle/57.29577951f;
   cosa  = (float)cos(radian_angle);
   sina  = (float)sin(radian_angle);

      /**************************
      *
      *   Loop over image array
      *
      **************************/

   for(i=0; i<rd->rows; i++){
      for(j=0; j<rd->cols; j++){

     /******************************************
     *
     *   new x = x.cos(a) - y.sin(a)
     *           -m.cos(a) + m + n.sin(a)
     *
     *   new y = y.cos(a) + x.sin(a)
     *           -m.sin(a) - n.cos(a) + n
     *
     *******************************************/

         tmpx = (float)(j)*cosa    -
                (float)(i)*sina    -
                (float)(rd->m)*cosa    +
                (float)(rd->m)         +
                (float)(rd->n)*sina;

         tmpy = (float)(i)*cosa    +
                (float)(j)*sina    -
                (float)(rd->m)*sina    -
                (float)(rd->n)*cosa    +
                (float)(rd->n);

         new_j = tmpx;
         new_i = tmpy;

         if(rd->bilinear == 0){
            if(new_j < 0       ||
               new_j >= rd->cols   ||
               new_i < 0       ||
               new_i >= rd->rows)
               rd->out_image[(i*rd->cols)+j] = FILL;
            else
               rd->out_image[(i*rd->cols)+j] =
                rd->the_image[(new_i*rd->cols)+new_j];
         }  /* ends if bilinear */
         else{
            rd->out_image[(i*rd->cols)+j] =
               bilinear_interpolate(rd->the_image,
                                    tmpx, tmpy);
         }  /* ends bilinear if */

      }  /* ends loop over j */
   }  /* ends loop over i */

}  /* ends arotate2 */

 /*
 *
 *   bilinear_interpolate(..
 *
 *   This routine performs bi-linear
 *   interpolation.
 *
 *   If x or y is out of range, i.e. less
 *   than zero or greater than rows or cols,
 *   this routine returns a zero.
 *
 *   If x and y are both in range, this
 *   routine interpolates in the horizontal
 *   and vertical directions and returns
 *   the proper gray level.
 *
 */
#define auxCeil(x) ((float)(int)((x)+1))
//int bilinear_interpolate(unsigned char *the_image, float x, float y, int rows, int cols)
int bilinear_interpolate(unsigned char *the_image, float x, float y)
{
   float fraction_x, fraction_y,
          one_minus_x, one_minus_y,
          tmp_float;
   int    ceil_x, ceil_y, floor_x, floor_y;
   int  p1, p2, p3, result = FILL;
	 float cyc, fyc;
      /******************************
      *
      *   If x or y is out of range,
      *   return a FILL.
      *
      *******************************/

   //if(x < 0.0f               ||
   //   x >= (float)(cols-1)   ||
   //   y < 0.0f               ||
   //   y >= (float)(rows-1))
   //   return(result);

   //tmp_float = floor(x);
   //floor_x    = (int)tmp_float;
	 //fraction_x = x - tmp_float;
	 floor_x = (int)x;
	 fraction_x = x - floor_x;
		
   //tmp_float = floor(y);
   //floor_y    = (int)tmp_float;
	 //fraction_y = y - tmp_float;
	 floor_y = (int)y;
	 fraction_y = y - floor_y;
		
   tmp_float = auxCeil(x);
   ceil_x     = (int)tmp_float;
   tmp_float = auxCeil(y);
   ceil_y     = (int)tmp_float; 

   one_minus_x = 1.0f - fraction_x;
   one_minus_y = 1.0f - fraction_y;
	 
   tmp_float = one_minus_x *
          (float)(the_image[(floor_y*COLS)+floor_x]) +
          fraction_x *
          (float)(the_image[(floor_y*COLS)+ceil_x]);
   p1         = (int)tmp_float;

   tmp_float = one_minus_x *
          (float)(the_image[(ceil_y*COLS)+floor_x]) +
          fraction_x *
          (float)(the_image[(ceil_y*COLS)+ceil_x]);
   p2         = (int)tmp_float;

   tmp_float = one_minus_y * (float)(p1) +
          fraction_y * (float)(p2);
   p3         = (int)tmp_float;


   return(p3);

}  /* ends bilinear_interpolate */
