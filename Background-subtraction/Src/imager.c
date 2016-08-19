#include "main.h"
#include <string.h>
#include <stdlib.h>
#include "imager.h"

int enhance_mask[3][3] =  {
       {-1,  0, -1},
       { 0,  4,  0},
       {-1,  0, -1} };

int g7[7][7] = {
     {  0,  0, -1, -1, -1,  0,  0},
     {  0, -2, -3, -3, -3, -2,  0},
     { -1, -3,  5,  5,  5, -3, -1},
     { -1, -3,  5, 16,  5, -3, -1},
     { -1, -3,  5,  5,  5, -3, -1},
     {  0, -2, -3, -3, -3, -2,  0},
     {  0,  0, -1, -1, -1,  0,  0}};

int g9[9][9] = {
   {  0,  0,  0,  -1, -1, -1,  0,  0,  0},
   {  0, -2, -3,  -3, -3, -3, -3, -2,  0},
   {  0, -3, -2,  -1, -1, -1, -2, -3,  0},
   { -1, -3, -1,   9,  9,  9, -1, -3, -1},
   { -1, -3, -1,   9, 19,  9, -1, -3, -1},
   { -1, -3, -1,   9,  9,  9, -1, -3, -1},
   {  0, -3, -2,  -1, -1, -1, -2, -3,  0},
   {  0, -2, -3,  -3, -3, -3, -3, -2,  0},
   {  0,  0,  0,  -1, -1, -1,  0,  0,  0}};

int edmask1[3][3] = {{0, 1, 0},
                       {0, 1, 0},
                       {0, 1, 0}};

int edmask2[3][3] = {{0, 0, 0},
                       {1, 1, 1},
                       {0, 0, 0}};

int edmask3[3][3] = {{0, 1, 0},
                       {1, 1, 1},
                       {0, 1, 0}};

int edmask4[3][3] = {{1, 1, 1},
                       {1, 1, 1},
                       {1, 1, 1}};

#if 0
void fix_edges( char*im, int w);
void erosion( unsigned char*the_image, unsigned char*out_image, int value, int threshold);
void dilation(unsigned char*the_image, unsigned char*out_image, int value, int threshold);
void copy_result( unsigned char*a, unsigned char*b);
void enhance_edges(unsigned char*the_image, unsigned char*out_image, int high);
void gaussian_edge(unsigned char*the_image, unsigned char*out_image, int size, int threshold, int high);
void mask_dilation(unsigned char*the_image, unsigned char*out_image, int value, int mask_type);
void mask_erosion(unsigned char*the_image, unsigned char*out_image, int value, int mask_type);
void copy_3_x_3(unsigned char*a, unsigned char*b);
#endif

#if 0
    erosion( the_image, out_image, 10, 2);
    copy_result( the_image, out_image);
    dilation( the_image, out_image, 50, 2);
    copy_result( the_image, out_image);

    enhance_edges( the_image, out_image, 1);
    copy_result( the_image, out_image);
    erosion( the_image, out_image, 1, 2);

    gaussian_edge( the_image, out_image, 7, 0, 1);

    mask_erosion( the_image, out_image, , 1);
    dilation( the_image, out_image, 50, 2);
    enhance_edges( the_image, out_image, 1);
    gaussian_edge( the_image, out_image, 7, 5, 1);
#endif


void copy_result( uint8_t*a, uint8_t*b)
{
	int i, j;
	
	// memcpy( a, b, sizeof( uint8_t) * ROWS * COLS);

	for(i=0; i<ROWS; i++)
	{
			for(j=0; j<COLS; j++)
			{
					a[(i*COLS)+j]=b[(i*COLS)+j];
			}
	}

}

//  bg - img
void image_difference( unsigned char*image1, unsigned char*image2)
{
	int i, tmp;
	
	for(i=0; i<(ROWS*COLS); i++) {
		//image2[i] = image1[i] - image2[i];
		tmp = (int)((int)image2[i] - (int)image1[i]);
		tmp = abs( tmp);
		if (tmp>255) tmp=(uint8_t)255;
		image2[i] = (uint8_t)tmp;
		// image2[i] = abs(image2[i] - image1[i]);
	}
	
}

/*
*
* erosion(...
*
* This function performs the erosion
* operation.  If a value pixel has more
* than the threshold number of 0
* neighbors, you erode it by setting it
* to 0.
*
*/
void erosion( unsigned char*the_image, unsigned char*out_image, int value, int threshold)
{
	int    a, b, count, i, j;

	/*
	*   Loop over image array
	*/

	for(i=0; i<ROWS; i++)
		for(j=0; j<COLS; j++)
			out_image[(i*COLS)+j] = the_image[(i*COLS)+j];


   for(i=1; i<ROWS-1; i++){
      //
      for(j=1; j<COLS-1; j++){
         if(the_image[(i*COLS)+j] == value){
            count = 0;
            for(a=-1; a<=1; a++){
                for(b=-1; b<=1; b++){
                      if(the_image[((i+a)*COLS)+j+b] == 0)
                         count++;
                }  /*  ends loop over b */
            }  /* ends loop over a */
            if(count > threshold) out_image[(i*COLS)+j] = 0;
         }  /* ends if the_image == value */
      }  /* ends loop over j */
   }  /* ends loop over i */

   // fix_edges(out_image, 3);

}  /* ends erosion */



/*
*
* dilation(...
*
* This function performs the dilation
* operation.  If a 0 pixel has more than
* threshold number of value neighbors,
* you dilate it by setting it to value.
*
*/
void dilation(unsigned char*the_image, unsigned char*out_image, int value, int threshold)
{
   int    a, b, count, i, j;


	/*
	*   Loop over image array
	*/

   for(i=1; i<ROWS-1; i++){
      //
      for(j=1; j<COLS-1; j++){
         out_image[(i*COLS)+j] = the_image[(i*COLS)+j];
         if(the_image[(i*COLS)+j] == 0){
            count = 0;
            for(a=-1; a<=1; a++){
                for(b=-1; b<=1; b++){
                   if(a!=0  &&  b!=0){
                      if(the_image[((i+a)*COLS)+j+b] == value)
                         count++;
                   }  /* ends avoid the center pixel */
                }  /*  ends loop over b */
            }  /* ends loop over a */
            if(count > threshold)
               out_image[(i*COLS)+j] = value;
         }  /* ends if the_image == 0 */
      }  /* ends loop over j */
   }  /* ends loop over i */

   // fix_edges(out_image, 3);

}  /* ends dilation */

#if 0
/*
*
* fix_edges(...
*
* This function fixes the edges of an image
* array after convolution was performed.
* It copies the points near the edge of the
* array out to the edge of the array.
*
*/
void fix_edges(char*im, int w)
{
   int i, j;


      /* four corners */
   for(i=w; i>0; i--){
      im[i-1][i-1] = im[i][i];
      im[i-1][COLS-(i-1)] = im[i][COLS-1-(i-1)];
      im[ROWS-(i-1)][i-1] = im[ROWS-1-(i-1)][i];
      im[ROWS-(i-1)][COLS-(i-1)] = im[ROWS-1-(i-1)][COLS-1-(i-1)];
   }  /* ends four corners loop */

   for(i=0; i<ROWS; i++){
      for(j=w; j>0; j--){
         im[i][j-1] = im[i][j];
         im[i][COLS-j] = im[i][COLS-j-1];
      }
   }

   for(j=0; j<COLS; j++){
      for(i=w; i>0; i--){
         im[i-1][j] = im[i][j];
         im[ROWS-i][j] = im[ROWS-i-1][j];
      }
   }

}  /* ends fix_edges */
#endif // 0

 /*
 *
 * enhance_edges(...
 *
 * This function enhances the edges in an
 * input image and writes the enhanced
 * result to an output image.  It operates
 * much the same way as detect_edges
 * except it uses only one type of mask.
 *
 * The threshold and high parameters perform
 * a different role in this function.  The
 * threshold parameter does not exist.  The
 * high parameter determines if the edge is
 * strong enough to enhance or change the
 * input image.
 *
 */
void enhance_edges(unsigned char*the_image, unsigned char*out_image, int high)
{
   int a, b, i, j, max, sum;

   max = 255;

   /* Do convolution over image array */
   for(i=1; i<ROWS-1; i++){
      // if( (i%10) == 0) printf("%d ", i);
      for(j=1; j<COLS-1; j++){
         sum = 0;
         for(a=-1; a<2; a++){
            for(b=-1; b<2; b++){
               sum = sum +
                     the_image[((i+a)*COLS)+j+b] *
                     enhance_mask[a+1][b+1];
            }
         }
         if(sum < 0)   sum = 0;
         if(sum > max) sum = max;
         if(sum > high)
            out_image[(i*COLS)+j] = max;
         else
            out_image[(i*COLS)+j] = the_image[(i*COLS)+j];
      }  /* ends loop over j */
   }  /* ends loop over i */

}  /* ends enhance_edges */

/*
*
*   gaussian_edge(...
*
*
*/
void gaussian_edge(unsigned char*the_image, unsigned char*out_image, int size, int threshold, int high)
{
   long sum;
   int  a, b, i, j,
        lower, max, new_hi, new_low,
        starti, stopi, startj, stopj,
        upper;

   new_hi  = 250;
   new_low = 16;

   max = 255;


   if(size == 7){
      lower = -3;
      upper =  4;
      starti =  3;
      startj =  3;
      stopi  =  ROWS-3;
      stopj  =  COLS-3;
   }

   if(size == 9){
      lower = -4;
      upper =  5;
      starti =  4;
      startj =  4;
      stopi  =  ROWS-4;
      stopj  =  COLS-4;
   }


   for(i=0; i<ROWS; i++)
      for(j=0; j<COLS; j++)
         out_image[(i*COLS)+j] = 0;


   for(i=starti; i<stopi; i++) {
	  //
      for(j=startj; j<stopj; j++) {
          //
          sum = 0;
          for(a=lower; a<upper; a++){
             for(b=lower; b<upper; b++){
                if(size == 7)
                   sum = sum + the_image[((i+a)*COLS)+j+b] *
                         g7[a+3][b+3];
                if(size == 9)
                   sum = sum + the_image[((i+a)*COLS)+j+b] *
                         g9[a+4][b+4];
             } /* ends loop over a */
          }  /* ends loop over b */

          if(sum < 0) sum = 0;
          if(sum > max) sum = max;
          out_image[(i*COLS)+j] = sum;

      }  /* ends loop over j */
   }  /* ends loop over i */

     /* if desired, threshold the output image */
   if(threshold == 1){
       for(i=0; i<ROWS; i++){
          for(j=0; j<COLS; j++){
             if(out_image[(i*COLS)+j] > high){
                  out_image[(i*COLS)+j] = new_hi;
             }
             else{
                  out_image[(i*COLS)+j] = new_low;
             }
          }
       }
   }  /* ends if threshold == 1 */

} /* ends gaussian_edge */

 /*
 *
 *   mask_dilation(...
 *
 *   This function performs the dilation
 *   operation using the erosion-dilation
 *   3x3 masks given above.  It works on
 *   0-value images.
 *
 */
void mask_dilation(unsigned char*the_image, unsigned char*out_image, int value, int mask_type)
{
   int    a, b, i, j;
   char  mask[3][3], max;

      /*
      *
      *   Copy the 3x3 erosion-dilation mask
      *   specified by the mask_type.
      *
      */

   switch(mask_type){
      case 1:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask1[i][j];
         break;
      case 2:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask2[i][j];
         break;
      case 3:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask3[i][j];
         break;
      case 4:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask4[i][j];
         break;
      default:
         //printf("\nInvalid mask type, using mask 4");
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask4[i][j];
         break;
   }

      /*
      *   Loop over image array
      */

   for(i=1; i<ROWS-1; i++){
      //
      for(j=1; j<COLS-1; j++){
         max = 0;
         for(a=-1; a<=1; a++){
             for(b=-1; b<=1; b++){
                if(mask[a+1][b+1] == 1){
                   if(the_image[((i+a)*COLS)+j+b] > max)
                      max = the_image[((i+a)*COLS)+j+b];
                }  /* ends if mask == 1 */
             }  /*  ends loop over b */
         }  /* ends loop over a */
         out_image[(i*COLS)+j] = max;
      }  /* ends loop over j */
   }  /* ends loop over i */

   //fix_edges(out_image, 3);

}  /* ends mask_dilation */





/*
*
*   mask_erosion(...
*
*   This function performs the erosion
*   operation using the erosion-dilation
*   3x3 masks given above.  It works on
*   0-value images.
*
*/
void mask_erosion(unsigned char*the_image, unsigned char*out_image, int value, int mask_type)
{
   int a, b, i, j;
   int mask[3][3];
	 int min;

      /*
      *   Copy the 3x3 erosion-dilation mask
      *   specified by the mask_type.
      */

   switch(mask_type){
      case 1:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask1[i][j];
         break;
      case 2:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask2[i][j];
         break;
      case 3:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask3[i][j];
         break;
      case 4:
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask4[i][j];
         break;
      default:
         //printf("\nInvalid mask type, using mask 4");
         for(i=0; i<3; i++)
          for(j=0; j<3; j++)
             mask[i][j] = edmask4[i][j];
         break;
   }

      /*
      *   Loop over image array
      */

   for(i=1; i<ROWS-1; i++){
      //
      for(j=1; j<COLS-1; j++){
         min = value;
         for(a=-1; a<=1; a++){
             for(b=-1; b<=1; b++){
                if(mask[a+1][b+1] == 1){
                   if(the_image[((i+a)*COLS)+j+b] < min)
                      min = the_image[((i+a)*COLS)+j+b];
                }  /* ends if mask == 1 */
             }  /*  ends loop over b */
         }  /* ends loop over a */
         out_image[(i*COLS)+j] = min;
      }  /* ends loop over j */
   }  /* ends loop over i */

   //fix_edges(out_image, 3);

}  /* ends mask_erosion */


