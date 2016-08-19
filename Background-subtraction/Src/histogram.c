#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "histogram.h"

/*
  Example of usage (COPS7.c)
   zero_histogram(histogram);
   calculate_histogram(the_image, histogram);
   smooth_histogram(histogram);
   find_peaks(histogram, &peak1, &peak2);
   peaks_high_low(histogram, peak1, peak2,
                  &hi, &low);
   threshold_image_array(the_image, out_image,
                         hi, low, value);

*/

						 
/*****************************************
*
*   zero_histogram(...
*
*   This function clears or zeros a
*   histogram array.
*
******************************************/
void zero_histogram(uint32_t*histogram)
{
  // int i;
  // for(i=0; i<GRAY_LEVELS; i++)
  //    histogram[i] = 0;
	
	memset( histogram, 0, sizeof(uint32_t)*GRAY_LEVELS);
}  /* ends zero_histogram */




/*****************************************
*
*   calculate_histogram(...
*
*   This function calculates the histogram
*   for an input image array.
*
******************************************/
void calculate_histogram( uint8_t*image, uint32_t*histogram)
{
   long  i,j;
   short k;
   
   for(i=0; i<ROWS; i++) {
     for(j=0; j<COLS; j++) {
       k = image[(i*COLS)+j];
	     histogram[k] = histogram[k] + 1;
	   }
   }
}  /* ends calculate_histogram */




/********************************************
*
*   smooth_histogram(...
*
*   This function smoothes the input histogram
*   and returns it.  It uses a simple averaging
*   scheme where each point in the histogram
*   is replaced by the average of itself and
*   the two points on either side of it.
*
*********************************************/
void smooth_histogram(uint32_t*histogram)
{
   int i;
   uint32_t*new_hist;

	 new_hist=(uint32_t*)malloc( GRAY_LEVELS*sizeof(uint32_t));
	
   zero_histogram(new_hist);

   new_hist[0] = (histogram[0] + histogram[1])/2;
   new_hist[GRAY_LEVELS-1] =
      (histogram[GRAY_LEVELS-1] +
       histogram[GRAY_LEVELS-2])/2;

   for(i=1; i<GRAY_LEVELS-1; i++){
      new_hist[i] = (histogram[i-1] +
                     histogram[i]   +
                     histogram[i+1])/3;
   }

   for(i=0; i<GRAY_LEVELS; i++)
      histogram[i] = new_hist[i];

	 //
	 free( new_hist);
}  /* ends smooth_histogram */




/*****************************************
*
*    perform_histogram_equalization(...
*
*    This function performs histogram
*    equalization on the input image array.
*
******************************************/
void perform_histogram_equalization(uint8_t*image, 
                               uint32_t*histogram,
                               int32_t new_grays)
{
   int i,
       j,
       k;
   uint32_t sum;
	 uint32_t*sum_of_h;

   float constant;

	 sum_of_h=(uint32_t*)malloc( GRAY_LEVELS*sizeof(uint32_t));
   sum = 0;
   for(i=0; i<GRAY_LEVELS; i++){
      sum         = sum + histogram[i];
      sum_of_h[i] = sum;
   }

      /* constant = new # of gray levels div by area */
   constant = (float)(new_grays)/(float)(ROWS*COLS);
   for(i=0; i<ROWS; i++){
      for(j=0; j<COLS; j++){
         k           = image[(i*COLS)+j];
         image[(i*COLS)+j] = sum_of_h[k] * constant;
      }
   }
	 //
	 free( sum_of_h);
}  /* ends perform_histogram_equalization */

						 
/********************************************
*
*   find_peaks(...
*
*   This function looks through the histogram
*   array and finds the two highest peaks.
*   The peaks must be separated, cannot be
*   next to each other, by a spacing defined
*   in cips.h.
*
*   The peaks array holds the peak value
*   in the first place and its location in
*   the second place.
*
*********************************************/
void find_peaks(uint32_t*histogram, uint32_t*peak1, uint32_t*peak2)
{
   int distance[PEAKS], peaks[PEAKS][2];
   int i, j=0, max=0, max_place=0;

   for(i=0; i<PEAKS; i++){
      distance[i] =  0;
      peaks[i][0] = -1;
      peaks[i][1] = -1;
   }

   for(i=0; i<=GRAY_LEVELS; i++){
      max       = histogram[i];
      max_place = i;
      insert_into_peaks(peaks, max, max_place);
   }  /* ends loop over i */

   for(i=1; i<PEAKS; i++){
      distance[i] = peaks[0][1] - peaks[i][1];
      if(distance[i] < 0)
         distance[i] = distance[i]*(-1);
   }

   *peak1 = peaks[0][1];
   for(i=PEAKS-1; i>0; i--)
    if(distance[i] > PEAK_SPACE) *peak2 = peaks[i][1];

}  /* ends find_peaks */

						 
						 
						 
/********************************************
*
*   peaks_high_low(...
*
*   This function uses the histogram array
*   and the peaks to find the best high and
*   low threshold values for the threshold
*   function.  You want the hi and low values
*   so that you will threshold the image around
*   the smaller of the two "humps" in the
*   histogram.  This is because the smaller
*   hump represents the objects while the
*   larger hump represents the background.
*
*********************************************/
void peaks_high_low(uint32_t*histogram, uint32_t peak1, uint32_t peak2, uint32_t*hi, uint32_t*low)
{
   int32_t i, mid_point;
   uint32_t sum1 = 0, sum2 = 0;

   if(peak1 > peak2)
      mid_point = ((peak1 - peak2)/2) + peak2;
   if(peak1 < peak2)
      mid_point = ((peak2 - peak1)/2) + peak1;

   for(i=0; i<mid_point; i++)
      sum1 = sum1 + histogram[i];

   for(i=mid_point; i<=GRAY_LEVELS; i++)
      sum2 = sum2 + histogram[i];
   if(sum1 >= sum2){
      *low = mid_point;
      *hi  = GRAY_LEVELS;
   }
   else{
      *low = 0;
      *hi  = mid_point;
   }

}  /* ends peaks_high_low */


/**************************************************
*
*   threshold_image_array(...
*
*   This function thresholds an input image array
*   and produces a binary output image array.
*   If the pixel in the input array is between
*   the hi and low values, then it is set to value.
*   Otherwise, it is set to 0.
*
***************************************************/
void threshold_image_array(uint8_t*in_image, uint8_t*out_image, uint32_t hi, uint32_t low, uint32_t value)
{
   int   counter = 0, i, j;
   for(i=0; i<ROWS; i++){
      for(j=0; j<COLS; j++){
         if(in_image[(i*COLS)+j] >= low  &&
            in_image[(i*COLS)+j] <= hi){
            out_image[(i*COLS)+j] = value;
            counter++;
         }
         else
            out_image[(i*COLS)+j] = 0;
      }  /* ends loop over j */
   }  /* ends loop over i */
   // printf("\n\tTIA> set %d points", counter);
}  /* ends threshold_image_array */

/********************************************
*
*   insert_into_peaks(...
*
*   This function takes a value and its
*   place in the histogram and inserts them
*   into a peaks array.  This helps us rank
*   the the peaks in the histogram.
*
*   The objective is to build a list of
*   histogram peaks and thier locations.
*
*   The peaks array holds the peak value
*   in the first place and its location in
*   the second place.
*
*********************************************/
void insert_into_peaks(int peaks[PEAKS][2], int max, int max_place)
{
   int i, j;

      /* first case */
   if(max > peaks[0][0]){
      for(i=PEAKS-1; i>0; i--){
         peaks[i][0] = peaks[i-1][0];
         peaks[i][1] = peaks[i-1][1];
      }
      peaks[0][0] = max;
      peaks[0][1] = max_place;
   }  /* ends if */

      /* middle cases */
   for(j=0; j<PEAKS-3; j++){
      if(max < peaks[j][0]  && max > peaks[j+1][0]){
         for(i=PEAKS-1; i>j+1; i--){
            peaks[i][0] = peaks[i-1][0];
            peaks[i][1] = peaks[i-1][1];
         }
         peaks[j+1][0] = max;
         peaks[j+1][1] = max_place;
      }  /* ends if */
   }  /* ends loop over j */
      /* last case */
   if(max < peaks[PEAKS-2][0]  && 
      max > peaks[PEAKS-1][0]){
      peaks[PEAKS-1][0] = max;
      peaks[PEAKS-1][1] = max_place;
   }  /* ends if */

}  /* ends insert_into_peaks */
