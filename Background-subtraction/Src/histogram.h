#include <stdint.h>

#define PEAKS                 30

void zero_histogram(uint32_t*histogram);
void calculate_histogram( uint8_t*image, uint32_t*histogram);
void smooth_histogram(uint32_t*histogram);

void perform_histogram_equalization(uint8_t*image, 
                               uint32_t*histogram,
                               int32_t new_grays);

void find_peaks(uint32_t*histogram, uint32_t*peak1, uint32_t*peak2);
void peaks_high_low(uint32_t*histogram, uint32_t peak1, uint32_t peak2, uint32_t*hi, uint32_t*low);
void threshold_image_array(uint8_t*in_image, uint8_t*out_image, uint32_t hi, uint32_t low, uint32_t value);
void insert_into_peaks(int peaks[PEAKS][2], int max, int max_place);

