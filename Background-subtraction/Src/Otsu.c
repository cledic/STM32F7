/*
* http://www.codeproject.com/Articles/38319/Famous-Otsu-Thresholding-in-C
* by Tolga Birdal
* OtsuThreshold 
*/
#include "main.h"
#include "string.h"
#include "stdlib.h"
#include "Otsu.h"

/* Private functions */
float Px(int32_t init, int32_t end, int32_t* hist);
float Mx(int32_t init, int32_t end, int32_t* hist);
int32_t findMax(float*vec, int32_t n);
void getHistogram(uint8_t* p, int32_t* hist);

// function is used to compute the q values in the equation
float Px(int32_t init, int32_t end, int32_t* hist)
{
	int32_t sum = 0;
	int32_t i;
	
	for (i = init; i <= end; i++)
		sum += hist[i];

	return (float)sum;
}

// function is used to compute the mean values in the equation (mu)
float Mx(int32_t init, int32_t end, int32_t* hist)
{
	int32_t sum = 0;
	int32_t i;
	
	for (i = init; i <= end; i++)
		sum += i * hist[i];

	return (float)sum;
}

// finds the maximum element in a vector
int32_t findMax(float*vec, int32_t n)
{
	float maxVec = 0;
	int32_t idx=0;
	int32_t i;

	for (i = 1; i < n - 1; i++)
	{
		if (vec[i] > maxVec)
		{
			maxVec = vec[i];
			idx = i;
		}
	}
	return idx;
}

// simply computes the image histogram
void getHistogram(uint8_t* p, int32_t* hist)
{
	int32_t index, i, j;
	
	for ( i = 0; i < ROWS; i++)
	{
		for ( j = 0; j < COLS; j++)
		{
			index = (i*COLS)+j;
			hist[p[index]]++;
		}
	}
}

// find otsu threshold
int8_t getOtsuThreshold(uint8_t*p)
{
	float p1, p2, p12, diff;
	int32_t k;

	int8_t t=0;
	float*vet;
	int32_t*hist;
	
	vet=(float*)malloc(sizeof(float) * 256);
	if ( vet==(float*)NULL)
		while(1);
	memset(vet, 0, sizeof(float) * 256);

	hist=(int32_t*)malloc(sizeof(int32_t) * 256);
	if ( hist==(int32_t*)NULL)
		while(1);
	memset(hist, 0, sizeof(int32_t) * 256);
	
	getHistogram( p, hist);

	// loop through all possible t values and maximize between class variance
	for (k = 1; k != 255; k++)
	{
		p1 = Px(0, k, hist);
		p2 = Px(k + 1, 255, hist);
		p12 = p1 * p2;
		if (p12 == 0) 
			p12 = 1;
		diff=(Mx(0, k, hist) * p2) - (Mx(k + 1, 255, hist) * p1);
		vet[k] = (float)((diff * diff) / p12);
	}

	t = (int8_t)findMax(vet, 256);

	free(vet);
	free(hist);
	
	return t;
}

// simple routine for thresholdin
void OtsuThreshold2(uint8_t*p, int thresh, uint8_t*b)
{ 
	
	int32_t i, j, t;
	
	for (i = 0; i < ROWS; i++)
	{
		for (j = 0; j < COLS; j++)
		{
			//p[(i*COLS)+j] = (uint8_t)((p[(i*COLS)+j] > (uint8_t)thresh) ? 255 : 0);
			t = (i*COLS)+j;
			p[t] = (uint8_t)((p[t] > (uint8_t)thresh) ? p[t] : 0);
			//
			if ( p[t] > b[t]) {
				b[t] = b[t] + 1;
			} else {
				if ( p[t] < b[t]) {
					b[t] = b[t] - 1;
				}
			}
		}
	}
}

void OtsuThreshold(uint8_t*p, int thresh)
{ 
	
	int32_t i, j;
	
	for (i = 0; i < ROWS; i++)
	{
		for (j = 0; j < COLS; j++)
		{
			//p[(i*COLS)+j] = (uint8_t)((p[(i*COLS)+j] > (uint8_t)thresh) ? 255 : 0);
			p[(i*COLS)+j] = (uint8_t)((p[(i*COLS)+j] > (uint8_t)thresh) ? p[(i*COLS)+j] : 0);
		}
	}
}

// simple routine for thresholdin
void OtsuThresholdGray(uint8_t*p, int thresh)
{ 
	
	int32_t i, j;
	
	for (i = 0; i < ROWS; i++)
	{
		for (j = 0; j < COLS; j++)
		{
			p[(i*COLS)+j] = (uint8_t)((p[(i*COLS)+j] > (uint8_t)thresh) ? p[(i*COLS)+j] : 0);
		}
	}
}

