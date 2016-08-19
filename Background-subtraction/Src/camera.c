/**
  ******************************************************************************
  * @file    BSP/Src/camera.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This example code shows how to use the camera feature in the
  *          stm32446e_eval driver
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>
#include "stm32746g_discovery_audio.h"

#include "stdlib.h"
#include "arm_math.h"
#include "arm_const_structs.h"

// Imager
#include "geom.h"
#include "imager.h"
#include "warp.h"
#include "histogram.h"
#include "Otsu.h"

//#include "rocks_image.h"
//#include "spioncino_image.h"
//#include "abba_320x240.h"
#include "volto.h"

/* ************************************************************************** */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
FRESULT res; 
uint32_t bytesread;

uint32_t cp[]={
	0x000000, 	0x000066, 	0x0000ff, 	0x006600, 	0x006666, 	0x0066ff,
	0x00ff00, 	0x00ff66, 	0x00ffff, 	0xff0000, 	0xff0066, 	0xff00FF,
	0xff6600, 	0xff6666, 	0xff66ff, 	0xffff00, 	0xffff66, 	0xffffFF,
	0x009900, 	0x009933, 	0x009966, 	0x009999, 	0x0099CC, 	0x0099FF,
	0x00CC00, 	0x00CC33, 	0x00CC66, 	0x00CC99, 	0x00CCCC, 	0x00CCFF,
	0x00FF00, 	0x00FF33, 	0x00FF66, 	0x00FF99, 	0x00FFCC, 	0x00FFFF,
	0x330000, 	0x330033, 	0x330066, 	0x330099, 	0x3300CC, 	0x3300FF,
	0x333300, 	0x333333, 	0x333366, 	0x333399, 	0x3333CC, 	0x3333FF,
	0x336600, 	0x336633, 	0x336666, 	0x336699, 	0x3366CC, 	0x3366FF,
	0x339900, 	0x339933, 	0x339966, 	0x339999, 	0x3399CC, 	0x3399FF,
	0x33CC00, 	0x33CC33, 	0x33CC66, 	0x33CC99, 	0x33CCCC, 	0x33CCFF,
	0x33FF00, 	0x33FF33, 	0x33FF66, 	0x33FF99, 	0x33FFCC, 	0x33FFFF,
	0x660000, 	0x660033, 	0x660066, 	0x660099, 	0x6600CC, 	0x6600FF,
	0x663300, 	0x663333, 	0x663366, 	0x663399, 	0x6633CC, 	0x6633FF,
	0x666600, 	0x666633, 	0x666666, 	0x666699, 	0x6666CC, 	0x6666FF,
	0x669900, 	0x669933, 	0x669966, 	0x669999, 	0x6699CC, 	0x6699FF,
	0x66CC00, 	0x66CC33, 	0x66CC66, 	0x66CC99, 	0x66CCCC, 	0x66CCFF,
	0x66FF00, 	0x66FF33, 	0x66FF66, 	0x66FF99, 	0x66FFCC, 	0x66FFFF,
	0x990000, 	0x990033, 	0x990066, 	0x990099, 	0x9900CC, 	0x9900FF,
	0x993300, 	0x993333, 	0x993366, 	0x993399, 	0x9933CC, 	0x9933FF,
	0x996600, 	0x996633, 	0x996666, 	0x996699, 	0x9966CC, 	0x9966FF,
	0x999900, 	0x999933, 	0x999966, 	0x999999, 	0x9999CC, 	0x9999FF,
	0x99CC00, 	0x99CC33, 	0x99CC66, 	0x99CC99, 	0x99CCCC, 	0x99CCFF,
	0x99FF00, 	0x99FF33, 	0x99FF66, 	0x99FF99, 	0x99FFCC, 	0x99FFFF,
	0xCC0000, 	0xCC0033, 	0xCC0066, 	0xCC0099, 	0xCC00CC, 	0xCC00FF,
	0xCC3300, 	0xCC3333, 	0xCC3366, 	0xCC3399, 	0xCC33CC, 	0xCC33FF,
	0xCC6600, 	0xCC6633, 	0xCC6666, 	0xCC6699, 	0xCC66CC, 	0xCC66FF,
	0xCC9900, 	0xCC9933, 	0xCC9966, 	0xCC9999, 	0xCC99CC, 	0xCC99FF,
	0xCCCC00, 	0xCCCC33, 	0xCCCC66, 	0xCCCC99, 	0xCCCCCC, 	0xCCCCFF,
	0xCCFF00, 	0xCCFF33, 	0xCCFF66, 	0xCCFF99, 	0xCCFFCC, 	0xCCFFFF,
	0xFF0000, 	0xFF0033, 	0xFF0066, 	0xFF0099, 	0xFF00CC, 	0xFF00FF,
	0xFF3300, 	0xFF3333, 	0xFF3366, 	0xFF3399, 	0xFF33CC, 	0xFF33FF,
	0xFF6600, 	0xFF6633, 	0xFF6666, 	0xFF6699, 	0xFF66CC, 	0xFF66FF,
	0xFF9900, 	0xFF9933, 	0xFF9966, 	0xFF9999, 	0xFF99CC, 	0xFF99FF,
	0xFFCC00, 	0xFFCC33, 	0xFFCC66, 	0xFFCC99, 	0xFFCCCC, 	0xFFCCFF,
	0xFFFF00, 	0xFFFF33, 	0xFFFF66, 	0xFFFF99, 	0xFFFFCC, 	0xFFFFFF,
};

#define CALL_LabelComponent(x,y,returnLabel) { STACK[mSP] = x; STACK[mSP+1] = y; STACK[mSP+2] = returnLabel; mSP += 3; goto START; }
#define RETURN { mSP -= 3;                \
                 switch (STACK[mSP+2])    \
                 {                       \
                 case 1 : goto RETURN1;  \
                 case 2 : goto RETURN2;  \
                 case 3 : goto RETURN3;  \
                 case 4 : goto RETURN4;  \
                 default: return;        \
                 }                       \
               }
#define X (STACK[mSP-3])
#define Y (STACK[mSP-2])

void LabelImage(unsigned short width, unsigned short height, unsigned char* input, unsigned char* output);
void LabelComponent(unsigned short* STACK, unsigned short width, unsigned short height, 
										unsigned char* input, unsigned char* output, int labelNo, unsigned short x, unsigned short y);

/** @addtogroup STM32F7xx_HAL_Examples
  * @{
  */

/** @addtogroup BSP
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/**
 *  @brief TS_CommandTypeDef
 *  Define Possible touch screen commands
 */
typedef enum
{
  TS_NO_COMMAND     = 0x00,
  TS_BRIGHTNESS_INC = 0x01,
  TS_BRIGHTNESS_DEC = 0x02,
  TS_CONTRAST_INC   = 0x03,
  TS_CONTRAST_DEC   = 0x04,
  TS_EFFECT         = 0x05,
  TS_RESOLUTION     = 0x06,
	TS_WARP						= 0x07,			// start warp effect
	TS_STRETCH				= 0x08,			// start stretch effect
	TS_CAMERA 				= 0x09,			// exit to camera stream
	TS_EDGE						= 0x0A,
	
} TS_CommandTypeDef;

/* Private define ------------------------------------------------------------*/
#define CAMERA_STATUS_TEXT_POS    30            /* Number of lines from bottom of screen */

#define CAMERA_VGA_RES_X          640
#define CAMERA_VGA_RES_Y          480
#define CAMERA_480x272_RES_X      480
#define CAMERA_480x272_RES_Y      272
#define CAMERA_QVGA_RES_X         320
#define CAMERA_QVGA_RES_Y         240
#define CAMERA_QQVGA_RES_X        160
#define CAMERA_QQVGA_RES_Y        120

#define CAMERA_RES_INDEX_MIN      CAMERA_R160x120
#define CAMERA_RES_INDEX_MAX      CAMERA_R640x480

#define CAMERA_CONTRAST_MIN       CAMERA_CONTRAST_LEVEL0
#define CAMERA_CONTRAST_MAX       CAMERA_CONTRAST_LEVEL4
#define CAMERA_BRIGHTNESS_MIN     CAMERA_BRIGHTNESS_LEVEL0
#define CAMERA_BRIGHTNESS_MAX     CAMERA_BRIGHTNESS_LEVEL4

#define cGEOMETRY		1
#define cWARP				2

#define R					3
#define G					2
#define B					1
/* Private macro -------------------------------------------------------------*/
#define	cTIME_MEASURE_1	0
#define	cTIME_MEASURE_2	1
#define	cTIME_MEASURE_3	2
#define	cTIME_MEASURE_4	3
#define	cTIME_MEASURE_5	4
#define cFFT_SIZE		4096
#define TIM_MEASURE_START {time_start = __HAL_TIM_GET_COUNTER(&TimHandle);}

#define TIM_MEASURE_END(x) {time_end = __HAL_TIM_GET_COUNTER(&TimHandle);     \
                        time_diff = time_end - time_start;								\
												tdiff[x]=(float)time_diff*(float)0.000000005;}
uint32_t time_start, time_end, time_diff;
float tdiff[5];
extern TIM_HandleTypeDef     TimHandle;
												
/* Private variables ---------------------------------------------------------*/
static uint32_t          CameraResX;
static uint32_t          CameraResY;
static uint32_t          CameraResIndex;
static volatile uint32_t CameraFrameBufferInitComplete = 0;
static volatile uint32_t CameraFrameBufferInitError = 0;
static volatile uint32_t Camera_AllowDma2dCopyCamFrmBuffToLcdFrmBuff = 0;
static uint32_t          offset_cam = 0;
static uint32_t          offset_lcd = 0;
static uint32_t          display_line_counter = 0;
static uint32_t          special_effect = CAMERA_BLACK_WHITE;        /* No special effect applied */
static uint32_t          color_effect = CAMERA_COLOR_EFFECT_BLUE;
static uint32_t          contrast = CAMERA_CONTRAST_LEVEL2;          /* Mid-level brightness */
static uint32_t          brightness = CAMERA_BRIGHTNESS_LEVEL2;      /* Mid-level contrast */
static DMA2D_HandleTypeDef hdma2d_camera;

/* Private function prototypes -----------------------------------------------*/
static void Camera_SetHint(int r);

static void ConvertCameraLineRgb565ToLcdLineARGB8888(void *pSrc,
                                                     void *pDst,
                                                     uint32_t xSize);

static void Camera_FrameBuffer_Init_TransferComplete(DMA2D_HandleTypeDef *hdma2d);

static void Camera_FrameBuffer_Init_TransferError(DMA2D_HandleTypeDef *hdma2d);

static uint32_t CameraFrameBufferRgb565_Init(uint32_t sizeX,
                                             uint32_t sizeY,
                                             uint32_t argb8888_Value);

void BSP_CAMERA_LineEventCallback(void);
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef *hdma2d);
int AllocaMemoria( void);
void ShowStaticImage( void);
void DistanceCalc_asm( short*pTchX, short*pTchY, float*pDstn, unsigned int size);
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point );
int Camera_Init( void);
void VisualizzaRisultato( unsigned char* image);
void VisualizzaRisultatoLabeling( uint8_t*image);
void VisualizzaFrame( void);
void VisualizzaRiquadro( uint8_t*image1, uint8_t*image2 );
void copiaFrameNeiBuffer( void);
void swapBuffer( volatile uint32_t**pa, volatile uint32_t**pb);
/* Private functions ---------------------------------------------------------*/

#ifndef USE_FULL_ASSERT
uint32_t    ErrorCounter = 0;
#endif

/* *****/
TS_StateTypeDef  TS_State;
uint32_t TS_command=TS_NO_COMMAND;
/* */
float Dst[6];
float DstOld[6];

/* */
uint8_t*the_image;
uint8_t*diff_image;
uint8_t*bg_image;
uint8_t*fg_image;
//
uint32_t*src_image;
uint32_t*dst_image;
//
uint32_t*img_centro;
uint32_t*img_dxup;
uint32_t*img_dxdwn;
//
uint8_t*ARGB_image;
uint32_t *ptr;
uint32_t img;	
/* */
uint32_t oldx, oldy;

uint32_t r, t, i, j;
int32_t tmp;

uint8_t thresh;
uint32_t frameCount;
unsigned char p;

/**
  * @brief  Camera demo
  * @param  None
  * @retval None
  */
void Camera_demo (void)
{
  uint8_t  status = 0;
//  uint32_t exit = 0;
  uint32_t camera_status = CAMERA_OK;

	/* per adesso non uso la camera... */
  //Camera_SetHint( 1);
  
	//
  status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  if (status != TS_OK)
  {
		return;
  }
	
	//
  ///*## Camera Initialization and start capture ############################*/
  //
	//status = Camera_Init();
	//if ( status == CAMERA_ERROR) {
	//	return;
	//}

	status = AllocaMemoria();
	if ( status) {
		return;
	}
	
	// assegno ai puntatori di lavoro, gli indirizzi dei due buffer
	src_image=(uint32_t*)&the_image[0];
	dst_image=(uint32_t*)&diff_image[0];
	
	// configurazione iniziale con il foreground nella finestra grande e 
	// nelle due piccole a dx il frame originale e il frame background
	img_centro = (uint32_t*)&fg_image[0];
	img_dxup   = (uint32_t*)&the_image[0];
	img_dxdwn  = (uint32_t*)&bg_image[0];
	
	if(FATFS_LinkDriver(&SD_Driver, SDPath) != 0)
	{
		while(1);
	}
	
	if(f_mount(&SDFatFs, (TCHAR const*)SDPath, 0) != FR_OK)
	{
		while(1);
	}
		
	// I file sono stati salvati ARGB in sequenza di immagini ( 320x240xARGB )
	if(f_open(&MyFile, "sanf32.bin", FA_READ) != FR_OK)
	{
		while(1);
	}

	res = f_read(&MyFile, ARGB_image, (ROWS*COLS*4), (UINT*)&bytesread);
	frameCount=1;
	// sposto il frame letto da SDCard nei vari buffer di lavoro...
	copiaFrameNeiBuffer();			
	// salvo il primo frame come frame di riferimento per il background
	copy_result( bg_image, the_image);
	thresh=25;
	
	res = f_read(&MyFile, ARGB_image, (ROWS*COLS*4), (UINT*)&bytesread);
	frameCount++;

	// loop lettura frame da SDCard...
	while( res == FR_OK && bytesread != 0)
	{
		// VisualizzaFrame();
		copiaFrameNeiBuffer();			// sposto il frame letto da SDCard nei vari buffer di lavoro...
				
		TIM_MEASURE_START;
		
		// loop unico di calcolo...
		for(i=0; i<(ROWS*COLS); i++) {
			// calcolo il frame differenza
			tmp = (int)((int)the_image[i] - (int)bg_image[i]);
			tmp = abs( tmp);
			//if (tmp>255) {
			//	tmp=(uint8_t)255;
			//}
			
			diff_image[i] = (uint8_t)tmp;		// out_image contiene l'immagine differenza fr_diff
			// verifico se il valore differenza è maggiore del threshold 
			if ( diff_image[i] > (uint8_t)thresh) {
				fg_image[i] = the_image[i];					// fg_image contiene il foreground
			} /*else {
				fg_image[i] = 0;
			}*/
			
			// modifico il background in funzione del nuovo frame
			if ( the_image[i] > bg_image[i]) {
				bg_image[i] = bg_image[i] + 1;
			} else {
				if ( the_image[i] < bg_image[i]) {
					bg_image[i] = bg_image[i] - 1;
				}
			}
		}
		TIM_MEASURE_END(cTIME_MEASURE_1);
		
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected == 1 ) {
			if ( CheckRectangle(320,0,160,120,TS_State.touchX[0],TS_State.touchY[0]) ) {
				swapBuffer( &img_centro, &img_dxup);
			}
			if ( CheckRectangle(320,120,160,120,TS_State.touchX[0],TS_State.touchY[0]) ) {
				swapBuffer( &img_centro, &img_dxdwn);
			}
		}

		while( TS_State.touchDetected >= 1)
			BSP_TS_GetState(&TS_State);
		
		VisualizzaRisultato( (uint8_t*)img_centro);				// visualizzo la differenza tra le due immagini.
		VisualizzaRiquadro( (uint8_t*)img_dxup, (uint8_t*)img_dxdwn);
		
		HAL_Delay(1);
		
		// leggo il frame successivo...
		TIM_MEASURE_START;
		res = f_read(&MyFile, ARGB_image, (ROWS*COLS*4), (UINT*)&bytesread);
		TIM_MEASURE_END(cTIME_MEASURE_2);
		
		memset(fg_image, 0, sizeof(uint8_t) * ROWS * COLS);
		frameCount++;
	}
									
	while(1);
	
}

void swapBuffer( volatile uint32_t**pa, volatile uint32_t**pb)
{
	volatile uint32_t* temp = *pa;
    *pa = *pb;
    *pb = temp;
}

void LabelComponent(unsigned short* STACK, unsigned short width, unsigned short height, 
										unsigned char* input, unsigned char* output, int labelNo, unsigned short x, unsigned short y)
{
  int mSP  = 3;
  int index;

  STACK[0] = x;
  STACK[1] = y;
  STACK[2] = 0;  /* return - component is labelled */
	
START: /* Recursive routine starts here */

  index = X + width*Y;
  if (input [index] == 0) RETURN;   /* This pixel is not part of a component */
  if (output[index] != 0) RETURN;   /* This pixel has already been labelled  */
	if (labelNo>255)
		while(1);
  output[index] = labelNo;

  if (X > 0) CALL_LabelComponent(X-1, Y, 1);   /* left  pixel */
RETURN1:

  if (X < width-1) CALL_LabelComponent(X+1, Y, 2);   /* rigth pixel */
RETURN2:

  if (Y > 0) CALL_LabelComponent(X, Y-1, 3);   /* upper pixel */
RETURN3:

  if (Y < height-1) CALL_LabelComponent(X, Y+1, 4);   /* lower pixel */
RETURN4:

  RETURN;
}


void LabelImage(unsigned short width, unsigned short height, unsigned char* input, unsigned char* output)
{
  unsigned short* STACK = (unsigned short*) malloc(3*sizeof(unsigned short)*(width*height + 1));

  int labelNo = 0;
  int index   = -1;
  unsigned short y;
  unsigned short x;

  for ( y = 0; y < height; y++)
  {
    for ( x = 0; x < width; x++)
    {
      index++;
      if (input [index] == 0) continue;   /* This pixel is not part of a component */
      if (output[index] != 0) continue;   /* This pixel has already been labelled  */
      /* New component found */
      labelNo++;
      LabelComponent(STACK, width, height, input, output, labelNo, x, y);
    }
  }

  free(STACK);
}


int Camera_Init( void)
{
	uint32_t argb8888_Value = 0x00FF00FF; /* = 0xF81F in RGB565 format */
	uint32_t camera_status = CAMERA_OK;
	
	CameraResIndex = CAMERA_R320x240;     /* Set QVGA default resolution */
	CameraResX     = CAMERA_QVGA_RES_X;
	CameraResY     = CAMERA_QVGA_RES_Y;
	
	/* Initialize the Camera */
	camera_status = BSP_CAMERA_Init(RESOLUTION_R320x240);
	if (camera_status != CAMERA_OK) {
		return CAMERA_ERROR;
	}
	
	offset_cam = 0;
	offset_lcd = 0;
	display_line_counter = 0;

	special_effect = CAMERA_BLACK_WHITE;      /* No special effect applied */
	color_effect   = CAMERA_BLACK_WHITE_NORMAL;
	contrast       = CAMERA_CONTRAST_LEVEL2;        /* Mid-level brightness */
	brightness     = CAMERA_BRIGHTNESS_LEVEL2;    /* Mid-level contrast */

	CameraFrameBufferInitComplete = 0;
	CameraFrameBufferInitError    = 0;

	/* Init or Re-Init Camera frame buffer by using DMA2D engine in mode Register to Memory */
	camera_status = CameraFrameBufferRgb565_Init(CAMERA_VGA_RES_X,
												 CAMERA_VGA_RES_Y,
												 argb8888_Value);
	ASSERT(camera_status != CAMERA_OK);

	/* Wait end of DMA2D operation of error : via IT callback update */
	while((CameraFrameBufferInitComplete == 0) && (CameraFrameBufferInitError == 0)) {;}

	/* Assert if error : no transfer complete */
	ASSERT(CameraFrameBufferInitComplete != 1);

	/* Start / Restart camera stream */
	BSP_CAMERA_ContinuousStart((uint8_t *)CAMERA_FRAME_BUFFER);

	/* Allow DMA2D copy from Camera frame buffer to LCD Frame buffer location */
	Camera_AllowDma2dCopyCamFrmBuffToLcdFrmBuff = 1;
	
	return CAMERA_OK;
}

/**
  * @brief  Display Camera demo hint
  * @param  None
  * @retval None
  */
static void Camera_SetHint(int r)
{
	if (r) {
		/* Clear the LCD */
		BSP_LCD_Clear(LCD_COLOR_WHITE);
	}
  /* Set Camera Demo description */
  BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
  
	if ( TS_command==TS_WARP) {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);		
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	}
	BSP_LCD_DisplayStringAt(330,  10, (uint8_t *)  " WARP   ", LEFT_MODE);

	if ( TS_command==TS_STRETCH) {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);		
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	}	
  BSP_LCD_DisplayStringAt(330,  50, (uint8_t *)  " STRATCH", LEFT_MODE);

	if ( TS_command==TS_EDGE) {
		BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
		BSP_LCD_SetBackColor(LCD_COLOR_BLUE);		
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	}	
  BSP_LCD_DisplayStringAt(330,  90, (uint8_t *)  " EDGE   ", LEFT_MODE);

	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);	
	if ( TS_command!=TS_CAMERA) {
		BSP_LCD_DisplayStringAt(330, 220, (uint8_t *)" CAMERA  ", LEFT_MODE);
	} else {
		BSP_LCD_DisplayStringAt(330, 220, (uint8_t *)"         ", CENTER_MODE);
	}
  /* Set the LCD Text Color */
  //BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
  //BSP_LCD_DrawRect(10, 100, BSP_LCD_GetXSize() - 20, BSP_LCD_GetYSize() - 110);
  //BSP_LCD_DrawRect(11, 101, BSP_LCD_GetXSize() - 22, BSP_LCD_GetYSize() - 112);
}

/**
  * @brief  DMA2D Camera frame buffer init Transfer completed callback
  * @param  hdma2d: DMA2D handle.
  * @retval None
  */
static void Camera_FrameBuffer_Init_TransferComplete(DMA2D_HandleTypeDef *hdma2d)
{
 CameraFrameBufferInitComplete = 1;
}

/**
  * @brief  DMA2D Camera frame buffer init Transfer error callback
  * @param  hdma2d: DMA2D handle.
  * @retval None
  */
static void Camera_FrameBuffer_Init_TransferError(DMA2D_HandleTypeDef *hdma2d)
{
 CameraFrameBufferInitError = 1;
}


/**
  * @brief  Init camera frame buffer with fixed color in format RGB565
  *         to a LCD display frame buffer line in format ARGB8888 using DMA2D service.
  * @param  sizeX: Size in X of rectangular region of the Camera frame buffer to initialize (in pixels unit)
  * @param  sizeX: Size in X of rectangular region of the Camera frame buffer to initialize (in pixels unit)
  * @param  argb_Value : Initialization value (pattern ARGB8888) to be applied to all rectangular region selected.
  * @retval Status CAMERA_OK or CAMERA_ERROR
  */
static uint32_t CameraFrameBufferRgb565_Init(uint32_t sizeX, uint32_t sizeY, uint32_t argb8888_Value)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  uint32_t status = CAMERA_ERROR;

  if((sizeX <= CAMERA_VGA_RES_X) && (sizeY <= CAMERA_VGA_RES_Y))
  {
    /* Register to memory mode with RGB565 as colorMode */
    hdma2d_camera.Init.Mode         = DMA2D_R2M; /* Mode Register to Memory */
    hdma2d_camera.Init.ColorMode    = DMA2D_RGB565; /* Output color mode */
    hdma2d_camera.Init.OutputOffset = 0x0; /* No offset in output */

    /* Set callback functions on transfer complete and transfer error */
    hdma2d_camera.XferCpltCallback  = Camera_FrameBuffer_Init_TransferComplete;
    hdma2d_camera.XferErrorCallback = Camera_FrameBuffer_Init_TransferError;

    hdma2d_camera.Instance = DMA2D;

    hal_status = HAL_DMA2D_Init(&hdma2d_camera);
    ASSERT(hal_status != HAL_OK);

    hal_status = HAL_DMA2D_Start_IT(&hdma2d_camera,
                                    argb8888_Value, /* Color value in Register to Memory DMA2D mode */
                                    (uint32_t)CAMERA_FRAME_BUFFER,  /* DMA2D output buffer */
                                    sizeX,  /* width of buffer in pixels */
                                    sizeY); /* height of buffer in lines */
    ASSERT(hal_status != HAL_OK);

    status = CAMERA_OK;
  }

  return (status);
}

/**
  * @brief  Converts a camera buffer line of format RGB565
  *         to a LCD display frame buffer line in format ARGB8888 using DMA2D service.
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Pointer to destination buffer
  * @param  xSize: Buffer width
  * @retval None
  */
static void ConvertCameraLineRgb565ToLcdLineARGB8888(void *pSrc, void *pDst, uint32_t xSize)
{
  HAL_StatusTypeDef hal_status = HAL_OK;
  
  /* Configure the DMA2D Mode, Color Mode and output offset */
  hdma2d_camera.Init.Mode         = DMA2D_M2M_PFC;
  hdma2d_camera.Init.ColorMode    = DMA2D_ARGB8888;
  hdma2d_camera.Init.OutputOffset = 0;

  /* Foreground Configuration */
  hdma2d_camera.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d_camera.LayerCfg[1].InputAlpha = 0xFF;
  hdma2d_camera.LayerCfg[1].InputColorMode = DMA2D_RGB565;
  hdma2d_camera.LayerCfg[1].InputOffset = 0;

  hdma2d_camera.Instance = DMA2D;

  /* DMA2D Initialization */
  if (HAL_DMA2D_Init(&hdma2d_camera) == HAL_OK)
  {
    if (HAL_DMA2D_ConfigLayer(&hdma2d_camera, 1) == HAL_OK)
    {
      if (HAL_DMA2D_Start(&hdma2d_camera, (uint32_t)pSrc, (uint32_t)pDst, xSize, 1) == HAL_OK)
      {
        /* Polling For DMA transfer */
        hal_status = HAL_DMA2D_PollForTransfer(&hdma2d_camera, 10);
        ASSERT(hal_status != HAL_OK);
      }
    }
  }
}

/**
  * @brief  Camera line event callback
  * @param  None
  * @retval None
  */
void BSP_CAMERA_LineEventCallback(void)
{
  uint32_t LcdResX = BSP_LCD_GetXSize();
  uint32_t LcdResY = BSP_LCD_GetYSize();

  if (Camera_AllowDma2dCopyCamFrmBuffToLcdFrmBuff)
  {
		// Commento questa parte perché voglio che le immagini dalla cam siano
		// posizionate in alto a sx.
		//
    //if ((offset_lcd == 0) && (CameraResX < LcdResX) && (CameraResY < LcdResY))
    //{
    //  /* If Camera resolution is lower than LCD resolution, set display in the middle of the screen */
    //  offset_lcd =   ((((LcdResY - CameraResY) / 2) * LcdResX)   /* Middle of the screen on Y axis */
    //                  +   ((LcdResX - CameraResX) / 2))             /* Middle of the screen on X axis */
    //                 * sizeof(uint32_t);
    //
    //  if (CameraResY == CAMERA_QQVGA_RES_Y)
    //  { /* Add offset for QQVGA */
    //    offset_lcd += 40 * LcdResX * sizeof(uint32_t);
    //  }
    //}

    if (display_line_counter < CameraResY)
    {
      if (display_line_counter < LcdResY)
      {
        if (CameraResX < LcdResX)
        {
          ConvertCameraLineRgb565ToLcdLineARGB8888((uint32_t *)(CAMERA_FRAME_BUFFER + offset_cam),
                                                   (uint32_t *)(LCD_FB_START_ADDRESS + offset_lcd),
                                                   CameraResX);
        }
        else
        {
          ConvertCameraLineRgb565ToLcdLineARGB8888((uint32_t *)(CAMERA_FRAME_BUFFER + offset_cam),
                                                   (uint32_t *)(LCD_FB_START_ADDRESS + offset_lcd),
                                                   LcdResX);
        }

        offset_cam  = offset_cam + (CameraResX * sizeof(uint16_t));
        offset_lcd  = offset_lcd + (LcdResX * sizeof(uint32_t));
      }
      display_line_counter++;
    }
    else
    {
      offset_cam = 0;
      offset_lcd = 0;
      display_line_counter = 0;
    }
  }
}

/**
  * @brief  Handles DMA2D interrupt request.
  * @param  None
  * @retval None
  */
void BSP_LCD_DMA2D_IRQHandler(void)
{
  HAL_DMA2D_IRQHandler(&hdma2d_camera);
}

/**
  * @brief DMA2D MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  *           - Peripheral's GPIO Configuration
  * @param hdma2d: DMA2D handle pointer
  * @retval None
  */
void HAL_DMA2D_MspInit(DMA2D_HandleTypeDef *hdma2d)
{
  /*##-1- Enable peripherals and GPIO Clocks #################################*/
  __HAL_RCC_DMA2D_CLK_ENABLE();

  /*##-2- NVIC configuration  ################################################*/
  /* NVIC configuration for DMA2D transfer complete interrupt */
  HAL_NVIC_SetPriority(DMA2D_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2D_IRQn);
}


/*
 * Questa funzione alloca i buffer per contenere i canali RGB, il buffer di uscita dell'elaborazione
 * ed il buffer con l'immagine sorgente.
 * A seguire, riempie con i dati dell'immagine campione i buffer.
 */
int AllocaMemoria( void)
{

	/* alloco lo spazio per il buffer che conterrà l'immagine */
	the_image=(unsigned char*)malloc(sizeof(uint8_t) * ROWS * COLS);
	if ( the_image==(unsigned char*)NULL)
		return 1;
	memset(the_image, 0, sizeof(uint8_t) * ROWS * COLS);
	
	/* alloco lo spazio per il buffer che conterrà il risultato */
	diff_image=(unsigned char*)malloc(sizeof(uint8_t) * ROWS * COLS);
	if ( diff_image==(unsigned char*)NULL)
		return 1;
	memset(diff_image, 0, sizeof(uint8_t) * ROWS * COLS);
		
	/* alloco lo spazio per il buffer bg_image */
	bg_image=(unsigned char*)malloc(sizeof(uint8_t) * ROWS * COLS);
	if ( bg_image==(unsigned char*)NULL)
		return 1;
	memset(bg_image, 0, sizeof(uint8_t) * ROWS * COLS);
	
	fg_image=(unsigned char*)malloc(sizeof(uint8_t) * ROWS * COLS);
	if ( fg_image==(unsigned char*)NULL)
		return 1;
	memset(fg_image, 0, sizeof(uint8_t) * ROWS * COLS);
	
	/* Alloco lo spazio per il buffer immagine da SDCard. Il file è scritto
	   con i byte in reverse, invece che ARGB è BGRA, perché per la visualizzazione diretta
		 uso la lettura a word. (vedi VisualizzaRisultatoImage)
	*/
	ARGB_image=(unsigned char*)malloc(ROWS * COLS * 4);
	if ( ARGB_image==(unsigned char*)NULL)
		return 1;
	memset(ARGB_image, 0, (ROWS * COLS * 4) );
		
	return 0; 
}

/* Questa funzione inizializza gli array di lavoro.
   In the_image copio l'immagine in grayscale, a seguire nei relativi
	 buffer copio i tre canali colore RGB.
*/
void copiaFrameNeiBuffer( void)
{
	int i, j;
	
	// Copio i dati letti da SDCard nei buffer di lavoro
	j=0;
	for (i=0; i<(ROWS*COLS);i++) {
		// il formato del file immagine dalla SDCard è: BGRA
		the_image[i]=((ARGB_image[j+R]*0.2126f)+(ARGB_image[j+G]*0.7152f)+(ARGB_image[j+B]*0.0722f));
		j+=4;
	}	

}

void VisualizzaRisultato( unsigned char* image)
{
	int j, i, t, q;
	
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			t=(i*BSP_LCD_GetXSize())+j;
			q=(i*COLS)+j;
			ptr[t]=0xFF000000 | (image[q]<<16) | (image[q]<<8) | image[q];
		}
	}
}

void VisualizzaRiquadro( uint8_t*image1, uint8_t*image2 )
{
	int j, i, t1, t2, q;
	
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<(ROWS);i+=2) {
		for ( j=0; j<(COLS); j+=2) {
			t1=((i/2)*BSP_LCD_GetXSize())+(j/2)+320;
			t2=(((i/2)+120)*BSP_LCD_GetXSize())+(j/2)+320;
			q=(i*(COLS))+j;
			ptr[t1]=0xFF000000 | (image1[q]<<16) | (image1[q]<<8) | image1[q];
			ptr[t2]=0xFF000000 | (image2[q]<<16) | (image2[q]<<8) | image2[q];
		}
	}
}

void VisualizzaRisultatoLabeling( uint8_t*image)
{
	int j, i, t, q, r;
	
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			t=(i*BSP_LCD_GetXSize())+j;
			q=(i*COLS)+j;
			r=image[q];
			//if (r)
			//	r--;
			ptr[t]=0xFF000000 | cp[r];
		}
	}
}

/* Funzione per visualizzare l'immagine da SDCard sullo schermo.
	 Nella SDCard i file immagine sono memorizzati con i canali colore in reverse.
	 Invece che ARGB sono BGRA. In questa maniera per la visualizzazione ho usato 
	 la copia della word.
*/
void VisualizzaFrame( void)
{
	int j, i, t;
	uint32_t*tmp;
	
	// per usare questo codice ho dovuto memorizzare il file delle immagini con i byte ARGB 
	// al contrario: BGRA. 
	tmp=(uint32_t*)ARGB_image;								// cast del puntatore all'immagine a word
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
		
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			t=(i*BSP_LCD_GetXSize())+j;
			ptr[t]=*tmp++;												// incremento il puntatore al buffer immagine. Si sposterà
																						// di una word passando al successivo blocco BGRA.
		}
	}
}

/*************************************************************************
 * Function Name: CheckRectangle
 * Parameters: Int32U X_up, Int32U Y_up - rectangle coordinate
 *             Int32U X_size, Int32U Y_size - rectangle size
 *             Int32U X_poin, Int32U Y_point - point coordinate
 *
 * Return: Boolean
 *    TRUE  - the point is inside  from the rectangle
 *    FALSE - the point is outside from the rectangle
 *
 * Description: Check whether the coordinate of point is inside from a rectangle
 *
 *************************************************************************/
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point )
{
  if((X_up <= X_point) && (X_point <= X_up+X_size) &&
     (Y_up <= Y_point) && (Y_point <= Y_up+Y_size))
  {
    return (1);
  }
  return (0);
}

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
