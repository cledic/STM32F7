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
/* Private macro -------------------------------------------------------------*/

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
void DoWarp( void);
void DoStretch( void);
void DistanceCalc_asm( short*pTchX, short*pTchY, float*pDstn, unsigned int size);
void ColoraImmagineWarp( void);
void ColoraImmagineGeom( void);
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point );
int Camera_Init( void);
void CopySnapShot( int r);
void VisualizzaRisultato( void);
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

/* Imager */
GEOMDATA gd;
ROTDATA rd;
WARPDATA wd;
/* */
unsigned char*the_image;
unsigned char*out_image;
unsigned char*r_image;
unsigned char*g_image;
unsigned char*b_image;
uint32_t *ptr;
uint32_t img;	
/* */
float stretch_inc=0.05;
uint32_t oldx, oldy;

/**
  * @brief  Camera demo
  * @param  None
  * @retval None
  */
void Camera_demo (void)
{
  uint8_t  status = 0;
  uint32_t exit = 0;
  uint32_t camera_status = CAMERA_OK;
//  uint32_t argb8888_Value = 0x00FF00FF; /* = 0xF81F in RGB565 format */
  //TS_StateTypeDef  TS_State;

  Camera_SetHint( 1);

  status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
  if (status != TS_OK)
  {
		return;
  }
	
  /*## Camera Initialization and start capture ############################*/

	status = Camera_Init();
	if ( status == CAMERA_ERROR) {
		return;
	}
	
	status = AllocaMemoria();
	if ( status) {
		return;
	}
	
  /* Infinite loop */
  while (1)
  {
		//
		/* Check in polling mode in touch screen the touch status and coordinates */
		/* if touch occurred                                                      */
		BSP_TS_GetState(&TS_State);
		if (TS_State.touchDetected == 1)
		{
			// Controllo se il tap è su WARP
			if ( CheckRectangle( 330, 10, 150, 30, TS_State.touchX[0], TS_State.touchY[0]) ) {
				TS_command=TS_WARP;
				camera_status = BSP_CAMERA_Stop();
			  /* Insert 10 ms delay */
				HAL_Delay(10);
				//
				CopySnapShot( 1);		
				oldx=oldy=0;
				/* Wait for touch screen no touch detected */
				do
				{
					BSP_TS_GetState(&TS_State);
				}while(TS_State.touchDetected > 0);
			}
			// Controllo se il tap è su Stretch
			if ( CheckRectangle( 330, 50, 150, 30, TS_State.touchX[0], TS_State.touchY[0]) ) {
				TS_command=TS_STRETCH;
				camera_status = BSP_CAMERA_Stop();
			  /* Insert 10 ms delay */
				HAL_Delay(10);				
				//
				CopySnapShot( 1);
				gd.x_stretch=1.0f; gd.y_stretch=1.0f; 
				/* Wait for touch screen no touch detected */
				do
				{
					BSP_TS_GetState(&TS_State);
				}while(TS_State.touchDetected > 0);
			}
			// Controllo se il tap è su Edge
			if ( CheckRectangle( 330, 90, 150, 30, TS_State.touchX[0], TS_State.touchY[0]) ) {
				TS_command=TS_EDGE;
				camera_status = BSP_CAMERA_Stop();
			  /* Insert 10 ms delay */
				HAL_Delay(10);				
				//
				CopySnapShot( 1);
				/* Wait for touch screen no touch detected */
				do
				{
					BSP_TS_GetState(&TS_State);
				}while(TS_State.touchDetected > 0);
			}
			
			// Controllo se il tap è Camera
			if ( CheckRectangle( 330, 220, 150, 30, TS_State.touchX[0], TS_State.touchY[0]) ) {
				if ( TS_command != TS_CAMERA) {
					TS_command=TS_CAMERA;
					Camera_Init();
					HAL_Delay(10);
					oldx=oldy=0;
					gd.x_stretch=1.0f; gd.y_stretch=1.0f; 
				}
			}
			// Visualizzo nuovamente il menu cambiato a seconda della selezione...
			Camera_SetHint( 0);				
		} // 
		
		switch( TS_command) {
			case TS_WARP:
				DoWarp();
				//ColoraImmagineWarp();
				break;
			case TS_STRETCH:
				DoStretch();
				//ColoraImmagineGeom();
				break;
			case TS_EDGE:
				gaussian_edge( the_image, out_image, 9, 0, 1);
				VisualizzaRisultato();
				TS_command=TS_NO_COMMAND;
				break;
		}
  }

  //if (camera_status == CAMERA_OK)
  //{
  //  /* Stop camera stream */
  //  camera_status = BSP_CAMERA_Stop();
  //  ASSERT(camera_status != CAMERA_OK);
  //}
  
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

void DoWarp( void)
{
	int i, j;
	int p=0;
	
	while(1) {
		BSP_TS_GetState(&TS_State);
		if(TS_State.touchDetected == 1 && CheckRectangle(0,0,320,240,TS_State.touchX[0],TS_State.touchY[0]) ) {
			//
			p=1;
			oldx=TS_State.touchX[0];
			oldy=TS_State.touchY[0];
			//
			wd.the_image=the_image; 
			wd.out_image=out_image;
			wd.x_control=TS_State.touchX[0]; wd.y_control=TS_State.touchY[0];
			wd.bilinear=1;
			wd.rows=ROWS; wd.cols=COLS;
			//
			warp2( &wd);
						
			ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
			for (i=0; i<ROWS;i++) {
				for ( j=0; j<COLS; j++) {
					img=0xFF000000 | (out_image[(i*COLS)+j]<<16) | (out_image[(i*COLS)+j]<<8) | out_image[(i*COLS)+j];
					ptr[(i*BSP_LCD_GetXSize())+j]=img;
				}
			}
		} else {
			if ( p) {
				p=0;
				ColoraImmagineWarp();
			}
			break;
		}
	}
}

void DoStretch( void)
{
	int i, j;
	int p=0;
	
		while(1) {
			BSP_TS_GetState(&TS_State);
			if(TS_State.touchDetected == 2 && CheckRectangle(0,0,320,240,TS_State.touchX[0],TS_State.touchY[0]) ) {
				p=1;
				DistanceCalc_asm( &TS_State.touchX[0], &TS_State.touchY[0], &Dst[0], TS_State.touchDetected);

				if ( Dst[0] > DstOld[0] && (Dst[0]-DstOld[0])>5.0f) {
					gd.x_stretch+=stretch_inc; gd.y_stretch+=stretch_inc;
					DstOld[0]=Dst[0];
				} else if ( Dst[0] < DstOld[0] && (DstOld[0]-Dst[0])>5.0f) {
					gd.x_stretch-=stretch_inc; gd.y_stretch-=stretch_inc;
					DstOld[0]=Dst[0];
				}

				gd.the_image=the_image;
				gd.out_image=out_image;
				gd.x_angle=0.0;
				//gd.x_stretch=1.01f; gd.y_stretch=1.01f;
				gd.x_displace=0; gd.y_displace=0;
				gd.x_cross=0; gd.y_cross=0;
				gd.bilinear=1;
				gd.cols=COLS; gd.rows=ROWS;
				//
				geometry2( &gd);

				ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
				for (i=0; i<ROWS;i++) {
					for ( j=0; j<COLS; j++) {
						img=0xFF000000 | (out_image[(i*COLS)+j]<<16) | (out_image[(i*COLS)+j]<<8) | out_image[(i*COLS)+j];
						ptr[(i*BSP_LCD_GetXSize())+j]=img;
					}
				}
		} else {
			if ( p) {
				p=0;
				ColoraImmagineGeom();
			}
			break;
		}
	}			
}

/*
 * Questa funzione alloca i buffer per contenere i canali RGB, il buffer di uscita dell'elaborazione
 * ed il buffer con l'immagine sorgente.
 * A seguire, riempie con i dati dell'immagine campione i buffer.
 */
int AllocaMemoria( void)
{
	int i, j;
	
	/* alloco lo spazio per il buffer che conterrà l'immagine */
	the_image=(unsigned char*)malloc(ROWS*COLS);
	if ( the_image==(unsigned char*)NULL)
		return 1;
	
	/* alloco lo spazio per il buffer che conterrà il risultato */
	out_image=(unsigned char*)malloc(ROWS*COLS);
	if ( out_image==(unsigned char*)NULL)
		return 1;

	/* alloco lo spazio per il buffer R */
	r_image=(unsigned char*)malloc(ROWS*COLS);
	if ( r_image==(unsigned char*)NULL)
		return 1;
	/* alloco lo spazio per il buffer G */
	g_image=(unsigned char*)malloc(ROWS*COLS);
	if ( g_image==(unsigned char*)NULL)
		return 1;
	/* alloco lo spazio per il buffer B */
	b_image=(unsigned char*)malloc(ROWS*COLS);
	if ( b_image==(unsigned char*)NULL)
		return 1;
	
	gd.x_stretch=1.0f; gd.y_stretch=1.0f;
	
	// I dati da copiare li prendo dal framebuffer quando l'utente decide 
	// lo snapshot.
	// Copio i dati nei buffer di lavoro
	//j=0;
	//for (i=0; i<(ROWS*COLS);i++) {
	//	the_image[i]=((image_320x240[j]*0.2126f)+(image_320x240[j+1]*0.7152f)+(image_320x240[j+2]*0.0722f));
	//	r_image[i]=image_320x240[j];
	//	g_image[i]=image_320x240[j+1];
	//	b_image[i]=image_320x240[j+2];
	//	j+=3;
	//}	
	//
	return 0; 
}

void VisualizzaRisultato( void)
{
	int j, i, t, q;
	
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			t=(i*BSP_LCD_GetXSize())+j;
			q=(i*COLS)+j;
			ptr[t]=0xFF000000 | (out_image[q]<<16) | (out_image[q]<<8) | out_image[q];
			//img=ptr[t];
			//r_image[q]=(img&0x00FF0000)>>16;
			//g_image[q]=(img&0x0000FF00)>>8;
			//b_image[q]=img&0x000000FF;
			//the_image[q]=((r_image[q]*0.2126f)+(g_image[q]*0.7152f)+(b_image[q]*0.0722f));
			//if ( r)
			//	ptr[t]=0xFF000000 | (the_image[q]<<16) | (the_image[q]<<8) | the_image[q];
		}
	}
}

void CopySnapShot( int r)
{
	int j, i, t, q;
	
	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			t=(i*BSP_LCD_GetXSize())+j;
			q=(i*COLS)+j;
			img=ptr[t];
			r_image[q]=(img&0x00FF0000)>>16;
			g_image[q]=(img&0x0000FF00)>>8;
			b_image[q]=img&0x000000FF;
			the_image[q]=((r_image[q]*0.2126f)+(g_image[q]*0.7152f)+(b_image[q]*0.0722f));
			if ( r)
				ptr[t]=0xFF000000 | (the_image[q]<<16) | (the_image[q]<<8) | the_image[q];
		}
	}
}

void ColoraImmagineWarp( void)
{
	int i, j;
	
	wd.the_image=r_image; 
	wd.out_image=out_image;
	wd.x_control=oldx; wd.y_control=oldy;
	wd.bilinear=1;
	wd.rows=ROWS; wd.cols=COLS;
	//
	warp2( &wd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFF00FFFF;
			img|=(out_image[(i*COLS)+j]<<16);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
		}
	}

	wd.the_image=g_image; 
	wd.out_image=out_image;
	wd.x_control=oldx; wd.y_control=oldy;
	wd.bilinear=1;
	wd.rows=ROWS; wd.cols=COLS;
	//
	warp2( &wd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFFFF00FF;
			img|=(out_image[(i*COLS)+j]<<8);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
		}
	}

	wd.the_image=b_image; 
	wd.out_image=out_image;
	wd.x_control=oldx; wd.y_control=oldy;
	wd.bilinear=1;
	wd.rows=ROWS; wd.cols=COLS;
	//
	warp2( &wd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFFFFFF00;
			img|=(out_image[(i*COLS)+j]);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
		}
	}					
}

void ColoraImmagineGeom( void)
{
	int i, j;
	
	gd.the_image=r_image;
	gd.out_image=out_image;
	gd.x_angle=0.0;
	gd.x_displace=0; gd.y_displace=0;
	gd.x_cross=0; gd.y_cross=0;
	gd.bilinear=1;
	gd.cols=COLS; gd.rows=ROWS;
	//
	geometry2( &gd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFF00FFFF;
			img|=(out_image[(i*COLS)+j]<<16);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
		}
	}

	gd.the_image=g_image;
	gd.out_image=out_image;
	gd.x_angle=0.0;
	gd.x_displace=0; gd.y_displace=0;
	gd.x_cross=0; gd.y_cross=0;
	gd.bilinear=1;
	gd.cols=COLS; gd.rows=ROWS;
	//
	geometry2( &gd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFFFF00FF;
			img|=(out_image[(i*COLS)+j]<<8);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
		}
	}

	gd.the_image=b_image;
	gd.out_image=out_image;
	gd.x_angle=0.0;
	gd.x_displace=0; gd.y_displace=0;
	gd.x_cross=0; gd.y_cross=0;
	gd.bilinear=1;
	gd.cols=COLS; gd.rows=ROWS;
	//
	geometry2( &gd);

	ptr = (uint32_t*)(LCD_FB_START_ADDRESS);
	for (i=0; i<ROWS;i++) {
		for ( j=0; j<COLS; j++) {
			img=ptr[(i*BSP_LCD_GetXSize())+j]&0xFFFFFF00;
			img|=(out_image[(i*COLS)+j]);
			ptr[(i*BSP_LCD_GetXSize())+j]=img;
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
