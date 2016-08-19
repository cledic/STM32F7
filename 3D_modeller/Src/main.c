/**
  ******************************************************************************
  * @file    FatFs/FatFs_uSD/Src/main.c
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   This sample code shows how to use FatFs with uSD card drive.
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
#include "stdlib.h"
#include "arm_math.h"
#include "arm_const_structs.h"

#include "3d.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
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

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef     TimHandle;
uint32_t time_start, time_end, time_diff;
float tdiff[5];
/* ************************************************************************** */
FATFS SDFatFs;  /* File system object for SD card logical drive */
FIL MyFile;     /* File object */
char SDPath[4]; /* SD card logical drive path */
uint32_t i=0;
uint32_t LCD_ActiveLayer=1;
uint8_t  status = 0;
TS_StateTypeDef  TS_State;
/* */
uint8_t stext[100];      /* File read buffer */
unsigned int v3DVal;     /* Indica il valore del movimento da applicare all'oggetto 3D */
unsigned int v3DFn;      /* Indica la funzione da svolgere: XYZRot, Zoom, Move */
unsigned int v3DZoomVal;     /* Indica il valore del movimento da applicare all'oggetto 3D */
unsigned int v3DZoomFn;      /* Indica la funzione da svolgere: XYZRot, Zoom, Move */
/* */
float Dst[10];
float DstOld[10];
int TouchPressed=0;
short xo, yo, dx, dy, dx_old, dy_old;
unsigned char xdir, ydir;
int elapsTime;
/* */
volatile unsigned int doRedraw=0;
/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);
static void LCD_InitApplication(void);
void TimeMeasureInit(void);
void esegui_mv( int c,  int k);
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point );
void DistanceCalc_asm( uint16_t*pTchX, uint16_t*pTchY, float*pDstn, unsigned int size);
void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc);
void FBSwitch( void);
/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
 {   
  FRESULT res;                                          /* FatFs function common result code */
	 
  /* Enable the CPU Cache */
   CPU_CACHE_Enable();

  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();
  
  /* Configure the system clock to 216 MHz */
  SystemClock_Config();
  
  /* Configure LED1  */
  BSP_LED_Init(LED1);
  
  /* Init Audio Application */
  LCD_InitApplication();
	
	status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if (status != TS_OK) {
		while(1);
	}
	
	TimeMeasureInit();
	
	v3DFn  = cEXEC_XYROT;
  v3DVal = cXY_NORD;
	
	Init3D();
	BSP_LCD_Clear(UNPLOT);
	
  /* Infinite loop */
  while (1)
  {
		//
		BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected) {
			if ( TS_State.touchDetected ==2)	{	// multitouch...
				// 
				TouchPressed=0;
				DistanceCalc_asm( (uint16_t*)&TS_State.touchX[0], (uint16_t*)&TS_State.touchY[0], &Dst[0], TS_State.touchDetected);
				//
				v3DZoomFn  = cEXEX_ZOOM;
				v3DZoomVal = 0;				
				if ( Dst[0] > DstOld[0] && (Dst[0]-DstOld[0])>5.0f) {
					v3DZoomVal = cZOOM_OUT;
				} else if ( Dst[0] < DstOld[0] && (DstOld[0]-Dst[0])>5.0f) {
					v3DZoomVal = cZOOM_IN;					
				}
				if ( v3DZoomVal!=0) {
					DstOld[0] = Dst[0];
					TIM_MEASURE_START;
					esegui_mv( v3DZoomVal, v3DZoomFn);				
					TIM_MEASURE_END(cTIME_MEASURE_1);
					sprintf((char*)stext, "#%f f/s",tdiff[cTIME_MEASURE_1]);
					BSP_LCD_SetFont(&Font12);
					BSP_LCD_DisplayStringAt( 0, 260, stext, CENTER_MODE);
					FBSwitch();
				}
			} else if ( TS_State.touchDetected ==1)	{														// single touch
				if ( TouchPressed==0) {
					TouchPressed=1;
					xo=TS_State.touchX[0];
					yo=TS_State.touchY[0];
				} else {
					dx=TS_State.touchX[0]-xo;
					dy=TS_State.touchY[0]-yo;
					if ( abs(dx-dx_old)>5 || abs(dy-dy_old)>5) {
						//
						dx_old=dx; dy_old=dy;
						//
						xdir=ydir=0;
						if ( dx!=0 && (abs(dx)>10)) { 
							if ( dx>0) {
								xdir=1;
							} else {
								xdir=4;
							}
						}
						//xdir = (dx>0)?1:4;
						if ( dy!=0 && (abs(dy)>10)) {
							if ( dy>0) {
								ydir=2;
							} else {
								ydir=7;
							}
						}
						//ydir = (dy>0)?2:7;
						v3DVal = xdir + ydir;
						if ( v3DVal != 0) {
							esegui_mv( v3DVal, v3DFn);
							FBSwitch();
						}
						//xo=TS_State.touchX[0];
						//yo=TS_State.touchY[0];
					}
				}
			}
		} else {
			//
			TouchPressed=0;
			//
			TIM_MEASURE_START;
			esegui_mv( v3DVal, v3DFn);
			TIM_MEASURE_END(cTIME_MEASURE_1);
			sprintf((char*)stext, "#%f f/s",tdiff[cTIME_MEASURE_1]);
			BSP_LCD_SetFont(&Font12);
			BSP_LCD_DisplayStringAt( 0, 260, stext, CENTER_MODE);

#if 0			
			while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));			// ****************
			BSP_LCD_SetLayerVisible( !LCD_ActiveLayer, DISABLE);
			while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));			// ****************
			BSP_LCD_SetLayerVisible( LCD_ActiveLayer, ENABLE);
      BSP_LCD_SelectLayer(!LCD_ActiveLayer);
			while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));
			BSP_LCD_SetLayerVisible( !LCD_ActiveLayer, DISABLE);
			
			LCD_ActiveLayer = !LCD_ActiveLayer;
#else
			FBSwitch();
#endif
		}
		elapsTime=(int)(tdiff[cTIME_MEASURE_1]*1000.0f);
		if ( elapsTime<25)
			HAL_Delay( 25-elapsTime);
  }
}

void FBSwitch( void)
{
	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));			// ****************
	BSP_LCD_SetLayerVisible( !LCD_ActiveLayer, DISABLE);
	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));			// ****************
	BSP_LCD_SetLayerVisible( LCD_ActiveLayer, ENABLE);
	BSP_LCD_SelectLayer(!LCD_ActiveLayer);
	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));
	BSP_LCD_SetLayerVisible( !LCD_ActiveLayer, DISABLE);
	
	LCD_ActiveLayer = !LCD_ActiveLayer;
	
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
    return (TRUE);
  }
  return (FALSE);
}

/**
  * @brief  LCD Init.
  * @param  None
  * @retval None
  */
static void LCD_InitApplication(void)
{
	
  /* Initialize the LCD */
  BSP_LCD_Init();

  /* LCD Initialization */ 
  BSP_LCD_LayerDefaultInit(0, LCD_FB_START_ADDRESS);
  BSP_LCD_LayerDefaultInit(1, LCD_FB_START_ADDRESS+(BSP_LCD_GetXSize()*BSP_LCD_GetYSize()*4));
    
  /* Enable the LCD */ 
  BSP_LCD_DisplayOn(); 
  
  /* Select the LCD Layer 0 */
  BSP_LCD_SelectLayer(0);
	BSP_LCD_SetBackColor( LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor( LCD_COLOR_WHITE); 
  /* Clear the Background Layer */ 
  BSP_LCD_Clear(LCD_COLOR_BLACK);  
  
  /* Select the LCD Layer 1 */
  BSP_LCD_SelectLayer(1);
	BSP_LCD_SetBackColor( LCD_COLOR_BLACK);
	BSP_LCD_SetTextColor( LCD_COLOR_WHITE); 
  /* Clear the Foreground Layer */ 
  BSP_LCD_Clear(LCD_COLOR_BLACK);	

	/* default on LCD Layer 1 */
	BSP_LCD_SelectLayer(!LCD_ActiveLayer);
	BSP_LCD_SetLayerVisible( !LCD_ActiveLayer, ENABLE);
	BSP_LCD_SetLayerVisible( LCD_ActiveLayer, DISABLE);

}
	
/**
  * @brief  Clock Config.
  * @param  hltdc: LTDC handle
  * @note   This API is called by BSP_LCD_Init()
  * @retval None
  */
void BSP_LCD_ClockConfig(LTDC_HandleTypeDef *hltdc, void *Params)
{
  static RCC_PeriphCLKInitTypeDef  periph_clk_init_struct;
  
  /* RK043FN48H LCD clock configuration */
  /* PLLSAI_VCO Input = HSE_VALUE/PLL_M = 1 Mhz */
  /* PLLSAI_VCO Output = PLLSAI_VCO Input * PLLSAIN = 192 Mhz */
  /* PLLLCDCLK = PLLSAI_VCO Output/PLLSAIR = 192/5 = 38.4 Mhz */
  /* LTDC clock frequency = PLLLCDCLK / LTDC_PLLSAI_DIVR_4 = 38.4/4 = 9.6Mhz */
  periph_clk_init_struct.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
  periph_clk_init_struct.PLLSAI.PLLSAIN = 192;
  periph_clk_init_struct.PLLSAI.PLLSAIR = RK043FN48H_FREQUENCY_DIVIDER;
  periph_clk_init_struct.PLLSAIDivR = RCC_PLLSAIDIVR_4;
  HAL_RCCEx_PeriphCLKConfig(&periph_clk_init_struct);
	
}

/**
  * System Clock Configuration
  *   System Clock source            = PLL (HSE)
  *   SYSCLK(Hz)                     = 200000000
  *   HCLK(Hz)                       = 200000000
  *   AHB Prescaler                  = 1
  *   APB1 Prescaler                 = 4
  *   APB2 Prescaler                 = 2
  *   HSE Frequency(Hz)              = 25000000
  *   PLL_M                          = 25
  *   PLL_N                          = 400
  *   PLL_P                          = 2
  *   PLL_Q                          = 8
  *   VDD(V)                         = 3.3
  *   Main regulator output voltage  = Scale1 mode
  *   Flash Latency(WS)              = 6
  */
static void SystemClock_Config (void) {
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  HAL_PWREx_EnableOverDrive();
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* Turn LED1 on */
  BSP_LED_On(LED1);
  while(1)
  {
    BSP_LED_Toggle(LED1);
    HAL_Delay(200);
  }
}

#ifdef USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif /* USE_FULL_ASSERT */ 

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  (*(uint32_t *) 0xE000ED94) &= ~0x5;
  (*(uint32_t *) 0xE000ED98) = 0x0; //MPU->RNR
  (*(uint32_t *) 0xE000ED9C) = 0x20010000 |1<<4; //MPU->RBAR
  (*(uint32_t *) 0xE000EDA0) = 0<<28 | 3 <<24 | 0<<19 | 0<<18 | 1<<17 | 0<<16 | 0<<8 | 30<<1 | 1<<0 ; //MPU->RASE  WT
  (*(uint32_t *) 0xE000ED94) = 0x5;
		
  /* Enable I-Cache */
	SCB_InvalidateICache();

  /* Enable branch prediction */
  SCB->CCR |= (1 <<18); 
  __DSB();
	
  SCB_EnableICache();

  /* Enable D-Cache */
	SCB_InvalidateDCache();
  SCB_EnableDCache();
}

 void TimeMeasureInit(void)
 {
	 /* Set Timers instance */
  TimHandle.Instance = TIM2;
  
	__HAL_RCC_TIM2_CLK_ENABLE();
	__HAL_RCC_TIMCLKPRESCALER(RCC_TIMPRES_ACTIVATED); // run the timer on HCLK freq
  
  /*====================== Master configuration : TIM2 =======================*/
  /* Initialize TIM2 peripheral in counter mode*/
  TimHandle.Init.Period            = 0xFFFFFFFF;
  TimHandle.Init.Prescaler         = 0;
  TimHandle.Init.ClockDivision     = 0;
  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
  TimHandle.Init.RepetitionCounter = 0;
  if(HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }  
	
	HAL_TIM_Base_Start(&TimHandle);
 }

void HAL_LTDC_LineEvenCallback(LTDC_HandleTypeDef *hltdc)
{
	doRedraw=1;
  HAL_LTDC_ProgramLineEvent(hltdc, 0);
}
 
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
