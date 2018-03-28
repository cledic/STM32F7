/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f769i_discovery_ts.h"
#include "arm_math.h"
#include "arm_const_structs.h"

/* Macro to save different timing */
#define	cTIME_MEASURE_1		0
#define	cTIME_MEASURE_2		1
#define	cTIME_MEASURE_3		2
#define	cTIME_MEASURE_4		3
#define	cTIME_MEASURE_5		4
/* Use this macro to keep track of the time */
#define TIM_MEASURE_START {time_start = __HAL_TIM_GET_COUNTER(&TimHandle);}
/* Use this macro with index to save a partial time. */
#define TIM_MEASURE_END(x) {time_end = __HAL_TIM_GET_COUNTER(&TimHandle);     \
														time_diff = time_end - time_start;								\
														tdiff[x]=(float)time_diff*(float)0.000000005;}

TIM_HandleTypeDef     TimHandle;
uint32_t time_start, time_end, time_diff;
float tdiff[5];
void TimeMeasureInit( void);

/* File system object for SD card logical drive */
FATFS SD_FatFs;  
/* SD card logical drive path */
char SD_Path[4]; 

/* Private function prototypes -----------------------------------------------*/
static void LCD_Config(void);
static void SystemClock_Config(void);
static void Error_Handler(void);
static void CPU_CACHE_Enable(void);;

void cifar( uint8_t*img, int8_t*outd);
uint8_t str[128];
uint8_t TS_status = 0;
/* Function to verifify where the user tap on the screen. */
uint32_t CheckRectangle (uint32_t X_up, uint32_t Y_up,
                        uint32_t X_size, uint32_t Y_size,
                        uint32_t X_point, uint32_t Y_point );

TS_StateTypeDef  TS_State;
uint32_t Touch;
uint32_t XTouch;
uint32_t YTouch;

uint32_t xpos, ypos;
uint32_t buff[32*32];
uint8_t image[32*32*3];
uint32_t i, ii, tmp;
/* Log file utility */
uint32_t writeInfo( uint32_t imgidx, uint32_t cres, uint32_t cidx, float ctm, uint32_t estimation);
uint32_t eraseInfo( void);

/*
 * Union data for CIFAR10 binary image format:
 * <1..9 label><32*32*3 = 3072 byte>
 * the first byte is the label of the first image, which is a number in the range 0-9.
 * The next 3072 bytes are the values of the pixels of the image. The first 1024 bytes are the
 * red channel values, the next 1024 the green, and the final 1024 the blue. The values are stored
 * in row-major order, so the first 32 bytes are the red channel values of the first row of the image.
 * https://www.cs.toronto.edu/~kriz/cifar.html
*/
typedef union
{
	uint8_t image[IMAGE_RECORD_LEN];      // full array: label + raw image data
	struct
	{
		uint8_t  label;             // label as values from 0 to 9
		uint8_t  r[1024];           // RGB raw data
		uint8_t  g[1024];
		uint8_t  b[1024];
	} color;
} CIFAR10_IMAGE_t;
CIFAR10_IMAGE_t img;

int8_t binaryImage[IMAGE_SIZE];
const char*labels[10];
uint32_t readImage( uint32_t idx);
void plotImage( uint32_t x, uint32_t y);
int8_t outdata[10];

/* Union for single pixel representation. */
typedef union 
{
  unsigned int   val;
  unsigned short sval[2];
  unsigned char  bval[4];
} SAMPLE;
SAMPLE colorVal;

/* CIFAR calculation result. */
int8_t cifarResult;
uint32_t cifarIndex;
uint32_t cifarOk=0;
uint32_t cifarBad=0;
uint32_t estimation;

uint32_t checkBox( uint32_t cidx, uint32_t x, uint32_t y);

/* RNG handle for random number generation */
RNG_HandleTypeDef RngHandle;
__IO uint32_t aRandom32bit;

/* X and Y for the singole image position on screen. */
uint32_t xt, yt;
void timeCheckInit( void);
#define TIMECHECKUP					HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_0, GPIO_PIN_SET);
#define TIMECHECKDOWN				HAL_GPIO_WritePin(GPIOJ, GPIO_PIN_0, GPIO_PIN_RESET);

char line[128];

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* CIFAR Labels */  
	labels[0]="airplane";
	labels[1]="automobile";
	labels[2]="bird";
	labels[3]="cat";
	labels[4]="deer";
	labels[5]="dog";
	labels[6]="frog";
	labels[7]="horse";
	labels[8]="ship";
	labels[9]="truck";

  /* Enable the CPU Cache */
  CPU_CACHE_Enable();
  
  /* STM32F7xx HAL library initialization:
       - Configure the Flash ART accelerator on ITCM interface
       - Configure the Systick to generate an interrupt each 1 msec
       - Set NVIC Group Priority to 4
       - Global MSP (MCU Support Package) initialization
     */
  HAL_Init();

  /* Configure the system clock to 200 MHz */
  SystemClock_Config();
	/* */
	TimeMeasureInit();
	timeCheckInit();
	
  /* Configure LED1 */
  BSP_LED_Init(LED1);

	/* Random number generator Init. */
	RngHandle.Instance = RNG;
  if (HAL_RNG_Init(&RngHandle) != HAL_OK)
  {
    /* Initialization Error */
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR INITIALIZING RANDOM GENERATOR DRIVER");						
    while(1);
  }

  /* LCD Init */
  LCD_Config();
	/* Clear the background */
	BSP_LCD_Clear(LCD_COLOR_WHITE);
	/* Set the Text Color */
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	/* Touchscreen Init */
	TS_status = BSP_TS_Init(BSP_LCD_GetXSize(), BSP_LCD_GetYSize());
	if (TS_status != TS_OK) 
	{
		/* Initialization Error */
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR INITIALIZING TouchScreen Driver");						
		while(1);
	}
	
	/* SDCard driver init. */
	if(FATFS_LinkDriver(&SD_Driver, SD_Path) != 0)
	{
		/* Initialization Error */
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR INITIALIZING FATFS DRIVER");				
		while(1);
	}
	/* */
	if(f_mount(&SD_FatFs, (TCHAR const*)"",0))
	{
		/* Initialization Error */
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		BSP_LCD_ClearStringLine(1);
		BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR MOUNTING SD File System");		
		while(1);
	}
	
	/* Delete Log file. */
	eraseInfo();
	
	/* Loop reading the images from the binary file */
	for ( yt=0; yt<15; yt++)		// Image vertical position y
	{
		for ( xt=0; xt<25; xt++)	// Image orizzontal position x
		{
			/* Generate a new random number and  use it as an index to read the image. */
			aRandom32bit = HAL_RNG_GetRandomNumber(&RngHandle);
			aRandom32bit = (aRandom32bit % 9990) + 1;
			/* Read an image from the CIFAR binary file. */
			if ( readImage( aRandom32bit /*(yt*32)+xt*/ )) 
			{
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_ClearStringLine(1);
				BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR READING IMAGE DATA FROM BINARY FILE");
				while(1);
			}
			/* Display the image on screen. */
			plotImage( xt, yt);
			/* Start timing measurement with Timer and GPIO pin. */
			TIMECHECKUP;TIM_MEASURE_START;
			/* Start Neural Network analisys. */
			cifar( img.image, &outdata[0]);
			TIM_MEASURE_END(cTIME_MEASURE_1); TIMECHECKDOWN; 
			/* Find the max value and relative index */
			arm_max_q7(outdata, 10, &cifarResult, &cifarIndex);
			/* Check the result */
			estimation = checkBox( cifarIndex, xt, yt);
			/* Log info to file. */
			if ( writeInfo( aRandom32bit, cifarResult, cifarIndex, tdiff[cTIME_MEASURE_1], estimation) )
			{
				/* Error */
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_RED);
				BSP_LCD_ClearStringLine(1);
				BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"ERROR WRITING LOG INFO FILE");
				while(1);				
			}
		}
	}

  /* Loop wait for a tap on the screen */
  while( 1) 
	{
		BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected) 
		{
			XTouch = TS_State.touchX[0];
			YTouch = TS_State.touchY[0];
			if( 0 == Touch) 
			{
				Touch = 1;
				BSP_LCD_Clear(LCD_COLOR_WHITE);
				BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
				BSP_LCD_ClearStringLine(1);
				BSP_LCD_DisplayStringAtLine(1, (uint8_t *)"Test on 375 images from file: test_batch.bin.");
				sprintf( line, " Images correctly estimated: %d, %3.1f%%.", cifarOk, (float)((cifarOk*1.0f/375.0f)*100.0f));
				BSP_LCD_DisplayStringAtLine(2, (uint8_t *)line);
				sprintf( line, " Execution Time for each image: %2.4f ms.", tdiff[cTIME_MEASURE_1]);
				BSP_LCD_DisplayStringAtLine(3, (uint8_t *)line);	
			}
    } else if( Touch) {
        Touch = 0;
    }
  }
	
	while(1)
	{
		;
	}
}

/**
 * @brief Read a single image
 * @param[in]	idx	image index.
 * @retval		1 => Error 0 => OK
*/
uint32_t readImage( uint32_t idx)
{
	FIL fp;
	UINT BytesRead;
	uint8_t label;
	uint32_t i;
	char fn[32];
	
	/* Apro il file binario delle imagini */
	if (f_open(&fp, (TCHAR const*)"test_batch.bin", FA_READ) != FR_OK)
	{
		return 1;
	}
	/* Mi posiziono sull'immagine richiesta. */
	f_lseek(&fp, IMAGE_RECORD_LEN*idx);
	/* Leggo il record con l'indice della label ed i colori. */
	f_read (&fp, img.image, IMAGE_RECORD_LEN, (UINT *)&BytesRead);
	f_close(&fp);
	
	if ( BytesRead != IMAGE_RECORD_LEN)
	{
		return 1;
	} 
	
	return 0;

}

/**
 * @brief Plot the image to the screen
 * @param[in]		x		x position
 * @param[in]		y 	y position
*/
void plotImage( uint32_t x, uint32_t y)
{
	uint32_t yp, xp, idx_p;
	
	idx_p=0;
	for ( yp=0; yp<32; yp++)		// y
	{
		for ( xp=0; xp<32; xp++)	// x
		{
			/* Compose the pixel value: ARGB */
			colorVal.bval[2]=img.color.r[idx_p];
			colorVal.bval[1]=img.color.g[idx_p];
			colorVal.bval[0]=img.color.b[idx_p];
			colorVal.bval[3]=0xFF;
			idx_p++;
			while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));
			BSP_LCD_DrawPixel( (x*32)+xp, (y*32)+yp, colorVal.val);
		}
	}
}

/**
 * @brief Plot a green/red box around the image
 * @param[in]		cidx	CIFAR index result
 * @param[in]		x			x position
 * @param[in]		y			y position
 * @retval			1 => Bad 0 => OK
*/
uint32_t checkBox( uint32_t cidx, uint32_t x, uint32_t y)
{
	uint32_t res=0;
	
	if ( cidx == img.color.label)
	{
		BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
		cifarOk++;
		res=0;
	} else {
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		cifarBad++;
		res=1;
	}
	/* */
	while (!(LTDC->CDSR & LTDC_CDSR_VSYNCS));
	BSP_LCD_DrawRect( x*32, y*32, 32, 32);	
	/* */
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	return res;
}

/**
 * @brief Write info about CIFAR result to a log
 * @param[in]	cres	CIFAR result
 * @param[in]	cidx	CIFAR image index
 * @param[in]	ctm		CIFAR time execution
 * @retval					1 => Error, 0 => OK
*/
uint32_t writeInfo( uint32_t imgidx, uint32_t cres, uint32_t cidx, float ctm, uint32_t estimation)
{
	FIL fp;
	UINT BytesRead;
	uint8_t label;
	
	/* Apro il file binario delle imagini */
	if (f_open(&fp, (TCHAR const*)"test_batch.txt", FA_OPEN_ALWAYS|FA_WRITE) != FR_OK)
	{
		return 1;
	}
	/* Move to end of the file to append data */
	f_lseek(&fp, f_size(&fp));
	/* Prepare the string to log. */
	sprintf( line, "Check:%s,Image#:%d,Label:%s,Label#:%02d,Result#:%02d,ResultVal:%d,Time:%2.4f;\n",(estimation==0)?"OK":"BAD",imgidx,labels[img.color.label],img.color.label,cidx,cres,ctm);
	f_puts(line, &fp); 	
	f_sync(&fp);
	f_close(&fp);
	
	return 0;
}

/**
 * @brief Erase the log file.
 * @retval		1 => Error, 0 => OK
*/
uint32_t eraseInfo( void)
{
	if ( f_unlink ((TCHAR const*)"test_batch.txt") != FR_OK)
		return 1;
	else
		return 0;
}

/* */
void timeCheckInit( void)
{
  GPIO_InitTypeDef  gpio_init_structure;
  
  __HAL_RCC_GPIOJ_CLK_ENABLE();
  
  gpio_init_structure.Pin   = GPIO_PIN_0;
  gpio_init_structure.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio_init_structure.Pull  = GPIO_PULLUP;
  gpio_init_structure.Speed = GPIO_SPEED_HIGH;
  
  HAL_GPIO_Init(GPIOJ, &gpio_init_structure);
}

/**
  * @brief  LCD configuration
  * @param  None
  * @retval None
  */
static void LCD_Config(void)
{
  uint8_t lcd_status = LCD_OK;
  
  /* LCD DSI initialization in mode Video Burst */
  /* Initialize DSI LCD */
  BSP_LCD_Init();
  while(lcd_status != LCD_OK);
  
  BSP_LCD_LayerDefaultInit(LTDC_ACTIVE_LAYER_BACKGROUND, LCD_FB_START_ADDRESS);
  
  /* Select the LCD Background Layer */
  BSP_LCD_SelectLayer(LTDC_ACTIVE_LAYER_BACKGROUND);
  
  /* Clear the Background Layer */
  BSP_LCD_Clear(LCD_COLOR_BLACK);

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
  }
}

/**
  * @brief  CPU L1-Cache enable.
  * @param  None
  * @retval None
  */
static void CPU_CACHE_Enable(void)
{
  /* Enable I-Cache */
  SCB_EnableICache();

  /* Enable D-Cache */
  SCB_EnableDCache();
}


/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow : 
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 200000000
  *            HCLK(Hz)                       = 200000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 25000000
  *            PLL_M                          = 25
  *            PLL_N                          = 432
  *            PLL_P                          = 2
  *            PLL_Q                          = 9
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 7
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  HAL_StatusTypeDef ret = HAL_OK;

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 400;  
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 7;

  ret = HAL_RCC_OscConfig(&RCC_OscInitStruct);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }

  /* Activate the OverDrive to reach the 200 MHz Frequency */
  ret = HAL_PWREx_EnableOverDrive();
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  ret = HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
  if(ret != HAL_OK)
  {
    while(1) { ; }
  }
}

/**
 * @brief
 * @param[in] none
 * @retval 		none
*/
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
  * @brief RNG MSP Initialization
  *        This function configures the hardware resources used in this example:
  *           - Peripheral's clock enable
  * @param hrng: RNG handle pointer
  * @retval None
  */
void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{
  /* RNG Peripheral clock enable */
  __HAL_RCC_RNG_CLK_ENABLE();
}

/**
  * @brief RNG MSP De-Initialization
  *        This function freeze the hardware resources used in this example:
  *          - Disable the Peripheral's clock
  * @param hrng: RNG handle pointer
  * @retval None
  */
void HAL_RNG_MspDeInit(RNG_HandleTypeDef *hrng)
{
  /* Enable RNG reset state */
  __HAL_RCC_RNG_FORCE_RESET();

  /* Release RNG from reset state */
  __HAL_RCC_RNG_RELEASE_RESET();
}

#ifdef  USE_FULL_ASSERT
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
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
