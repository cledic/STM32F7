/**
  ******************************************************************************
  * @file    FatFs/FatFs_uSD/Inc/main.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    25-June-2015
  * @brief   Header for main.c module
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
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

/* FatFs includes component */
#include "ff_gen_drv.h"
#include "sd_diskio.h"

/* Exported define -----------------------------------------------------------*/
/* AudioFreq * DataSize (2 bytes) * NumChannels (Stereo: 2) */
#define DEFAULT_AUDIO_IN_BIT_RESOLUTION     ((uint8_t)16)
#define DEFAULT_AUDIO_IN_CHANNEL_NBR        ((uint8_t)2) /* Mono = 1, Stereo = 2 */
#define DEFAULT_AUDIO_IN_VOLUME             ((uint16_t)64)
#define AUDIO_OUT_BUFFER_SIZE                      8192
#define AUDIO_IN_BUFFER_SIZE                       8192
#define AUDIO_IN_PCM_BUFFER_SIZE                   4*2304 /* buffer size in half-word */

/* Exported types ------------------------------------------------------------*/
typedef enum {
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;

typedef enum {
  BUFFER_EMPTY = 0,  
  BUFFER_FULL,     
}WR_BUFFER_StateTypeDef;

typedef struct {
  uint16_t pcm_buff[AUDIO_IN_BUFFER_SIZE];
  uint32_t pcm_ptr;
  WR_BUFFER_StateTypeDef wr_state;
  uint32_t offset;  
  uint32_t fptr;
}AUDIO_IN_BufferTypeDef;


/* Exported variables --------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
