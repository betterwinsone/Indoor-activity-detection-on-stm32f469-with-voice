/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Inc/main.h 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-February-2017
  * @brief   Header for main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */  
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "usbh_core.h"
#include "usbh_msc.h" 
#include "stm32f4xx_hal.h"
#include "stm32469i_discovery.h"
#include "stm32469i_discovery_audio.h"
#include "stm32469i_discovery_ts.h"
#include "lcd_log.h"
#include "ff.h"    
#include "ff_gen_drv.h"
#include "usbh_diskio.h"

/* Exported Defines ----------------------------------------------------------*/
#define AUDIO_OUT_BUFFER_SIZE                      512
#define AUDIO_IN_PCM_BUFFER_SIZE                   4*2304 /* buffer size in half-word */
#define AUDIO_IN_PDM_BUFFER_SIZE                   INTERNAL_BUFF_SIZE

#define FILEMGR_LIST_DEPDTH                        24
#define FILEMGR_FILE_NAME_SIZE                     40
#define FILEMGR_FULL_PATH_SIZE                     256
#define FILEMGR_MAX_LEVEL                          4    
#define FILETYPE_DIR                               0
#define FILETYPE_FILE                              1

#define SAI1_FSA_Pin GPIO_PIN_4
#define SAI1_FSA_GPIO_Port GPIOE
#define SPKR_HP_Pin GPIO_PIN_3
#define SPKR_HP_GPIO_Port GPIOE
#define AUDIO_RST_Pin GPIO_PIN_2
#define AUDIO_RST_GPIO_Port GPIOE
#define ARDUINO_USART6_TX_Pin GPIO_PIN_14
#define ARDUINO_USART6_TX_GPIO_Port GPIOG
#define FMC_NBL1_Pin GPIO_PIN_1
#define FMC_NBL1_GPIO_Port GPIOE
#define FMC_NBL0_Pin GPIO_PIN_0
#define FMC_NBL0_GPIO_Port GPIOE
#define I2C1_SCL_Pin GPIO_PIN_8
#define I2C1_SCL_GPIO_Port GPIOB
#define I2S3_CK_Pin GPIO_PIN_3
#define I2S3_CK_GPIO_Port GPIOB
#define uSD_CLK_Pin GPIO_PIN_12
#define uSD_CLK_GPIO_Port GPIOC
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SAI1_SCKA_Pin GPIO_PIN_5
#define SAI1_SCKA_GPIO_Port GPIOE
#define I2C1_SDA_Pin GPIO_PIN_9
#define I2C1_SDA_GPIO_Port GPIOB
#define OTG_FS1_OverCurrent_Pin GPIO_PIN_7
#define OTG_FS1_OverCurrent_GPIO_Port GPIOB
#define QSPI_BK1_NCS_Pin GPIO_PIN_6
#define QSPI_BK1_NCS_GPIO_Port GPIOB
#define SDNCAS_Pin GPIO_PIN_15
#define SDNCAS_GPIO_Port GPIOG
#define MIC_DATA_Pin GPIO_PIN_6
#define MIC_DATA_GPIO_Port GPIOD
#define D2_Pin GPIO_PIN_0
#define D2_GPIO_Port GPIOD
#define uSD_D3_Pin GPIO_PIN_11
#define uSD_D3_GPIO_Port GPIOC
#define uSD_D2_Pin GPIO_PIN_10
#define uSD_D2_GPIO_Port GPIOC
#define USB_FS1_P_Pin GPIO_PIN_12
#define USB_FS1_P_GPIO_Port GPIOA
#define FMC_NBL2_Pin GPIO_PIN_4
#define FMC_NBL2_GPIO_Port GPIOI
#define LED3_Pin GPIO_PIN_5
#define LED3_GPIO_Port GPIOD
#define D3_Pin GPIO_PIN_1
#define D3_GPIO_Port GPIOD
#define D27_Pin GPIO_PIN_3
#define D27_GPIO_Port GPIOI
#define D26_Pin GPIO_PIN_2
#define D26_GPIO_Port GPIOI
#define USB_FS1_N_Pin GPIO_PIN_11
#define USB_FS1_N_GPIO_Port GPIOA
#define A0_Pin GPIO_PIN_0
#define A0_GPIO_Port GPIOF
#define FMC_NBL3_Pin GPIO_PIN_5
#define FMC_NBL3_GPIO_Port GPIOI
#define D29_Pin GPIO_PIN_7
#define D29_GPIO_Port GPIOI
#define D31_Pin GPIO_PIN_10
#define D31_GPIO_Port GPIOI
#define D28_Pin GPIO_PIN_6
#define D28_GPIO_Port GPIOI
#define LED4_Pin GPIO_PIN_3
#define LED4_GPIO_Port GPIOK
#define USART6_RX_Pin GPIO_PIN_9
#define USART6_RX_GPIO_Port GPIOG
#define LED2_Pin GPIO_PIN_4
#define LED2_GPIO_Port GPIOD
#define uSD_CMD_Pin GPIO_PIN_2
#define uSD_CMD_GPIO_Port GPIOD
#define D23_Pin GPIO_PIN_15
#define D23_GPIO_Port GPIOH
#define D25_Pin GPIO_PIN_1
#define D25_GPIO_Port GPIOI
#define USB_FS1_ID_Pin GPIO_PIN_10
#define USB_FS1_ID_GPIO_Port GPIOA
#define A1_Pin GPIO_PIN_1
#define A1_GPIO_Port GPIOF
#define D30_Pin GPIO_PIN_9
#define D30_GPIO_Port GPIOI
#define D21_Pin GPIO_PIN_13
#define D21_GPIO_Port GPIOH
#define D22_Pin GPIO_PIN_14
#define D22_GPIO_Port GPIOH
#define D24_Pin GPIO_PIN_0
#define D24_GPIO_Port GPIOI
#define VBUS_FS1_Pin GPIO_PIN_9
#define VBUS_FS1_GPIO_Port GPIOA
#define uSD_D1_Pin GPIO_PIN_9
#define uSD_D1_GPIO_Port GPIOC
#define A2_Pin GPIO_PIN_2
#define A2_GPIO_Port GPIOF
#define uSD_D0_Pin GPIO_PIN_8
#define uSD_D0_GPIO_Port GPIOC
#define A3_Pin GPIO_PIN_3
#define A3_GPIO_Port GPIOF
#define I2C2_SCL_Pin GPIO_PIN_4
#define I2C2_SCL_GPIO_Port GPIOH
#define SDCLK_Pin GPIO_PIN_8
#define SDCLK_GPIO_Port GPIOG
#define A4_Pin GPIO_PIN_4
#define A4_GPIO_Port GPIOF
#define I2C2_SDA_Pin GPIO_PIN_5
#define I2C2_SDA_GPIO_Port GPIOH
#define SDNE0_Pin GPIO_PIN_3
#define SDNE0_GPIO_Port GPIOH
#define SAI1_MCLKA_Pin GPIO_PIN_7
#define SAI1_MCLKA_GPIO_Port GPIOG
#define LED1_Pin GPIO_PIN_6
#define LED1_GPIO_Port GPIOG
#define QSPI_BK1_IO2_Pin GPIO_PIN_7
#define QSPI_BK1_IO2_GPIO_Port GPIOF
#define QSPI_BK1_IO3_Pin GPIO_PIN_6
#define QSPI_BK1_IO3_GPIO_Port GPIOF
#define A5_Pin GPIO_PIN_5
#define A5_GPIO_Port GPIOF
#define SDCKE0_Pin GPIO_PIN_2
#define SDCKE0_GPIO_Port GPIOH
#define D1_Pin GPIO_PIN_15
#define D1_GPIO_Port GPIOD
#define D15_Pin GPIO_PIN_10
#define D15_GPIO_Port GPIOD
#define QSPI_CLK_Pin GPIO_PIN_10
#define QSPI_CLK_GPIO_Port GPIOF
#define QSPI_BK1_IO1_Pin GPIO_PIN_9
#define QSPI_BK1_IO1_GPIO_Port GPIOF
#define QSPI_BK1_IO0_Pin GPIO_PIN_8
#define QSPI_BK1_IO0_GPIO_Port GPIOF
#define D0_Pin GPIO_PIN_14
#define D0_GPIO_Port GPIOD
#define D14_Pin GPIO_PIN_9
#define D14_GPIO_Port GPIOD
#define D13_Pin GPIO_PIN_8
#define D13_GPIO_Port GPIOD
#define SDNWE_Pin GPIO_PIN_0
#define SDNWE_GPIO_Port GPIOC
#define OTG_FS1_PowerSwitchOn_Pin GPIO_PIN_2
#define OTG_FS1_PowerSwitchOn_GPIO_Port GPIOB
#define A6_Pin GPIO_PIN_12
#define A6_GPIO_Port GPIOF
#define A11_Pin GPIO_PIN_1
#define A11_GPIO_Port GPIOG
#define A9_Pin GPIO_PIN_15
#define A9_GPIO_Port GPIOF
#define MIC_CK_Pin GPIO_PIN_13
#define MIC_CK_GPIO_Port GPIOD
#define uSD_Detect_Pin GPIO_PIN_2
#define uSD_Detect_GPIO_Port GPIOG
#define LCD_INT_Pin GPIO_PIN_5
#define LCD_INT_GPIO_Port GPIOJ
#define D20_Pin GPIO_PIN_12
#define D20_GPIO_Port GPIOH
#define WAKEUP_Pin GPIO_PIN_0
#define WAKEUP_GPIO_Port GPIOA
#define A7_Pin GPIO_PIN_13
#define A7_GPIO_Port GPIOF
#define A10_Pin GPIO_PIN_0
#define A10_GPIO_Port GPIOG
#define D5_Pin GPIO_PIN_8
#define D5_GPIO_Port GPIOE
#define D17_Pin GPIO_PIN_9
#define D17_GPIO_Port GPIOH
#define D19_Pin GPIO_PIN_11
#define D19_GPIO_Port GPIOH
#define A8_Pin GPIO_PIN_14
#define A8_GPIO_Port GPIOF
#define DSI_TE_Pin GPIO_PIN_2
#define DSI_TE_GPIO_Port GPIOJ
#define SDNMT48LC4M32B2B5_6A_RAS_RAS___Pin GPIO_PIN_11
#define SDNMT48LC4M32B2B5_6A_RAS_RAS___GPIO_Port GPIOF
#define D6_Pin GPIO_PIN_9
#define D6_GPIO_Port GPIOE
#define D8_Pin GPIO_PIN_11
#define D8_GPIO_Port GPIOE
#define D11_Pin GPIO_PIN_14
#define D11_GPIO_Port GPIOE
#define STLK_RX_Pin GPIO_PIN_10
#define STLK_RX_GPIO_Port GPIOB
#define D16_Pin GPIO_PIN_8
#define D16_GPIO_Port GPIOH
#define D18_Pin GPIO_PIN_10
#define D18_GPIO_Port GPIOH
#define LCD_BL_CTRL_Pin GPIO_PIN_3
#define LCD_BL_CTRL_GPIO_Port GPIOA
#define EXT_RESET_Pin GPIO_PIN_0
#define EXT_RESET_GPIO_Port GPIOB
#define D4_Pin GPIO_PIN_7
#define D4_GPIO_Port GPIOE
#define D7_Pin GPIO_PIN_10
#define D7_GPIO_Port GPIOE
#define D9_Pin GPIO_PIN_12
#define D9_GPIO_Port GPIOE
#define D12_Pin GPIO_PIN_15
#define D12_GPIO_Port GPIOE
#define D10_Pin GPIO_PIN_13
#define D10_GPIO_Port GPIOE
#define STLK_TX_Pin GPIO_PIN_11
#define STLK_TX_GPIO_Port GPIOB

/* Exported types ------------------------------------------------------------*/
/* Application State Machine Structure */
typedef enum {
  APPLICATION_IDLE = 0,  
  APPLICATION_START,   
  APPLICATION_READY,
  APPLICATION_DISCONNECT,
}AUDIO_ApplicationTypeDef;
    
/* Audio Demo State Structure */    
typedef enum {
  AUDIO_DEMO_IDLE = 0,
  AUDIO_DEMO_WAIT,  
  AUDIO_DEMO_EXPLORE,
  AUDIO_DEMO_PLAYBACK,
  AUDIO_DEMO_IN,  
}AUDIO_Demo_State;

/* Audio Demo State Machine Structure */
typedef struct _DemoStateMachine {
  __IO AUDIO_Demo_State state;
  __IO uint8_t select;  
}AUDIO_DEMO_StateMachine;

typedef enum {
  AUDIO_STATE_IDLE = 0,
  AUDIO_STATE_WAIT,    
  AUDIO_STATE_INIT,    
  AUDIO_STATE_PLAY,
  AUDIO_STATE_PRERECORD,
  AUDIO_STATE_RECORD,  
  AUDIO_STATE_NEXT,  
  AUDIO_STATE_PREVIOUS,
  AUDIO_STATE_FORWARD,   
  AUDIO_STATE_BACKWARD,
  AUDIO_STATE_STOP,   
  AUDIO_STATE_PAUSE,
  AUDIO_STATE_RESUME,
  AUDIO_STATE_VOLUME_UP,
  AUDIO_STATE_VOLUME_DOWN,
  AUDIO_STATE_ERROR,  
}AUDIO_PLAYBACK_StateTypeDef;

typedef enum {
  AUDIO_SELECT_MENU = 0,
  AUDIO_PLAYBACK_CONTROL,  
}AUDIO_DEMO_SelectMode;

typedef enum {
  BUFFER_OFFSET_NONE = 0,  
  BUFFER_OFFSET_HALF,  
  BUFFER_OFFSET_FULL,     
}BUFFER_StateTypeDef;

/* Audio buffer control struct */
typedef struct {
  uint8_t buff[AUDIO_OUT_BUFFER_SIZE];
  BUFFER_StateTypeDef state;
  uint32_t fptr;
}AUDIO_OUT_BufferTypeDef;

typedef enum {
  BUFFER_EMPTY = 0,  
  BUFFER_FULL,     
}WR_BUFFER_StateTypeDef;

typedef struct {
  uint16_t pdm_buff[AUDIO_IN_PDM_BUFFER_SIZE];
  uint16_t pcm_buff[AUDIO_IN_PCM_BUFFER_SIZE];
  uint32_t pcm_ptr;
  WR_BUFFER_StateTypeDef wr_state;
  uint32_t offset;  
  uint32_t fptr;
}AUDIO_IN_BufferTypeDef;

typedef struct {
  uint32_t ChunkID;       /* 0 */ 
  uint32_t FileSize;      /* 4 */
  uint32_t FileFormat;    /* 8 */
  uint32_t SubChunk1ID;   /* 12 */
  uint32_t SubChunk1Size; /* 16*/  
  uint16_t AudioFormat;   /* 20 */ 
  uint16_t NbrChannels;   /* 22 */   
  uint32_t SampleRate;    /* 24 */
  
  uint32_t ByteRate;      /* 28 */
  uint16_t BlockAlign;    /* 32 */  
  uint16_t BitPerSample;  /* 34 */  
  uint32_t SubChunk2ID;   /* 36 */   
  uint32_t SubChunk2Size; /* 40 */    
}WAVE_FormatTypeDef;

typedef struct _FILELIST_LineTypeDef {
  uint8_t type;
  uint8_t name[FILEMGR_FILE_NAME_SIZE];
}FILELIST_LineTypeDef;

typedef struct _FILELIST_FileTypeDef {
  FILELIST_LineTypeDef  file[FILEMGR_LIST_DEPDTH] ;
  uint16_t              ptr; 
}FILELIST_FileTypeDef;

typedef enum {
  AUDIO_ERROR_NONE = 0,  
  AUDIO_ERROR_IO,
  AUDIO_ERROR_EOF,
  AUDIO_ERROR_INVALID_VALUE,     
}AUDIO_ErrorTypeDef;

extern USBH_HandleTypeDef hUSBHost;
extern AUDIO_ApplicationTypeDef AppliState;
extern AUDIO_PLAYBACK_StateTypeDef AudioState;
extern FATFS USBH_fatfs;
extern FIL WavFile;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
/* TS API */
uint8_t  Touchscreen_Calibration(void);
uint16_t TouchScreen_Get_Calibrated_X(uint16_t x);
uint16_t TouchScreen_Get_Calibrated_Y(uint16_t y);
uint8_t  TouchScreen_IsCalibrationDone(void);

/* Menu API */
void AUDIO_MenuInit(void);
void AUDIO_MenuProcess(void);
uint8_t AUDIO_ShowWavFiles(void);

/* Disk Explorer API */
uint8_t AUDIO_StorageInit(void); 
FRESULT AUDIO_StorageParse(void);
uint16_t AUDIO_GetWavObjectNumber(void);

/* Toggle LEDs */
void Toggle_Leds(void);

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
