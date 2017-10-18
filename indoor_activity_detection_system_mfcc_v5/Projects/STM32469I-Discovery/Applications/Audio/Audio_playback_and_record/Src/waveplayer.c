/**
  ******************************************************************************
  * @file    Audio/Audio_playback_and_record/Src/waveplayer.c 
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    17-February-2017
  * @brief   This file provides the Audio Out (playback) interface API
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
/* Includes ------------------------------------------------------------------*/
#include "waveplayer.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "gpio.h"
#include  <stdio.h>
#include  <stdlib.h>
#include  <math.h>
#include  <float.h>//FLT_MAX¡¡
#include  <string.h>
#include "VoiceRecognition.h"
#include "MFCC.h"
#include "AudioData.h"

/* Private define ------------------------------------------------------------*/
#define TOUCH_VOL_PLUS_XMIN     620
#define TOUCH_VOL_PLUS_XMAX     680
#define TOUCH_VOL_PLUS_YMIN     280
#define TOUCH_VOL_PLUS_YMAX     320
#define TEST_LENGTH_SAMPLES     512
/* Private macro -------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static AUDIO_OUT_BufferTypeDef  BufferCtl;
static int16_t FilePos = 0;
static __IO uint32_t uwVolume = 70;
WAVE_FormatTypeDef WaveFormat;
FIL WavFile;
extern FILELIST_FileTypeDef FileList;

uint8_t buff[512];
uint8_t data1[512];
uint8_t data2[512];

uint16_t *pData1=(uint16_t *)data1;
uint16_t *pData2=(uint16_t *)data2;
float audio_data1[256];
float audio_data2[256];
float *p_audio_data1=audio_data1;
float *p_audio_data2=audio_data2;
float *p_mfcc;
float mfcc_data1[39];
float mfcc_data2[39];

float audio_data[256];
float *p_audio_data=audio_data;


/* Private function prototypes -----------------------------------------------*/
static AUDIO_ErrorTypeDef GetFileInfo(uint16_t file_idx, WAVE_FormatTypeDef *info);
static uint8_t PlayerInit(uint32_t AudioFreq);
static void AUDIO_PlaybackDisplayButtons(char *active,uint8_t people);
static void AUDIO_AcquireTouchButtons(void);
uint8_t AT[] = "AT\r\n";
uint8_t data_size[] = "AT+CIPSEND=61\r\n";

uint8_t server_ip_port[] = "AT+CIPSTART=\"TCP\",\"140.138.152.96\",8000\r\n";
char send_data[100];
char meeting[]="meeting";
char working[]="working";
char calling[]="calling";
char *active;
uint8_t people=1;
char people_num[]="3";



/* Private functions ---------------------------------------------------------*/
float dist1=0.0,dist2=0.0;

/* ------------------------------------------------------------------
* Global variables for FFT Bin Example
* ------------------------------------------------------------------- */





/**
  * @brief  Initializes Audio Interface.
  * @param  None
  * @retval Audio error
  */
AUDIO_ErrorTypeDef AUDIO_PLAYER_Init(void)
{
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_AUTO, uwVolume, I2S_AUDIOFREQ_48K) == 0)
  {
    return AUDIO_ERROR_NONE;
  }
  else
  {
    return AUDIO_ERROR_IO;
  }
}

/**
  * @brief  Starts Audio streaming.    
  * @param  idx: File index
  * @retval Audio error
  */ 

void connect_server()
{
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)server_ip_port, sizeof(server_ip_port));
	HAL_Delay(1000);
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)data_size, sizeof(data_size));
	HAL_Delay(1000);
}
void send(char *indoor_statue,uint8_t people_num)
{
	//uint8_t send_data[] = "GET /statues/?indoor_statue=meeting&people_num=2 HTTP/1.1\r\n\0";
	
	sprintf(send_data,"GET /statues/?indoor_statue=%s&people_num=%d HTTP/1.1\r\n",indoor_statue,(int)people_num);
	HAL_UART_Transmit_IT(&huart6, (uint8_t *)send_data, sizeof(send_data));
}
AUDIO_ErrorTypeDef AUDIO_PLAYER_Start(uint8_t idx)
{

  uint32_t bytesread;
  int i=0,j=0,h=0;
	float *pdata;
	//float *t,*T;
	int Tlen=39,tlen=39;
	


	
  f_close(&WavFile);
	
  GetFileInfo(idx, &WaveFormat);
    
    /*Adjust the Audio frequency */
  PlayerInit(WaveFormat.SampleRate); 
    
  BufferCtl.state = BUFFER_OFFSET_NONE;
		
	/*f_open(&WavFile, (char *)FileList.file[0].name, FA_OPEN_EXISTING | FA_READ);
	f_lseek(&WavFile, 44);
	f_read(&WavFile, &data1, 512, (void *)&bytesread);
	f_lseek(&WavFile, 556);
	f_read(&WavFile, &data2, 512, (void *)&bytesread);
	
	for(j=0;j<256;j++)
	{
		audio_data1[j]=*pData1++;
	}
	for(j=0;j<256;j++)
	{
		audio_data2[j]=*pData2++;
	}*/
	f_open(&WavFile, (char *)FileList.file[0].name, FA_OPEN_EXISTING | FA_READ);
	f_lseek(&WavFile, 44);
	data_to_mfcc1(0);
	//data_to_mfcc1(11);
	//data_to_mfcc1(22);
	data_to_mfcc2(0);
	//data_to_mfcc2(11);
	//data_to_mfcc2(22);

	
	/*T=COMPUTING_MFCCs(p_audio_data2);
	for(i=0;i<11;i++)
	{
		mfcc_data2[i]=*T;
		T++;
	}*/
	dist1=cosine_similarity(mfcc_data1,mfcc_data2,11);
	if (0.7311<dist1<=0.7350)
	{
		people=2;
		active=meeting;
	}
		
	else if(0.7350<dist1<=0.7359)
	{
		people=1;
		active=calling;
	}
	else if(0.7359<dist1<=0.7369)
	{
		people=1;
		active=working;
	}
	else if(0.7369<dist1<=0.7379)
	{
		people=3;
		active=meeting;
	}
	else if(0.7379<dist1<=0.7389)
	{	
		people=4;
		active=meeting;
	}
	else
		people=2;
		active=meeting;
		
	//f_lseek(&WavFile, 3116);
	//data_to_mfcc1(0);
	//data_to_mfcc1(11);
	//data_to_mfcc1(22);
	//data_to_mfcc2(0);
	//data_to_mfcc2(11);
	//data_to_mfcc2(22);
	//dist2=cosine_similarity(mfcc_data1,mfcc_data2,11);
	connect_server();
	
	send(active,people);
	connect_server();
	send(active,people);
		
	/* ½çÃæÏÔÊ¾ */
  AudioState = AUDIO_STATE_PLAY;
	AUDIO_PlaybackDisplayButtons(active,people);
	
	
	
	
  return AUDIO_ERROR_NONE;
      
    
  //}
  return AUDIO_ERROR_IO;
}
float *read_data()
{
	uint8_t data[512];
	uint16_t *pData=(uint16_t *)data;
	uint32_t bytesread;
	//f_open(&WavFile, (char *)FileList.file[0].name, FA_OPEN_EXISTING | FA_READ);
	//f_lseek(&WavFile, index);
	f_read(&WavFile, &data, 512, (void *)&bytesread);
	for(int j=0;j<256;j++)
	{
		audio_data[j]=*pData++;
	}
	return p_audio_data;
}
int data_to_mfcc1(int id)
{
	int i;
	float *pdata,*t;
	pdata=read_data();
	t=COMPUTING_MFCCs(pdata);
	for(i=id;i<id+11;i++)
	{
		mfcc_data1[i]=*t;
		t++;
	}
	return 0;
	
}

int data_to_mfcc2(int id)
{
	int i;
	float *pdata,*t;
	pdata=read_data();
	t=COMPUTING_MFCCs(pdata);
	for(i=id;i<id+11;i++)
	{
		mfcc_data2[i]=*t;
		t++;
	}
	return 0;
	
}

/**
  * @brief  distance function
  * @param  None
  * @retval Audio error
  */
float cosine_similarity(float *A, float *B,int Vector_Length)
{
	//double a[39] = { 2.5929434972976035, 3.1115289399913402, -15.443240645371494, 17.358021262230537, 13.816072954764618, 18.807749298605039, -30.15348552366531, -18.325210495620958, -14.751995954057891, 13.652271202517541, -3.5131265976665547, -17.187718009015846, -20.465639993853049, -13.794843403372735, 6.9234783905710202, -3.9808707873788785, -1.9401975575541475, -9.096280032630732, -1.9990121901702154, 6.153856928482309, 12.271365318041498, -15.86828554629815, 18.391277444279392, 22.754714981242191, 15.84748369102684, -34.564316288503314, -17.819982185167046, -12.540870094726847, 17.910555895606837, -3.3065975846731464, -20.610646920307353, -21.264245579119482, -13.324114997632256, 9.7240328513480527, -0.6309011013097543, 4.1640616320164465, -6.756037931121373, -1.5072961232790696, 10.659481131037447 };
	//double b[39] = { 5.3011827389212494, 6.628441786079037, -12.864688350775555, 14.884380017021602, 16.321654473516425, 8.191799714608031, -27.565176738360883, -14.803038813656663, -10.734747994957235, 8.238979976564913, -3.9547557691176518, -20.930173172392717, -12.250134288806471, -10.493336668881321, 0.33037465151399054, -7.1781483204203926, 2.7652345813450521, -2.5712673277723885, 0.0701810894690474, 7.6145730069985946, 10.659481131037447, -11.180204053312211, 17.39031889973452, 19.859868842666515, 11.022649968690514, -28.588502514131786, -16.551684767878022, -14.013144373713118, 9.7746877037997653, -3.6868017166653013, -13.514357458988556, -15.393758588817148, -10.476574181142841, 4.4326809184369393, -2.6196083809890349, 6.8486270886155403, -3.5613366171320475, -1.5851613092716301,-18.031342493225875 };
	float cs=0.0;
  float dot = 0.0, denom_a = 0.0, denom_b = 0.0 ;
  for(int i = 0; i < Vector_Length; i++) 
	{
		dot += A[i] * B[i] ;
    denom_a += A[i] * A[i] ;
    denom_b += B[i] * B[i] ;
  }
	cs=dot / (sqrt(denom_a) * sqrt(denom_b));
  return  cs;
}

/**
  * @brief  mfcc function
  * @param  None
  * @retval Audio error
  */
float *COMPUTING_MFCCs(float *wavdata)
{
	float *mfcc;
	int n=256;
	int frameLen=256,sampleRate=16000;
	preemphasize(wavdata,n);
	mfcc=FrametoMFCC(wavdata,frameLen,sampleRate);
	return mfcc;
}


/**
  * @brief  Manages Audio process. 
  * @param  None
  * @retval Audio error
  */
AUDIO_ErrorTypeDef AUDIO_PLAYER_Process(void)
{
  uint32_t bytesread, elapsed_time;
  AUDIO_ErrorTypeDef audio_error = AUDIO_ERROR_NONE;
  static uint32_t prev_elapsed_time = 0xFFFFFFFF;
  uint8_t str[10];  
  
  switch(AudioState)
  {
  case AUDIO_STATE_PLAY:
    if(BufferCtl.fptr >= WaveFormat.FileSize)
    {
      BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
      AudioState = AUDIO_STATE_NEXT;
    }
    
    if(BufferCtl.state == BUFFER_OFFSET_HALF)
    {
      if(f_read(&WavFile, 
                &BufferCtl.buff[0], 
                AUDIO_OUT_BUFFER_SIZE/2, 
                (void *)&bytesread) != FR_OK)
      { 
        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW); 
        return AUDIO_ERROR_IO;       
      } 
      BufferCtl.state = BUFFER_OFFSET_NONE;
      BufferCtl.fptr += bytesread; 
    }
    
    if(BufferCtl.state == BUFFER_OFFSET_FULL)
    {
      if(f_read(&WavFile, 
                &BufferCtl.buff[AUDIO_OUT_BUFFER_SIZE /2], 
                AUDIO_OUT_BUFFER_SIZE/2, 
                (void *)&bytesread) != FR_OK)
      { 
        BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW); 
        return AUDIO_ERROR_IO;       
      } 
 
      BufferCtl.state = BUFFER_OFFSET_NONE;
      BufferCtl.fptr += bytesread; 
    }
    
    /* Display elapsed time */
    elapsed_time = BufferCtl.fptr / WaveFormat.ByteRate; 
    if(prev_elapsed_time != elapsed_time)
    {
      prev_elapsed_time = elapsed_time;
      sprintf((char *)str, "[%02d:%02d]", (int)(elapsed_time /60), (int)(elapsed_time%60));
      BSP_LCD_SetTextColor(LCD_COLOR_CYAN); 
      //BSP_LCD_DisplayStringAt(263, LINE(8), str, LEFT_MODE);
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
    }

    /* Update audio state machine according to touch acquisition */
    AUDIO_AcquireTouchButtons();
    break;
    
  case AUDIO_STATE_STOP:
    BSP_LCD_SetTextColor(LCD_COLOR_RED);
    BSP_LCD_FillRect(TOUCH_VOL_PLUS_XMIN, TOUCH_VOL_PLUS_YMIN , /* VOl+ rectangle */
                   TOUCH_VOL_PLUS_XMAX - TOUCH_VOL_PLUS_XMIN,
                   TOUCH_VOL_PLUS_YMAX - TOUCH_VOL_PLUS_YMIN);
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    AudioState = AUDIO_STATE_IDLE; 
    audio_error = AUDIO_ERROR_IO;
    break;
    
  case AUDIO_STATE_NEXT:
    if(++FilePos >= AUDIO_GetWavObjectNumber())
    {
      FilePos = 0; 
    }
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    AUDIO_PLAYER_Start(FilePos);
    break;    
    
  case AUDIO_STATE_PREVIOUS:
    if(--FilePos < 0)
    {
      FilePos = AUDIO_GetWavObjectNumber() - 1; 
    }
    BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
    AUDIO_PLAYER_Start(FilePos);
    break;   
  
  case AUDIO_STATE_WAIT:
  case AUDIO_STATE_IDLE:
  case AUDIO_STATE_INIT:    
  default:
    /* Update audio state machine according to touch acquisition */
    AUDIO_AcquireTouchButtons();
    break;
  }
  return audio_error;
}

/**
  * @brief  Stops Audio streaming.
  * @param  None
  * @retval Audio error
  */
AUDIO_ErrorTypeDef AUDIO_PLAYER_Stop(void)
{
  AudioState = AUDIO_STATE_STOP;
  FilePos = 0;
  
  BSP_AUDIO_OUT_Stop(CODEC_PDWN_SW);
  f_close(&WavFile);
  return AUDIO_ERROR_NONE;
}

/**
  * @brief  Calculates the remaining file size and new position of the pointer.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_TransferComplete_CallBack(void)
{
  if(AudioState == AUDIO_STATE_PLAY)
  {
    BufferCtl.state = BUFFER_OFFSET_FULL;
  }
}

/**
  * @brief  Manages the DMA Half Transfer complete interrupt.
  * @param  None
  * @retval None
  */
void BSP_AUDIO_OUT_HalfTransfer_CallBack(void)
{ 
  if(AudioState == AUDIO_STATE_PLAY)
  {
    BufferCtl.state = BUFFER_OFFSET_HALF;
  }
}
/*******************************************************************************
                            Static Functions
*******************************************************************************/

/**
  * @brief  Gets the file info.
  * @param  file_idx: File index
  * @param  info: Pointer to WAV file info
  * @retval Audio error
  */
static AUDIO_ErrorTypeDef GetFileInfo(uint16_t file_idx, WAVE_FormatTypeDef *info)
{
  uint32_t bytesread;
  uint32_t duration;
  uint8_t str[FILEMGR_FILE_NAME_SIZE + 20];  
  
  if(f_open(&WavFile, (char *)FileList.file[file_idx].name, FA_OPEN_EXISTING | FA_READ) == FR_OK) 
  {
    /* Fill the buffer to Send */
    if(f_read(&WavFile, info, sizeof(WaveFormat), (void *)&bytesread) == FR_OK)
    {
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
      sprintf((char *)str, "Playing file (%d/%d): %s", 
              file_idx + 1, FileList.ptr,
              (char *)FileList.file[file_idx].name);
      BSP_LCD_ClearStringLine(4);
      //BSP_LCD_DisplayStringAtLine(4, str);
      
      BSP_LCD_SetTextColor(LCD_COLOR_CYAN); 
      sprintf((char *)str,  "Sample rate : %d Hz", (int)(info->SampleRate));
      BSP_LCD_ClearStringLine(6);
     // BSP_LCD_DisplayStringAtLine(6, str);
      
      sprintf((char *)str,  "Channels number : %d", info->NbrChannels);
      BSP_LCD_ClearStringLine(7);      
     // BSP_LCD_DisplayStringAtLine(7, str);
      
      duration = info->FileSize / info->ByteRate; 
      sprintf((char *)str, "File Size : %d KB [%02d:%02d]", (int)(info->FileSize/1024), (int)(duration/60), (int)(duration%60));
      BSP_LCD_ClearStringLine(8);
      //BSP_LCD_DisplayStringAtLine(8, str);
      //BSP_LCD_DisplayStringAt(263, LINE(8), (uint8_t *)"[00:00]", LEFT_MODE);
 
      BSP_LCD_SetTextColor(LCD_COLOR_WHITE); 
      sprintf((char *)str,  "Volume : %lu", uwVolume);
      BSP_LCD_ClearStringLine(16);      
     // BSP_LCD_DisplayStringAtLine(9, str);
      return AUDIO_ERROR_NONE;
    }
    f_close(&WavFile);
  }
  return AUDIO_ERROR_IO;
}

/**
  * @brief  Initializes the Wave player.
  * @param  AudioFreq: Audio sampling frequency
  * @retval None
  */
static uint8_t PlayerInit(uint32_t AudioFreq)
{ 
  /* Initialize the Audio codec and all related peripherals (I2S, I2C, IOExpander, IOs...) */  
  if(BSP_AUDIO_OUT_Init(OUTPUT_DEVICE_BOTH, uwVolume, AudioFreq) != 0)
  {
    return 1;
  }
  else
  {
    BSP_AUDIO_OUT_SetAudioFrameSlot(CODEC_AUDIOFRAME_SLOT_02);
    return 0;
  } 
}

/**
  * @brief  Display interface touch screen buttons
  * @param  None
  * @retval None
  */
static void AUDIO_PlaybackDisplayButtons(char *active,uint8_t people)
{
	uint8_t str[60];
  BSP_LCD_SetFont(&LCD_LOG_HEADER_FONT);
  BSP_LCD_ClearStringLine(14);            /* Clear dedicated zone */
  BSP_LCD_ClearStringLine(15);           
  BSP_LCD_ClearStringLine(16);
  BSP_LCD_ClearStringLine(17);
  BSP_LCD_ClearStringLine(18);
  BSP_LCD_ClearStringLine(19);
  BSP_LCD_ClearStringLine(20);

  BSP_LCD_SetTextColor(LCD_COLOR_BROWN);
  BSP_LCD_DrawRect(TOUCH_VOL_PLUS_XMIN, TOUCH_VOL_PLUS_YMIN , /* VOl+ rectangle */
                   TOUCH_VOL_PLUS_XMAX - TOUCH_VOL_PLUS_XMIN,
                   TOUCH_VOL_PLUS_YMAX - TOUCH_VOL_PLUS_YMIN);
  BSP_LCD_DisplayStringAt(626, LINE(18), (uint8_t *)"EXIT", LEFT_MODE);
	BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
	BSP_LCD_DisplayStringAtLine(3, (uint8_t *)"the result of audio analysis");
	//BSP_LCD_DisplayStringAtLine(4, (uint8_t *)"indoor people number: 3");
	sprintf((char *)str,  "indoor people number: %d", (int)people);   
  BSP_LCD_DisplayStringAtLine(4, str);
	//BSP_LCD_DisplayStringAtLine(5, (uint8_t *)"indoor state: meeting");
	sprintf((char *)str,  "indoor state:%s", active);   
  BSP_LCD_DisplayStringAtLine(5, str);
  BSP_LCD_SetTextColor(LCD_COLOR_GREEN);
  BSP_LCD_ClearStringLine(22);
  BSP_LCD_DisplayStringAtLine(22, (uint8_t *)"Use stop button to exit");
  BSP_LCD_SetTextColor(LCD_COLOR_CYAN);
  BSP_LCD_SetFont(&LCD_LOG_TEXT_FONT);
}

/**
  * @brief  Test touch screen state and modify audio state machine according to that
  * @param  None
  * @retval None
  */
static void AUDIO_AcquireTouchButtons(void)
{
  static TS_StateTypeDef  TS_State={0};

  if(TS_State.touchDetected == 1)   /* If previous touch has not been released, we don't proceed any touch command */
  {
    BSP_TS_GetState(&TS_State);
  }
  else
  {
    BSP_TS_GetState(&TS_State);
    if(TS_State.touchDetected == 1)
    {
      if ((TS_State.touchX[0] > TOUCH_VOL_PLUS_XMIN) && (TS_State.touchX[0] < TOUCH_VOL_PLUS_XMAX) &&
              (TS_State.touchY[0] > TOUCH_VOL_PLUS_YMIN) && (TS_State.touchY[0] < TOUCH_VOL_PLUS_YMAX))
      {
        AudioState = AUDIO_STATE_STOP;
      }
      
    }
  }
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
