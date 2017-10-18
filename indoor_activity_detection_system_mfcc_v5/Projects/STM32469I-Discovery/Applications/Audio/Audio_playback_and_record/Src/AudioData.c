#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "AudioData.h"
#include "waveplayer.h"

static AUDIO_OUT_BufferTypeDef  BufferCtl;
static __IO uint32_t uwVolume = 70;



extern FILELIST_FileTypeDef FileList;
FIL WAV;

/*
 * input:
	 FileNameLWAV's file name
 * return :
	fileHeader:WAV's fileHeader will fill this pointer
 * author: tbc.dengwenqi@gmail.com
   all right reserved
*/
WavFileHeaderStruct* openWAVFile(int file_idx)
{
	//FILE* WAVFile;
	uint32_t bytesread = 0;
	
	f_close(&WAV);
	WAVE_FormatTypeDef *info;
	char ChunkID[4];
	WavFileHeaderStruct *fileHeader= calloc(1, sizeof(WavFileHeaderStruct));
	

	f_open(&WAV, (char *)FileList.file[1].name, FA_OPEN_EXISTING | FA_READ);
	//if(&WAVFile == NULL)
	//{
//		fprintf(stderr, "error：%s,%s\n", __FILE__, __LINE__);
		//free(fileHeader);
		//return NULL;	
	//}
	//RIFF Chunk
	//fread(&fileHeader->RIFFChunk,sizeof(fileHeader->RIFFChunk),1,WAVFile);
	f_read(&WAV, &fileHeader->RIFFChunk, sizeof(RIFFRStruct), (void *)&bytesread);
	//fprintf(stdout, "%d\n", ret);

	if (strncmp(fileHeader->RIFFChunk.ChunkID,"RIFF",4) ||
		strncmp(fileHeader->RIFFChunk.Type,"WAVE",4))
	{
		fprintf(stderr, "9error：%s,%d\n", __FILE__, __LINE__);
		free(fileHeader);
		f_close(&WAV);
		return NULL;
	}
	//format chunk 
	f_read(&WAV,&fileHeader->FormatChunk,sizeof(fileHeader->FormatChunk),(void *)&bytesread);
	if (strncmp(fileHeader->FormatChunk.ChunkID, "fmt", 3))
	{
		fprintf(stderr, "error：%s,%d\n", __FILE__, __LINE__);
		free(fileHeader);
		f_close(&WAV);
		return NULL;
	}
	if(fileHeader->FormatChunk.ChunkSize == 18)
	{
		//skip Additional Information
		f_lseek(&WAV,2L);
	}
	//fact chunk,optional
	f_read(&WAV,ChunkID,4,(void *)&bytesread);
	//whether comtain fact chunk
	if(!strncmp(ChunkID,"fact",4)) 
	{
		
		f_read(&WAV,&fileHeader->FactChunk.ChunkSize,sizeof(fileHeader->FactChunk.ChunkSize),(void *)&bytesread);
		f_read(&WAV,&fileHeader->FactChunk.Data,sizeof(fileHeader->FactChunk.Data),(void *)&bytesread);
		f_read(&WAV,fileHeader->DataChunk.ChunkID,4,(void *)&bytesread);	
		ChunkID[0] = 'n';
	}
	else
	{
		fileHeader->FactChunk.ChunkSize = 4;
		fileHeader->FactChunk.Data = 0;
		strncpy(fileHeader->DataChunk.ChunkID, ChunkID, 4); 
	}
	if(strncmp(fileHeader->DataChunk.ChunkID,"data",4))
	{
		fprintf(stderr, "11error：%s,%d\n", __FILE__, __LINE__);
		free(fileHeader);
		f_close(&WAV);
		return NULL;
	}
	f_read(&WAV,&fileHeader->DataChunk.ChunkSize,sizeof(fileHeader->DataChunk.ChunkSize),(void *)&bytesread);	
	fileHeader->DataChunk.data = (uint8_t *)calloc(fileHeader->DataChunk.ChunkSize,sizeof(uint8_t));
	f_read(&WAV,fileHeader->DataChunk.data,fileHeader->DataChunk.ChunkSize,(void *)&bytesread);
	
	f_close(&WAV);
	return fileHeader;
}

int closeWAVFIle(WavFileHeaderStruct * fileHeader)
{
	if (!fileHeader)
	{
	//	fprintf(stderr, "error：%s,%s\n", __FILE__, __LINE__);
		return -1;
	}
	if (!fileHeader->DataChunk.data)
		free(fileHeader->DataChunk.data);
	free(fileHeader);
	return 0;
}
/*
 *  
 */
float *readAllWAVData(WavFileHeaderStruct *fileHeader,int *n)
{
	int16_t *dataInt16;
	int16_t max;
	int i;
	float *data = NULL;

	if (!fileHeader || !fileHeader->DataChunk.data)
	{
		fprintf(stderr, "33error：%s,%d\n", __FILE__, __LINE__);
		return NULL;
	}

	*n =  fileHeader->DataChunk.ChunkSize / (fileHeader->FormatChunk.BitsPerSample / 8);
	if (!n)
	{
		
	//	fprintf(stderr, "error：%s,%s\n", __FILE__, __LINE__);
		return NULL;
	}
	data = calloc(*n, sizeof(float));
	if (!data) return NULL;

	if (fileHeader->FormatChunk.NumChannels == 2)
	{
		//双声道，暂不处理
	//	fprintf(stderr, "error：%s,%s\n", __FILE__, __LINE__);
		return NULL;
	}
	if (fileHeader->FormatChunk.BitsPerSample == 8)
	{
		max = fileHeader->DataChunk.data[0];
		for (i = 1; i < *n; ++i)
			if (fileHeader->DataChunk.data[i] > max)
				max = fileHeader->DataChunk.data[i];
		if (!max) max = 1;
		for (i = 0; i < *n; ++i)
			data[i] = fileHeader->DataChunk.data[i] / (float)max;
	}
	else if (fileHeader->FormatChunk.BitsPerSample == 16)
	{
		dataInt16 = (int16_t*)fileHeader->DataChunk.data;
		max = dataInt16[0];
		for (i = 1; i < *n; ++i)
			if(dataInt16[i] > max)
				max = dataInt16[i];
		if (!max) max = 1;
		for (i = 0; i < *n; ++i)
			data[i] = dataInt16[i] / (float)max;
	}
	return data;
}
