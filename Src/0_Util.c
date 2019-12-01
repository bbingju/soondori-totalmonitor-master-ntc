#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "0_StartRs485Task.h"
#include "string.h"           //memset �� include

uint8_t *bytes;
uint8_t BuzzerEnable;

/*********************************************************************
*	 ByteArrayToFloat
*  String �� �迭�� ���� �Ѵ�. ������ �� ���� �迭�� 0���� �ʱ�ȭ �Ѵ�.
*  byteArray			: ���� �迭�� �޴´�. 4����Ʈ float
*	 return	    		: float �� ��ȯ�Ͽ� ��ȯ �Ѵ�. 
**********************************************************************/
float ByteArrayToFloat(uint8_t *byteArray)
{
//	uint8_t data[4] = {0, 0, 0, 0};
	bytes = byteArray;
	uint32_t res = 0.0;

	res  =  ((uint32_t)*byteArray 
				| ((uint32_t)*(byteArray + 1) << 8) 
				| ((uint32_t)*(byteArray + 2) << 16) 
				| ((uint32_t)*(byteArray + 3) << 24));

	return	*((float*)&res);
}

/*********************************************************************
*	 StringCopyToArray
*  String �� �迭�� ���� �Ѵ�. ������ �� ���� �迭�� 0���� �ʱ�ȭ �Ѵ�.
*  TagetArray			: �̰����� �����Ѵ�.
*  OriginalString : �̰��� �����Ѵ�.
*  CopyLength     : ������ ����
*	 TotalLength    : Ÿ�� �迭�� �� ���� 
**********************************************************************/
uint8_t CopyToArray(	uint8_t* TagetArray, 	uint8_t* OriginalString, 
							uint16_t CopyLength, 	uint16_t TotalLength)
{
	uint16_t i = 0; 

	for(i = 0; i < TotalLength; i++)
	{
		if(i < CopyLength)
		{
			*TagetArray++ = *OriginalString++;
		}
		else
		{
			*TagetArray++ = 0;
		}
	}

	return TRUE;
}

/*********************************************************************
*	doMakeSendData
*	����� �޾Ѥ����� �������ִ� ���ڿ� ���� �Լ� 
*	SendData		: �̰����� ���ڿ��� �����Ѵ�.
*	Command 		: 1�� ����Ʈ�� command �� ����Ʈ�� �޴´�. (0�� ����Ʈ���� ����)
*	Data    		: 2������ ��ϵ� ���ڿ��� �޴´�.
*	DataLength		: Data �� ���̸� �޴´�.
*	BufferLength	: ��ü SendData�� ���̸� �޴´�. etx�� ��ġ�� Ȯ�� 
**********************************************************************/
#if 0
void doMakeSendData(	uint8_t* SendData,	    
                            uint8_t Command, 		uint8_t* Data, 			
							uint8_t  DataLength,	uint8_t BufferLength)
{
	//uint8_t i;
	uni2Byte crc;
	
	*SendData++ = CMD_STX;
	*SendData++ = sendSlotNumber + '0';
	*SendData++ = Command;
	CopyToArray(SendData, Data, DataLength, DATA_FULL_LENGTH);	
	*SendData -= 3;

	crc.UI16 =	CRC16_Make(&SendData[0], DATA_FULL_LENGTH + 2);
	SendData += DATA_FULL_LENGTH + 3;

	*SendData++ = crc.UI8[0];
	*SendData++ = crc.UI8[1];
	
	*SendData = CMD_ETX;
}
#endif 

void doMakeSendSlotData(	uint8_t* SendData,	    uint8_t SlotNumber,
                                uint8_t Command, 		uint8_t* Data, 			
							    uint8_t  DataLength,	uint8_t BufferLength)
{
	uni2Byte crc;
	
	*SendData++ = CMD_STX;
	*SendData++ = SlotNumber;
	*SendData++ = Command;
	CopyToArray(SendData, Data, DataLength, DATA_FULL_LENGTH);	
	SendData -= 2;

	crc.UI16 =	CRC16_Make(&SendData[0], BufferLength - 4);
	SendData += BufferLength - 4;

	*SendData++ = crc.UI8[0];
	*SendData++ = crc.UI8[1];
	
	*SendData = CMD_ETX;
}

void doMakeSend485Data(	uint8_t* SendData,	    uint8_t  Command,
                                uint8_t  Option, 		uint8_t* Data, 			
							    uint16_t DataWriteLen,	uint16_t DataLen,
							    uint16_t BufferLen)
{
	uni2Byte crc;
	uni4Byte len;

	len.UI32 = 0;
	len.UI16[0] = BufferLen;

	memset((void*)SendData,0x00, BufferLen);
	
    HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
  
	*SendData++ = RS_STX;
	*SendData++ = Command;
	*SendData++ = Option;
	*SendData++ = len.UI8[0];
	*SendData++ = len.UI8[1];
	*SendData++ = len.UI8[2];
	*SendData++ = len.UI8[3];
	*SendData++ = 192;      				//ip
	*SendData++ = 168;
	*SendData++ = 255;
	*SendData++ = 255;
	*SendData++ = SysTime.Date.Year;        //time
	*SendData++ = SysTime.Date.Month;
	*SendData++ = SysTime.Date.Date;
	*SendData++ = SysTime.Time.Hours;
	*SendData++ = SysTime.Time.Minutes;
	*SendData++ = SysTime.Time.Seconds;
	//util_mem_cpy(&SendData[17], Data, DataWriteLen);
	CopyToArray(SendData, Data, DataWriteLen, DataLen);	
    SendData -= 16;

	crc.UI16 =	CRC16_Make(&SendData[0], BufferLen - 4);
	SendData += BufferLen - 4;

	*SendData++ = crc.UI8[0];
	*SendData++ = crc.UI8[1];
	
	*SendData = RS_ETX;
}

void doMakeSend485DataDownLoad(	uint8_t* SendData,	    uint8_t  Command,
                                			uint8_t  Option, 		uint8_t* Data, 			
							    			uint16_t DataWriteLen,	uint16_t DataLen,
							    			uint16_t BufferLen)
{
	uni2Byte crc;
	uni4Byte len;

	len.UI32 = 0;
	len.UI16[0] = BufferLen;

	memset((void*)SendData,0x00, sizeof(tx485DataDMA));
	
    HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN);
  
	SendData[ 0] = RS_STX;
	SendData[ 1] = Command;
	SendData[ 2] = Option;
	SendData[ 3] = len.UI8[0];
	SendData[ 4] = len.UI8[1];
	SendData[ 5] = len.UI8[2];
	SendData[ 6] = len.UI8[3];
	SendData[ 7] = 192;      				//ip
	SendData[ 8] = 168;
	SendData[ 9] = 255;
	SendData[10] = 255;
	SendData[11] = SysTime.Date.Year;        //time
	SendData[12] = SysTime.Date.Month;
	SendData[13] = SysTime.Date.Date;
	SendData[14] = SysTime.Time.Hours;
	SendData[15] = SysTime.Time.Minutes;
	SendData[16] = SysTime.Time.Seconds;
	util_mem_cpy(&SendData[17], Data, DataWriteLen);
	//CopyToArray(SendData, Data, DataWriteLen, DataLen);	
    //SendData -= 16;

	crc.UI16 =	CRC16_Make(&SendData[1], BufferLen - 4);
	//SendData += BufferLen - 4;

	SendData[BufferLen - 3] = crc.UI8[0];
	SendData[BufferLen - 2] = crc.UI8[1];
	
	SendData[BufferLen - 1] = RS_ETX;
}

/*********************************************************************
*	doPlayBuzzer100ms
*  	�� ������ 100ms�� �ҿ� �Ǵ� ���� ������ �߻� �Ѵ�.
*	�߻� ���ļ��� �� 1kHz �̴�. 
*  	count	 : 100ms ������ ����� �ݼ� ������ �޴´�. 
*	ex) count = 10 �Ϥ��� 1000ms �� �������� �߻� �Ѵ�. 
**********************************************************************/
void doPlayBuzzer100ms(uint16_t count)
{
	uint16_t i, j, k, msec;

	if(BuzzerEnable == ENABLE)
	{
		for(msec = 0; msec < count; msec++)
		{
			//���⼭ ���� �� ���ÿ� 100ms �ҿ� �ȴ�.
			//BUZZER_OFF;
			//HAL_GPIO_WritePin(Debug_out_GPIO_Port, Debug_out_Pin, GPIO_PIN_SET);
			for(i = 0; i < 105; i++)
			{
				BUZZER_ON;
				//HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
				//HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
				for(j = 0; j < 100; j++)
				{
					for(k = 0; k < 100; k++){	asm("NOP");	}
				}
				BUZZER_OFF;
				//HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
				//HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
				for(j = 0; j < 100; j++)
				{
					for(k = 0; k < 100; k++){	asm("NOP");	}
				}
			}
		}
	}
}

/*********************************************************************
*	doBuzzerEnable
*  	���� �˸��� ���� ���� ��Ű�� ���� �÷��� ���� 
*  	enable	 : ENABLE / DISABLE �� �޾Ƽ� �÷��׸� ���� �Ѵ�. 
**********************************************************************/
void doBuzzerEnable(uint8_t enable)
{
	BuzzerEnable = enable;
}

/*********************************************************************
*	doBuzzerEnable
*  	���� �˸�
*  	delay	 : msec ������ ���� ���� 
**********************************************************************/
void doBuzzerPlay(uint16_t delay)
{
	HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_SET);
	osDelay(delay);
	HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
*	doNOP
*  	NOP ��� �ݺ� ��ƾ
*  	count	 : 16��Ʈ ī��Ʈ 
**********************************************************************/
void doNOP(uint16_t count)
{
	uint16_t i;
	for(i = 0; i < count; i++)
		asm("NOP");
}

// CRC16 
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len)
{
	uint16_t	crc = 0xFFFF;
    uint16_t	i;
    uint16_t	j;
    
	for (i = 0; i < len; i++)
	{
		crc ^= byMsg[i];
		for (j = 0; j < 8; j++)
		{
			uint16_t flag = (uint16_t)(crc & 0x0001);
			crc >>= 1;
			if (flag > 0) crc ^= 0xA001;
		}
	}
	return crc;
}

void swap(float *a, float *b) 
{ 
	float tmp = *a; 
	*a = *b; 
	*b = tmp; 
}


float midADC_float(float * inData) /* using Bubble sort */ 
{
	uint8_t 	i, j; 
	float	sortData[11];

	sortData[0] 	= *(inData++);
	sortData[1] 	= *(inData++);
	sortData[2] 	= *(inData++);
	sortData[3] 	= *(inData++);
	sortData[4] 	= *(inData++);
	sortData[5] 	= *(inData++);
	sortData[6] 	= *(inData++);
	sortData[7] 	= *(inData++);
	sortData[8] 	= *(inData++);
	sortData[9] 	= *(inData++);
	sortData[10]	= *(inData);

	for(i = 0; i < 11 - 1; ++i) 
	{ 
		for(j = 11 - 1; i < j; --j) 
		{
			if(sortData[j - 1] > sortData[j]) 
				swap(&sortData[j - 1], &sortData[j]); 
		} 
	} 
	return sortData[6]; 
}

/*-----------------------------------------------------------------------*/
/* String functions                                                      */
/*-----------------------------------------------------------------------*/

/* Copy memory to memory */
void util_mem_cpy (void* dst, const void* src, UINT cnt) 
{
	BYTE *d = (BYTE*)dst;
	const BYTE *s = (const BYTE*)src;

	if (cnt) {
		do {
			*d++ = *s++;
		} while (--cnt);
	}
}

/* Fill memory block */
void util_mem_set (void* dst, int val, UINT cnt) 
{
	BYTE *d = (BYTE*)dst;

	do {
		*d++ = (BYTE)val;
	} while (--cnt);
}

