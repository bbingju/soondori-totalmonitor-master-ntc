#include "0_Util.h"
#include "0_StartSlotUartTask.h"
#include "external_uart_task.h"
#include <string.h>

uint8_t *bytes;
uint8_t BuzzerEnable;

/*********************************************************************
*	 ByteArrayToFloat
*  String 를 배열로 복사 한다. 복사한 뒤 남은 배열은 0으로 초기화 한다.
*  byteArray			: 원본 배열을 받는다. 4바이트 float
*	 return	                : float 로 변환하여 반환 한다.
**********************************************************************/
float ByteArrayToFloat(uint8_t *byteArray)
{
    bytes = byteArray;
    uint32_t res = 0.0;

    res  =  ((uint32_t)*byteArray
             | ((uint32_t)*(byteArray + 1) << 8)
             | ((uint32_t)*(byteArray + 2) << 16)
             | ((uint32_t)*(byteArray + 3) << 24));

    return *((float*)&res);
}

/*********************************************************************
*  StringCopyToArray
*  String 를 배열로 복사 한다. 복사한 뒤 남은 배열은 0으로 초기화 한다.
*  TagetArray			: 이곳으로 복사한다.
*  OriginalString : 이것을 복사한다.
*  CopyLength     : 복사할 길이
*  TotalLength    : 타겟 배열의 총 길이
**********************************************************************/

uint8_t CopyToArray(uint8_t* TagetArray, uint8_t* OriginalString,
                    uint16_t CopyLength, uint16_t TotalLength)
{
    for (int i = 0; i < TotalLength; i++)
    {
        if (i < CopyLength)
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
*	명령을 받앗ㅅ을때 리턴해주는 문자열 생성 함수
*	SendData		: 이곳으로 문자열을 생성한다.
*	Command                 : 1번 바이트의 command 의 바이트를 받는다. (0번 바이트부터 시작)
*	Data                    : 2번부터 기록될 문자열을 받는다.
*	DataLength		: Data 의 길이를 받는다.
*	BufferLength	: 전체 SendData의 길이를 받는다. etx의 위치를 확인
**********************************************************************/
#if 0
void doMakeSendData(	uint8_t* SendData,
                        uint8_t Command, uint8_t* Data,
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

void doMakeSendSlotData(uint8_t* SendData, uint8_t SlotNumber,
                        uint8_t Command, uint8_t* Data,
                        uint8_t  DataLength, uint8_t BufferLength)
{
    uni2Byte crc;

    *SendData++ = CMD_STX;
    *SendData++ = SlotNumber;
    *SendData++ = Command;
    CopyToArray(SendData, Data, DataLength, DATA_FULL_LENGTH);
    SendData -= 2;

    crc.UI16 = CRC16_Make(&SendData[0], BufferLength - 4);
    SendData += BufferLength - 4;

    *SendData++ = crc.UI8[0];
    *SendData++ = crc.UI8[1];

    *SendData = CMD_ETX;
}


void doMakeSend485Data(uint8_t* SendData, uint8_t  Command, uint8_t  Option,
                       uint8_t* Data, uint16_t DataWriteLen, uint16_t DataLen,
                       uint16_t BufferLen)
{
    uni2Byte crc;
    uni4Byte len;

    len.UI32 = 0;
    len.UI16[0] = BufferLen;

    memset(SendData, 0x00, BufferLen);

    /* HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN); */
    /* HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN); */

    *SendData++ = RS_STX;
    *SendData++ = Command;
    *SendData++ = Option;
    *SendData++ = len.UI8[0];
    *SendData++ = len.UI8[1];
    *SendData++ = len.UI8[2];
    *SendData++ = len.UI8[3];
    *SendData++ = 192;                                      //ip
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

    crc.UI16 = CRC16_Make(&SendData[0], BufferLen - 4);
    SendData += BufferLen - 4;

    *SendData++ = crc.UI8[0];
    *SendData++ = crc.UI8[1];

    *SendData = RS_ETX;
}

void doMakeSend485DataDownLoad(uint8_t* SendData, uint8_t Command, uint8_t Option,
                               uint8_t* Data, uint16_t DataWriteLen, uint16_t DataLen,
                               uint16_t BufferLen)
{
        uni2Byte crc;
        uni4Byte len;

        len.UI32 = 0;
        len.UI16[0] = BufferLen;

        memset(SendData, 0x00, BufferLen);

        /* HAL_RTC_GetTime(&hrtc, &SysTime.Time, RTC_FORMAT_BIN); */
        /* HAL_RTC_GetDate(&hrtc, &SysTime.Date, RTC_FORMAT_BIN); */

        SendData[ 0] = RS_STX;
        SendData[ 1] = Command;
        SendData[ 2] = Option;
        SendData[ 3] = len.UI8[0];
        SendData[ 4] = len.UI8[1];
        SendData[ 5] = len.UI8[2];
        SendData[ 6] = len.UI8[3];
        SendData[ 7] = 192;                                     //ip
        SendData[ 8] = 168;
        SendData[ 9] = 255;
        SendData[10] = 255;
        SendData[11] = SysTime.Date.Year;        //time
        SendData[12] = SysTime.Date.Month;
        SendData[13] = SysTime.Date.Date;
        SendData[14] = SysTime.Time.Hours;
        SendData[15] = SysTime.Time.Minutes;
        SendData[16] = SysTime.Time.Seconds;
        memcpy(&SendData[17], Data, DataWriteLen);
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
*       한 루프에 100ms가 소요 되는 부저 파형을 발생 한다.
*	발생 주파수는 약 1kHz 이다.
*       count	 : 100ms 단위로 몇번을 반속 할지를 받는다.
*	ex) count = 10 일ㄷ대 1000ms 의 부저음이 발생 한다.
**********************************************************************/
void doPlayBuzzer100ms(uint16_t count)
{
    if (BuzzerEnable == ENABLE) {
        for (int msec = 0; msec < count; msec++) {
            //여기서 부터 한 루팅에 100ms 소요 된다.
            // BUZZER_OFF;
            // HAL_GPIO_WritePin(Debug_out_GPIO_Port, Debug_out_Pin, GPIO_PIN_SET);
            for (int i = 0; i < 105; i++) {
                BUZZER_ON;
                // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
                // HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
                for (int j = 0; j < 100 * 100; j++) {
                    __NOP();
                }
                BUZZER_OFF;
                // HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
                // HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
                for (int j = 0; j < 100 * 100; j++) {
                    __NOP();
                }
            }
        }
    }
}

/*********************************************************************
*	doBuzzerEnable
*       부저 알림을 강제 정지 시키기 위해 플레그 설정
*       enable	 : ENABLE / DISABLE 를 받아서 플래그를 지정 한다.
**********************************************************************/
void doBuzzerEnable(uint8_t enable)
{
        BuzzerEnable = enable;
}

/*********************************************************************
*	doBuzzerEnable
*       부저 알림
*       delay	 : msec 단위로 부저 동작
**********************************************************************/
void doBuzzerPlay(uint16_t delay)
{
        HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_SET);
        osDelay(delay);
        HAL_GPIO_WritePin(BUZZER_SEL_GPIO_Port, BUZZER_SEL_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
*	doNOP
*       NOP 명령 반복 루틴
*       count	 : 16비트 카운트
**********************************************************************/
void doNOP(uint16_t count)
{
    for (int i = 0; i < count; i++)
        asm("NOP");
}

// CRC16
uint16_t CRC16_Make(uint8_t *byMsg, uint16_t len)
{
    uint16_t  crc = 0xFFFF;

        for (int i = 0; i < len; i++)
        {
                crc ^= byMsg[i];
                for (int j = 0; j < 8; j++)
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
        float	sortData[11];

        sortData[0]     = *(inData++);
        sortData[1]     = *(inData++);
        sortData[2]     = *(inData++);
        sortData[3]     = *(inData++);
        sortData[4]     = *(inData++);
        sortData[5]     = *(inData++);
        sortData[6]     = *(inData++);
        sortData[7]     = *(inData++);
        sortData[8]     = *(inData++);
        sortData[9]     = *(inData++);
        sortData[10]	= *(inData);

        for(int i = 0; i < 11 - 1; ++i)
        {
                for(int j = 11 - 1; i < j; --j)
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
/* void util_mem_cpy (void* dst, const void* src, UINT cnt)  */
/* { */
/*      BYTE *d = (BYTE*)dst; */
/*      const BYTE *s = (const BYTE*)src; */

/*      if (cnt) { */
/*              do { */
/*                      *d++ = *s++; */
/*              } while (--cnt); */
/*      } */
/* } */

/* Fill memory block */
/* void util_mem_set (void* dst, int val, UINT cnt)  */
/* { */
/*      BYTE *d = (BYTE*)dst; */

/*      do { */
/*              *d++ = (BYTE)val; */
/*      } while (--cnt); */
/* } */
