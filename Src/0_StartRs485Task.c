#include "0_StartRs485Task.h"
#include "0_soonQueue.h"
#include "0_StartSlotUartTask.h"
#include "0_soonFlashMemory.h"
#include "0_SdCard.h"

/*********************************************************************
*	Private variables
**********************************************************************/

// �Լ� ���η� ������ �ű�� Stack�� �����ϸ鼭 ������ �߻��Ͽ�
// ũ�⸦ ��ġ�ϴ� ��� Buffer type�� ������ ������ ���������� �����Ͽ���.


uint8_t         rx485DataDMA[320] = { 0 };      //dma �뵵
uint8_t         Rx485Data[40] = { 0 };          //parsing �뵵

uint8_t         tx485DataDMA[MAX_485_BUF_LEN] = { 0 };       //dma �뵵

uint8_t         Rx485ReadCount = 0;
uint8_t         noReturn485SendCt = 0;

uint8_t 		ReadFileBuf[MAX_485_BUF_LEN];

//uint8_t FunctionHandlingFlag = READY;

RS_485_RX_QUEUE_STRUCT Rs485RxQueue;
RS_485_TX_QUEUE_STRUCT Rs485TxQueue;
//SD_CARD_STRUCT SdCard;
//TEST_STATE_STRUCT TestState;
uint8_t			FindFilelistFlag = 0;		//0 : ���� ����Ʈ �˻� ���ϴ��� / 1 : ���� ����Ʈ �˻���


void SendUart485String(uint8_t *data, uint16_t length)
{
	if(CountingSem485TxHandle != NULL)  //TX
	{
		if(osSemaphoreWait(CountingSem485TxHandle, 2000) == osOK)
		{
		    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
		    osDelay(1);
			HAL_UART_Transmit_DMA(&UART_RS485_HANDEL, data, length);
//		    osDelay(7);
		}
	}
}

void SendUart485NonDma(uint8_t *data, uint16_t length)
{
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_SET);
    osDelay(1);
	HAL_UART_Transmit(&UART_RS485_HANDEL, data, length, 100);
    osDelay(100);
    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
}

/*********************************************************************
*	Bluetooth Task
*	������� ����� ���� ��� �ʱ�ȭ
*	������� ������� ������ ��ɵ��� �Ľ�
**********************************************************************/
void StartRs485Task(void const * argument)
{
	/* USER CODE BEGIN StartBluetooehTask */
    //uint8_t     i;

    HAL_GPIO_WritePin(RS485_EN_GPIO_Port, RS485_EN_Pin, GPIO_PIN_RESET);
	Rs485RTxQueue_Init(&Rs485RxQueue);			//��������� rx ����, ��Ŭ��ť ����
    HAL_UART_Receive_DMA(&huart1, rx485DataDMA, 32);
	Rs485RxQueue_Clear(&Rs485RxQueue);

    //Task ���� �Ϸ� �÷���
    SysProperties.bootingWate[1] = TRUE;

    while(1)
    {
        if( (SysProperties.bootingWate[0] == TRUE) &&   // 0 : StartDiaplayTask,
            (SysProperties.bootingWate[1] == TRUE) &&   // 1 : StartRs485Task,
            (SysProperties.bootingWate[2] == TRUE) &&   // 2 : StartSlotUartTask,
            (SysProperties.bootingWate[3] == TRUE) )    // 3 : StartRateTask
        {
            break;
        }
        osDelay(100);
    }

    /* Infinite loop */
    for(;;)
    {
        HAL_UART_Receive_DMA(&huart1, rx485DataDMA, 32);
        osDelay(1);

        //RX FUNCTION
        RxFunction();
		UnpackingRs485RxQueue();

        if(Rx485ReadCount == 31)
        {
            JumpToFunction485();
            if(osSemaphoreGetCount(CountingSem485TxHandle) == 0)
            {
                osSemaphoreRelease(CountingSem485TxHandle);
            }
            Rx485ReadCount = 0;
        }

        osDelay(30);
    }
    /* USER CODE END StartBluetooehTask */
}

void RxFunction(void)
{
    uint8_t i;

	if(BinarySem485RxHandle != NULL)  //RX
	{
		if(osSemaphoreWait(BinarySem485RxHandle, 0) == osOK)
		{
			for(i = 0; i < huart1.RxXferSize; i++)
			{
				osDelay(1); 	  // nop �� ����� HardFault �߻� ��. osDelayUntil(&xLastWakeTime, 180); �� ���� ��� �ؾ� ��.
				//doNOP(25000);
				Rs485RxQueue_Send(&Rs485RxQueue, rx485DataDMA[i]);
			}
			noReturn485SendCt = 0;
		}
	}
}

void UnpackingRs485RxQueue(void)
{
    uint16_t    etxLength0 = 0;
	uint8_t 	temp;			//�̰� Warning ���͵� ������ ����.

	if(Rs485RxQueue_empty(&Rs485RxQueue) == FALSE)	//Rx ���ۿ� �Է� �����Ͱ� �մ��� Ȯ��
	{

		if((Rs485RxQueue.tail + 31) >= RS_485_RX_BUF_MAX)  {   etxLength0 = Rs485RxQueue.tail + 31 - RS_485_RX_BUF_MAX;   }
		else											   {   etxLength0 = Rs485RxQueue.tail + 31; 					}

		if(Rs485RxQueue_Count(&Rs485RxQueue) > 31)// || (RxQueue_Count(&RxQueue) >= 134))
		{
			if( (Rs485RxQueue.ar[Rs485RxQueue.tail] == RS_STX) && (Rs485RxQueue.ar[etxLength0] == RS_ETX))
			{
				Rx485ReadCount = 0;
				Rx485Data[0] = Rs485RxQueue_Recive(&Rs485RxQueue);
				if(Rx485Data[0] == RS_STX)
				{
					do{
						Rx485ReadCount++;
						Rx485Data[Rx485ReadCount] = Rs485RxQueue_Recive(&Rs485RxQueue);
						if(Rs485RxQueue_empty(&Rs485RxQueue)) break;
					}while(Rx485Data[Rx485ReadCount] != RS_ETX);
				}
			}
			else
			{
				while(	!Rs485RxQueue_empty(&Rs485RxQueue)	&& (Rs485RxQueue.ar[Rs485RxQueue.tail] != RS_STX)
					&& (Rs485RxQueue.ar[etxLength0] != RS_ETX) )
				{
					temp = Rs485RxQueue_Recive(&Rs485RxQueue);
				}
				if(osSemaphoreGetCount(CountingSem485TxHandle) == 0)
				{
					osSemaphoreRelease(CountingSem485TxHandle);
				}
			}
		}
		else
		{
			Rs485RxQueue_Clear(&Rs485RxQueue);
		}
	}
}

/*********************************************************************
*	JumpToFunction
* 	�� ��ɺ��� �б� ���Ѽ� �Լ��� �и� ��.
**********************************************************************/
void JumpToFunction485(void)
{
	//uint8_t i;

	//SysProperties.FunctionHandlingFlag = BUSY;

    if(Rx485Data[0] == RS_STX)
    {
        if(Rx485Data[31] == RS_ETX)
        {
        	switch(Rx485Data[1])
        	{
        		case CMD_TEMP_TEST:
        			switch(Rx485Data[2])
                    {
                        case OP_TEMP_START_RX:
                            SysProperties.start_flag = TRUE;
                            break;
                        case OP_TEMP_STOP:
                            SysProperties.start_flag = FALSE;
                            break;
                        case OP_TEMP_SAMPLE_RATE:
                            doSaveIntervalTime();
                            break;
                    }
                    break;
                case CMD_WARNING_TEMP:
        			switch(Rx485Data[2])
                    {
                        case OP_WARNING_TEMP_SET:
							CmdWarningTempSet();
                            break;
                        case OP_WARNING_TEMP_REQ:
							CmdWarningTempReq();
                            break;
                    }
                    break;
                case CMD_REVISION:
        			switch(Rx485Data[2])
        			{
        				case OP_REVISION_APPLY_SET:
							CmdRevisionApplySet();
        					break;
						case OP_REVISION_CONSTANT_SET:
							CmdRevisionConstantSet();
							break;
						case OP_REVISION_APPLY_REQ:
							CmdRevisionApplyReq();
							break;
						case OP_REVISION_CONSTANT_REQ:
							CmdRevisionConstantReq();
							break;
        			}
                    break;
				case CMD_CALIBRATION:
        			switch(Rx485Data[2])
        			{
        				case OP_CALIBRATION_RTD_CONSTANT_SET:
							CmdCalibrationRTDConstSet();
        					break;
        				case OP_CALIBRATION_NTC_CON_TABLE_CAL:
							CmdCalibrationNTCTableCal();
        					break;
        				case OP_CALIBRATION_NTC_CONSTANT_SET:
							CmdCalibrationNTCConstantSet();
        					break;
        				case OP_CALIBRATION_RTD_CONSTANT_REQ:
							CmdCalibrationRTDConstReq();
        					break;
        				case OP_CALIBRATION_NTC_CON_TABLE_REQ:
							CmdCalibrationNTCTableReq();
        					break;
        				case OP_CALIBRATION_NTC_CONSTANT_REQ:
							CmdCalibrationNTCConstantReq();
        					break;
					}
                    break;
                case CMD_SD_CARD:
        			switch(Rx485Data[2])
        			{
        				case OP_SDCARD_LIST:
							FindFilelistFlag = 1;
							DoReadFileList();
							FindFilelistFlag = 0;
        					break;
        				case OP_SDCARD_DOWNLOAD:
							DoSendFile();
        					break;
        				case OP_SDCARD_DELETE:
        					break;
        				case OP_SDCARD_FORMAT:
        					break;
        				case OP_SDCARD_ERROR:
        					break;
        			}
                    break;
                case CMD_SLOT:
        			switch(Rx485Data[2])
                    {
                        case OP_SLOT_SET:
                            break;
                        case OP_SLOT_REQ:
                            break;
                    }
                    break;
                case CMD_TIME:
        			switch(Rx485Data[2])
                    {
                        case OP_TIME_SET:
                            doSetTime();
                            break;
                        case OP_TIME_REQ:
                            doGetTime();
                            break;
                    }
                    break;
        	}
        }
    }
    HAL_UART_Receive_DMA(&UART_RS485_HANDEL, rx485DataDMA, RS_READ_SIZE);
}

void DoSendFile(void)
{
	DoSendFileOpen();
	DoSendFileBodyPacket(0x0000, (UINT)(16 * 1024));
	DoSendFileClose();
}

void DoSendFileOpen(void)
{
	uint8_t t[1] = {0};
	FRESULT res = FR_OK;
	uint8_t	fileNameLen = 0;
	FILINFO fno;

	util_mem_set(&sdValue.sendFileName[0], 0x00, sizeof(sdValue.sendFileName));
	sdValue.sendFileName[0] = '0';
	sdValue.sendFileName[1] = ':';

	for(sdValue.sendFileNameLen = 0; sdValue.sendFileNameLen < 22; sdValue.sendFileNameLen++)
	{
		if(Rx485Data[7 + sdValue.sendFileNameLen] == '>') break;

		sdValue.sendFileName[13 + sdValue.sendFileNameLen] = Rx485Data[7 + sdValue.sendFileNameLen];
	}
	sdValue.sendFileName[ 2] = '/';
	sdValue.sendFileName[ 3] = '2';
	sdValue.sendFileName[ 4] = '0';
	sdValue.sendFileName[ 5] = sdValue.sendFileName[14];
	sdValue.sendFileName[ 6] = sdValue.sendFileName[15];
	sdValue.sendFileName[ 7] = '/';
	sdValue.sendFileName[ 8] = sdValue.sendFileName[16];
	sdValue.sendFileName[ 9] = sdValue.sendFileName[17];
	sdValue.sendFileName[10] = '/';
	sdValue.sendFileName[11] = sdValue.sendFileName[18];
	sdValue.sendFileName[12] = sdValue.sendFileName[19];

	res = f_stat((const TCHAR*)sdValue.sendFileName, &fno);

	if(osSemaphoreGetCount(CountingSem485TxHandle) == 0)
		osSemaphoreRelease(CountingSem485TxHandle);

	if(res == FR_OK){
		res = f_open(&sdValue.sendFileObject, sdValue.sendFileName, FA_OPEN_EXISTING | FA_READ);
		if(res == FR_OK)	//���� ���� ����
		{
			sdValue.sdState = SCS_OK;
			doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_DOWNLOAD_HEADER, sdValue.sendFileName, fileNameLen, 36, 56);
			SendUart485String(tx485DataDMA, 56);
		}
		else		//���� ���� ����
		{
			sdValue.sdState = SCS_OPEN_ERROR;
			t[0] = SCS_OPEN_ERROR;
			doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
			SendUart485String(tx485DataDMA, 32);
		}
	}
}

void DoSendFileBodyPacket(uint32_t Offset, UINT packetSize)
{
	uint8_t t[1] = {0};
    int32_t ReadSize;
    uint16_t i = 0, j, ct;
	uint16_t len;
	uni4Byte temp;

	if(osSemaphoreGetCount(CountingSem485TxHandle) == 0)
		osSemaphoreRelease(CountingSem485TxHandle);

	do{
		util_mem_set(&ReadFileBuf[0], 0x00, sizeof(ReadFileBuf));
		temp.UI32 = Offset + (packetSize * i);
		ReadFileBuf[0] = temp.UI8[0];
		ReadFileBuf[1] = temp.UI8[1];
		ReadFileBuf[2] = temp.UI8[2];
		ReadFileBuf[3] = temp.UI8[3];
		//util_mem_cpy(&ReadFileBuf[0], &temp.UI8[0], 4);
		temp.UI32 = Offset + (packetSize * (i + 1));
		ReadFileBuf[4] = temp.UI8[0];
		ReadFileBuf[5] = temp.UI8[1];
		ReadFileBuf[6] = temp.UI8[2];
		ReadFileBuf[7] = temp.UI8[3];
		//util_mem_cpy(&ReadFileBuf[4], &temp.UI8[0], 4);

	    ReadSize = DoSendFileRead(Offset + (packetSize * i), packetSize );
		doMakeSend485DataDownLoad(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_DOWNLOAD_BADY, &ReadFileBuf[0], ReadSize + 8, packetSize + 8, packetSize + 8 + 20);
		SendUart485String(&tx485DataDMA[0], packetSize + 8 + 20);

/*		len = (packetSize + 8 + 20);
		ct  = (len / 32);

		for(j = 0; j < ct; j++)
		{
			SendUart485String(&tx485DataDMA[j * 32], 32);
			len -= 32;
		}
		osDelay(1);
		SendUart485String(&tx485DataDMA[j * 32], len);
		*/
		if(packetSize != ReadSize)	//������ ��Ŷ
		{
			osDelay(1);
			break;
		}
		i++;
	}while(1);

	osDelay(1);
}

void DoSendFileClose(void)
{
	FRESULT res = FR_OK;
	uint8_t t[1] = {0};

	res = f_close(&sdValue.sendFileObject);
	if(res == FR_OK)	//���� �������� ����
	{
		sdValue.sdState = SCS_OK;
		doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_DOWNLOAD_FOOTER, sdValue.sendFileName, sdValue.sendFileNameLen, 36, 56);
		SendUart485String(tx485DataDMA, 56);
	}
	else		//���� �ݱ� ����
	{
		sdValue.sdState = SCS_CLOSE_ERROR;
		t[0] = SCS_CLOSE_ERROR;
		doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
		SendUart485String(tx485DataDMA, 32);
	}
}

int32_t DoSendFileRead(FSIZE_t Offset, UINT ReadSize)
{
	FRESULT res = FR_OK;
	UINT br = 0;
	uint8_t t[1] = {0};

	res = f_lseek(&sdValue.sendFileObject, Offset);
	if(res != FR_OK)
	{
		sdValue.sdState = SCS_OK;
		t[0] = SCS_SEEK_ERROR;
		doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
		SendUart485String(tx485DataDMA, 32);
	}

	res = f_read(&sdValue.sendFileObject, &ReadFileBuf[8], ReadSize, &br);
	if(res != FR_OK)
	{
		sdValue.sdState = SCS_READ_ERROR;
		t[0] = SCS_READ_ERROR;
		doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_ERROR, t, 1, 12, 32);
		SendUart485String(tx485DataDMA, 32);
	}

	return (int32_t)br;
}

void DoReadFileList(void)
{
	uint8_t	t[1] = {0};
	TCHAR 	root_directory[3] = "0:";
	uint8_t i = 0;

	doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_LIST_START, t, 0, 12, 32);
	SendUart485String(tx485DataDMA, 32);

	util_mem_set(&sdValue.scanDir[0], 0x00, sizeof(sdValue.scanDir));
	osDelay(1);
	sdValue.scanDirDeep = 0;
	util_mem_set(&sdValue.scanFilePath, 0x00, sizeof(sdValue.scanFilePath));
	osDelay(1);

	if(Rx485Data[7] == '.')	//��Ʈ�� ��û ���� ���
	{
		osDelay(1);
		scan_files(root_directory);
	}
	else	//��� ���� ���� ���
	{
		util_mem_set(&sdValue.scanReadFileName[0], 0x00, sizeof(sdValue.scanReadFileName));
		sdValue.scanReadFileName[0] = '0';
		sdValue.scanReadFileName[1] = ':';

		for(i = 0; i < 22; i++)
		{
			if(Rx485Data[7 + i] == '>') break;

			sdValue.scanReadFileName[2 + i] = Rx485Data[7 + i];
		}

		while(scan_files((char*)sdValue.scanReadFileName) != FR_OK)
		{
			osDelay(1);
		}
	}

	doMakeSend485Data(tx485DataDMA, CMD_SD_CARD, OP_SDCARD_LIST_END, t, 0, 12, 32);
	SendUart485String(tx485DataDMA, 32);
}

void doSaveIntervalTime(void)   //���÷���Ʈ
{
    SysProperties.intervalTime.UI8[0] = Rx485Data[ 7];
    SysProperties.intervalTime.UI8[1] = Rx485Data[ 8];
    SysProperties.intervalTime.UI8[2] = Rx485Data[ 9];
    SysProperties.intervalTime.UI8[3] = Rx485Data[10];

	doMakeSend485Data(tx485DataDMA, CMD_TEMP_TEST, OP_TEMP_SAMPLE_RATE, &SysProperties.intervalTime.UI8[0], 4, 12, 32);
	SendUart485String(tx485DataDMA, 32);
}

void CmdWarningTempSet(void)
{
	uni4Byte setThreshold;

	setThreshold.UI8[0] = Rx485Data[ 9];		//���� �ϴ� �µ�
	setThreshold.UI8[1] = Rx485Data[10];
	setThreshold.UI8[2] = Rx485Data[11];
	setThreshold.UI8[3] = Rx485Data[12];

	DoThresholdSet(Rx485Data[7], Rx485Data[8], setThreshold);
}

void CmdWarningTempReq(void)
{
	DoThresholdReq(Rx485Data[7]);
}

void CmdRevisionApplySet(void)
{
	DoRevisionApplySet(Rx485Data[7], Rx485Data[8]);	//slot ��ȣ ���� , 0: �����µ� ���, 1: �����µ� ���
}

void CmdRevisionConstantSet(void)
{
	TestData.revisionConstant[Rx485Data[7]].UI8[0] = Rx485Data[ 8];
	TestData.revisionConstant[Rx485Data[7]].UI8[1] = Rx485Data[ 9];
	TestData.revisionConstant[Rx485Data[7]].UI8[2] = Rx485Data[10];
	TestData.revisionConstant[Rx485Data[7]].UI8[3] = Rx485Data[11];

	DoRevisionConstantSet(Rx485Data[7]);
}

void CmdRevisionApplyReq(void)
{
	DoRevisionApplyReq(Rx485Data[7]);
}

void CmdRevisionConstantReq(void)
{
	DoRevisionConstantReq(Rx485Data[7]);
}

void CmdCalibrationRTDConstSet(void)
{
	uni4Byte 	readConst;
	uint8_t		i = 0;

	TestData.rtdCalibrationConst.UI8[0] = Rx485Data[ 7];
	TestData.rtdCalibrationConst.UI8[1] = Rx485Data[ 8];
	TestData.rtdCalibrationConst.UI8[2] = Rx485Data[ 9];
	TestData.rtdCalibrationConst.UI8[3] = Rx485Data[10];

	readConst.UI32 = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);

	if(TestData.rtdCalibrationConst.Float != readConst.Float)
	{
		do{
			doFlashWriteRevision();
			readConst.UI32 = ReadFlash(FLASH_RTD_CALIBRATION_CONSTAN);
			i++;
			if(i > 10)
				break;
		}while(TestData.rtdCalibrationConst.Float != readConst.Float);
	}

	if(i > 10)		//��� ����
	{
		i == 0xFF;
		doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET, &i, 1, 12, 32);
	}
	else			//��� ����
	{
		doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET, &readConst.UI8[0], 4, 12, 32);
	}
	SendUart485String(tx485DataDMA, 32);
}

void CmdCalibrationNTCTableCal(void)
{
	DoCalibrationNTCTableCal(Rx485Data[7]);	//slot ��ȣ ����
}

void CmdCalibrationNTCConstantSet(void)
{
	TestData.ntcCalibrationConst.UI8[0] = Rx485Data[ 8];
	TestData.ntcCalibrationConst.UI8[1] = Rx485Data[ 9];
	TestData.ntcCalibrationConst.UI8[2] = Rx485Data[10];
	TestData.ntcCalibrationConst.UI8[3] = Rx485Data[11];

	DoCalibrationNTCConstantSet(Rx485Data[ 7]);
}

void CmdCalibrationRTDConstReq(void)
{
	doMakeSend485Data(tx485DataDMA, CMD_CALIBRATION, OP_CALIBRATION_RTD_CONSTANT_SET, &TestData.rtdCalibrationConst.UI8[0], 4, 12, 32);
	SendUart485String(tx485DataDMA, 32);
}

void CmdCalibrationNTCTableReq(void)
{
	DoCalibrationNTCTableReq(Rx485Data[7]); //slot ��ȣ ����
}

void CmdCalibrationNTCConstantReq(void)
{
	DoCalibrationNTCConstantReq(Rx485Data[7]); //slot ��ȣ ����
}

void doSetTime(void)
{
//    uint8_t i = 0;
    uint8_t res = TRUE;
	HAL_StatusTypeDef resHal;
    RTC_DateTypeDef setDate, getDate;
    RTC_TimeTypeDef setTime, getTime;

    setDate.Year    = Rx485Data[7];
    setDate.Month   = Rx485Data[8];
    setDate.Date    = Rx485Data[9];
    setDate.WeekDay = Rx485Data[13];
    setTime.Hours   = Rx485Data[10];
    setTime.Minutes = Rx485Data[11];
    setTime.Seconds = Rx485Data[12];
	setTime.TimeFormat = RTC_HOURFORMAT12_AM;
	setTime.StoreOperation = RTC_STOREOPERATION_RESET;

    do{
        while(1)
		{
			resHal = HAL_RTC_SetTime(&hrtc, &setTime, RTC_FORMAT_BIN);
			if(resHal == HAL_OK)
				break;
			osDelay(1);
        }
		while(1)
		{
			resHal = HAL_RTC_SetDate(&hrtc, &setDate, RTC_FORMAT_BIN);
			if(resHal == HAL_OK)
				break;
			osDelay(1);
        }

        HAL_RTC_GetTime(&hrtc, &getTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &getDate, RTC_FORMAT_BIN);

        /*i++;
        if(i > 100)
        {
            res = FALSE;
            break;
        }*/

        if( (setDate.Year  == getDate.Year)  && (setDate.Month   == getDate.Month)   && (setDate.Date    == getDate.Date)   &&
            (setTime.Hours == getTime.Hours) && (setTime.Minutes == getTime.Minutes) && (setTime.Seconds == getTime.Seconds)  )
        {
            break;
        }
    }while(1);

    doMakeSend485Data(tx485DataDMA, CMD_TIME, OP_TIME_SET, &res, 1, 12, 32);
    SendUart485String(tx485DataDMA, 32);
}

void doGetTime(void)
{
    uint8_t res = 0;

    doMakeSend485Data(tx485DataDMA, CMD_TIME, OP_TIME_REQ, &res, 0, 12, 32);
    SendUart485String(tx485DataDMA, 32);
}
