/**
  ******************************************************************************
  * @file   fatfs.c
  * @brief  Code for fatfs applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

#include "fatfs.h"

uint8_t retSD;    /* Return value for SD */
char SDPath[4];   /* SD logical drive path */
FATFS SDFatFS;    /* File system object for SD logical drive */
FIL SDFile;       /* File object for SD */

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */    

void MX_FATFS_Init(void) 
{
  /*## FatFS: Link the SD driver ###########################*/
  retSD = FATFS_LinkDriver(&SD_Driver, SDPath);

  /* USER CODE BEGIN Init */
	/* additional user code for init */
	if(retSD != 0)	// 0이면 정상
	{
		sdValue.sdState = SCS_LINK_ERROR;
	}
	else
	{
		sdValue.sdState = SCS_OK;
	}
  /* USER CODE END Init */
}

/**
  * @brief  Gets Time from RTC 
  * @param  None
  * @retval Time in DWORD
  */
DWORD get_fattime(void)
{
  /* USER CODE BEGIN get_fattime */
  RTC_TimeTypeDef Time_Value;
  RTC_DateTypeDef Date_Value;
	
  HAL_RTC_GetTime(&hrtc, &Time_Value, RTC_FORMAT_BIN);
  HAL_RTC_GetDate(&hrtc, &Date_Value, RTC_FORMAT_BIN);

  return (DWORD)((  Date_Value.Year + 20 ) 	<< 25
                 | Date_Value.Month   		<< 21
                 | Date_Value.Date    		<< 16
                 | Time_Value.Hours   		<< 11
                 | Time_Value.Minutes 		<< 5
                 | Time_Value.Seconds );
  /* USER CODE END get_fattime */  
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
