#include "0_soonFlashMemory.h"
#include "app_ctx.h"

extern app_ctx_t ctx;

/**************************************************************************************************************
*	Write 동작의 흐름은 아래와 같다.
*	1) Control register unlock
*	2) 4byte 단위로 쓰기 수행
*	3) Control register lock
***************************************************************************************************************/
HAL_StatusTypeDef WriteFlash(uint32_t Address, uint32_t data, uint32_t* errorcode)
{
	HAL_StatusTypeDef res = HAL_OK;

	/* Unlock to control */
	HAL_FLASH_Unlock();

	__HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR | FLASH_SR_PGAERR | FLASH_SR_PGPERR | FLASH_SR_PGSERR);

	/* Writing data to flash memory */
	if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, data) == HAL_OK)
	{
		errorcode = 0;
	}
	else
	{
		*errorcode = HAL_FLASH_GetError();
		res = HAL_ERROR;
	}

	/* Lock flash control register */
	HAL_FLASH_Lock();  

	return res;
}

/**************************************************************************************************************
*	Read 동작의 경우에는 직관적으로 해당 Memory 주소를 Access 하여 4byte 데이터 읽어온다.
***************************************************************************************************************/
__IO uint32_t ReadFlash(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}

/**************************************************************************************************************
*	Flash에 쓰기 위해 해당 영역을 지우는 Erase 동작을 살펴보자.
*	코드의 흐름을 살펴보면 아래와 같다.
*	1) Control register unlock
*	2) 지우고자 하는 섹터의 크기 계산
*	3) 섹터 삭제 명령
*	4) 데이터 캐시에 이미 올라간 데이터인 경우에 Cache까지 삭제가 필요한 경우에는 Cache Clear
* 	5) Control register lock
***************************************************************************************************************/
HAL_StatusTypeDef EraseFlash(uint32_t startAdd, uint32_t endAdd)
{
  uint32_t SectorError = 0;
    
  /* Unlock to control */
  HAL_FLASH_Unlock();
  
  /* Erase sectors */
  FLASH_EraseInitTypeDef EraseInitStruct;
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = GetSector(startAdd);
  EraseInitStruct.NbSectors = endAdd;

  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
  { 
    uint32_t errorcode = HAL_FLASH_GetError();            
    return HAL_ERROR;
  }
  
  /* Lock flash control register */
  HAL_FLASH_Lock();
  
  return HAL_OK;  
}

/*********************************************************************
*	doFlashWriteRevision
*	플레시 메로지 내용 수정 체크 함수 
**********************************************************************/
uint32_t doFlashWriteRevision(float value)
{
	uint32_t flashError = 0;

	EraseFlash(FLASH_SAVE_CHK, 1U);
	WriteFlash(FLASH_SAVE_CHK, FLASH_SAVE_FLAG, &flashError);
	WriteFlash(FLASH_RTD_CALIBRATION_CONSTAN, ctx.rtd.calibration_const, &flashError);

	return flashError;
}

/*********************************************************************
*	GetSector
*	플레시 기록을 위해 해당 섹터를 확인 
**********************************************************************/
uint32_t GetSector(uint32_t Address)
{
	uint32_t sector = 0;
	
	if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
	{
	  sector = FLASH_SECTOR_0;	
	}
	else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
	{
	  sector = FLASH_SECTOR_1;	
	}
	else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
	{
	  sector = FLASH_SECTOR_2;	
	}
	else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
	{
	  sector = FLASH_SECTOR_3;	
	}
	else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
	{
	  sector = FLASH_SECTOR_4;	
	}
	else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
	{
	  sector = FLASH_SECTOR_5;	
	}
	else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
	{
	  sector = FLASH_SECTOR_6;	
	}
	else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
	{
	  sector = FLASH_SECTOR_7;	
	}
	else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
	{
	  sector = FLASH_SECTOR_8;	
	}
	else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
	{
	  sector = FLASH_SECTOR_9;	
	}
	else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
	{
	  sector = FLASH_SECTOR_10;  
	}
	else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
	{
	  sector = FLASH_SECTOR_11;
	}
	
	return sector;
}

