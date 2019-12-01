#include "0_soonFlashMemory.h"


/**************************************************************************************************************
*	Write ������ �帧�� �Ʒ��� ����.
*	1) Control register unlock
*	2) 4byte ������ ���� ����
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
*	Read ������ ��쿡�� ���������� �ش� Memory �ּҸ� Access �Ͽ� 4byte ������ �о�´�.
***************************************************************************************************************/
__IO uint32_t ReadFlash(uint32_t Address)
{
	return *(__IO uint32_t*)Address;
}

/**************************************************************************************************************
*	Flash�� ���� ���� �ش� ������ ����� Erase ������ ���캸��.
*	�ڵ��� �帧�� ���캸�� �Ʒ��� ����.
*	1) Control register unlock
*	2) ������� �ϴ� ������ ũ�� ���
*	3) ���� ���� ���
*	4) ������ ĳ�ÿ� �̹� �ö� �������� ��쿡 Cache���� ������ �ʿ��� ��쿡�� Cache Clear
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
*	�÷��� �޷��� ���� ���� üũ �Լ� 
**********************************************************************/
uint32_t doFlashWriteRevision(void)
{
	uint32_t	flashError = 0;

	EraseFlash(FLASH_SAVE_CHK, 1U);
	WriteFlash(FLASH_SAVE_CHK, FLASH_SAVE_FLAG, &flashError);
	WriteFlash(FLASH_RTD_CALIBRATION_CONSTAN, TestData.rtdCalibrationConst.UI32, &flashError);

	return flashError;
}

/*********************************************************************
*	GetSector
*	�÷��� ����� ���� �ش� ���͸� Ȯ�� 
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

