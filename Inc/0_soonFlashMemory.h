#ifndef __SOON_FLASH_MEMORY_H__
#define __SOON_FLASH_MEMORY_H__

#ifndef _STDINT
	#include "stdint.h"
#endif

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

#include "0_GlobalValue.h"
#include "0_GlobalDefine.h"


////////////////////////////////////////////////////////////////////
// Flash Memory Address
////////////////////////////////////////////////////////////////////
#define	FLASH_START_ADDRESS				((uint32_t)0x08000000)

#define FLASH_SAVE_CHK					ADDR_FLASH_SECTOR_11
#define	FLASH_SAVE_FLAG					((uint32_t)0x0000007F)

#define	FLASH_RTD_CALIBRATION_CONSTAN	(FLASH_SAVE_CHK + 0x00000008)					//a?κ? 4BYTE (FLOAT) * 32?? = 128 (0X80) (0~15 : ?????μ?, 16~31 : ???μ?)

#if defined(STM32F405xx)
 // /* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     		((uint32_t)0x08000000) /* Base @ of Sector 0,  16 Kbytes */
#define ADDR_FLASH_SECTOR_1     		((uint32_t)0x08004000) /* Base @ of Sector 1,  16 Kbytes */
#define ADDR_FLASH_SECTOR_2     		((uint32_t)0x08008000) /* Base @ of Sector 2,  16 Kbytes */
#define ADDR_FLASH_SECTOR_3     		((uint32_t)0x0800C000) /* Base @ of Sector 3,  16 Kbytes */
#define ADDR_FLASH_SECTOR_4     		((uint32_t)0x08010000) /* Base @ of Sector 4,  64 Kbytes */
#define ADDR_FLASH_SECTOR_5     		((uint32_t)0x08020000) /* Base @ of Sector 5,  128 Kbytes */
#define ADDR_FLASH_SECTOR_6     		((uint32_t)0x08040000) /* Base @ of Sector 6,  128 Kbytes */
#define ADDR_FLASH_SECTOR_7     		((uint32_t)0x08060000) /* Base @ of Sector 7,  128 Kbytes */
#define ADDR_FLASH_SECTOR_8     		((uint32_t)0x08080000) /* Base @ of Sector 8,  128 Kbytes */
#define ADDR_FLASH_SECTOR_9     		((uint32_t)0x080A0000) /* Base @ of Sector 9,  128 Kbytes */
#define ADDR_FLASH_SECTOR_10     		((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11     		((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */
#endif

#if 0
 // /* Base address of the Flash sectors */
#define ADDR_FLASH_SECTOR_0     		((uint32_t)0x08000000) /* Base @ of Sector 0,  1 Kbytes */
#define ADDR_FLASH_SECTOR_1     		((uint32_t)0x08000400) /* Base @ of Sector 1,  1 Kbytes */
#define ADDR_FLASH_SECTOR_2     		((uint32_t)0x08000800) /* Base @ of Sector 2,  1 Kbytes */
#define ADDR_FLASH_SECTOR_3     		((uint32_t)0x08000C00) /* Base @ of Sector 3,  1 Kbytes */
#define ADDR_FLASH_SECTOR_4     		((uint32_t)0x08001000) /* Base @ of Sector 4,  1 Kbytes */
#define ADDR_FLASH_SECTOR_5     		((uint32_t)0x08001400) /* Base @ of Sector 5,  1 Kbytes */
#define ADDR_FLASH_SECTOR_6     		((uint32_t)0x08001800) /* Base @ of Sector 6,  1 Kbytes */
#define ADDR_FLASH_SECTOR_7     		((uint32_t)0x08001C00) /* Base @ of Sector 7,  1 Kbytes */
#define ADDR_FLASH_SECTOR_8     		((uint32_t)0x08002000) /* Base @ of Sector 8,  1 Kbytes */
#define ADDR_FLASH_SECTOR_9     		((uint32_t)0x08002400) /* Base @ of Sector 9,  1 Kbytes */
#define ADDR_FLASH_SECTOR_10     		((uint32_t)0x08002800) /* Base @ of Sector 10, 1 Kbytes */
#define ADDR_FLASH_SECTOR_11     		((uint32_t)0x08002C00) /* Base @ of Sector 11, 1 Kbytes */
#define ADDR_FLASH_SECTOR_12     		((uint32_t)0x08003000) /* Base @ of Sector 12, 1 Kbytes */
#define ADDR_FLASH_SECTOR_13     		((uint32_t)0x08003400) /* Base @ of Sector 13, 1 Kbytes */
#define ADDR_FLASH_SECTOR_14     		((uint32_t)0x08003800) /* Base @ of Sector 14, 1 Kbytes */
#define ADDR_FLASH_SECTOR_15     		((uint32_t)0x08003C00) /* Base @ of Sector 15, 1 Kbytes */
#define ADDR_FLASH_SECTOR_16     		((uint32_t)0x08004000) /* Base @ of Sector 16, 1 Kbytes */
#define ADDR_FLASH_SECTOR_17     		((uint32_t)0x08004400) /* Base @ of Sector 17, 1 Kbytes */
#define ADDR_FLASH_SECTOR_18     		((uint32_t)0x08004800) /* Base @ of Sector 18, 1 Kbytes */
#define ADDR_FLASH_SECTOR_19     		((uint32_t)0x08004C00) /* Base @ of Sector 19, 1 Kbytes */
#define ADDR_FLASH_SECTOR_20     		((uint32_t)0x08005000) /* Base @ of Sector 20, 1 Kbytes */
#define ADDR_FLASH_SECTOR_21     		((uint32_t)0x08005400) /* Base @ of Sector 21, 1 Kbytes */
#define ADDR_FLASH_SECTOR_22     		((uint32_t)0x08005800) /* Base @ of Sector 22, 1 Kbytes */
#define ADDR_FLASH_SECTOR_23     		((uint32_t)0x08005C00) /* Base @ of Sector 23, 1 Kbytes */
#define ADDR_FLASH_SECTOR_24     		((uint32_t)0x08006000) /* Base @ of Sector 24, 1 Kbytes */
#define ADDR_FLASH_SECTOR_25     		((uint32_t)0x08006400) /* Base @ of Sector 25, 1 Kbytes */
#define ADDR_FLASH_SECTOR_26     		((uint32_t)0x08006800) /* Base @ of Sector 26, 1 Kbytes */
#define ADDR_FLASH_SECTOR_27     		((uint32_t)0x08006C00) /* Base @ of Sector 27, 1 Kbytes */
#define ADDR_FLASH_SECTOR_28     		((uint32_t)0x08007000) /* Base @ of Sector 28, 1 Kbytes */
#define ADDR_FLASH_SECTOR_29     		((uint32_t)0x08007400) /* Base @ of Sector 29, 1 Kbytes */
#define ADDR_FLASH_SECTOR_30     		((uint32_t)0x08007800) /* Base @ of Sector 30, 1 Kbytes */
#define ADDR_FLASH_SECTOR_31     		((uint32_t)0x08007C00) /* Base @ of Sector 31, 1 Kbytes */
#endif

HAL_StatusTypeDef WriteFlash(uint32_t Address, uint32_t data, uint32_t* errorcode);
__IO uint32_t ReadFlash(uint32_t Address);
HAL_StatusTypeDef EraseFlash(uint32_t startAdd, uint32_t endAdd);
uint32_t doFlashWriteRevision(void);
uint32_t GetSector(uint32_t Address);

#endif

