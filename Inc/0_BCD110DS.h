#ifndef __SOON_BCD110DS_H__
#define __SOON_BCD110DS_H__

#include "0_Util.h"
#include "cmsis_os.h"
#include "stm32f4xx_hal.h"


#ifndef __STM32F4xx_HAL_H
  #include "stm32f4xx_hal.h"
#endif

#ifndef _STDINT
	#include "stdint.h"
#endif

#include "0_GlobalValue.h"
#include "0_soonQueue.h"



#define AT_AT	 					"AT\r"																//  3글자 
#define AT_PLUS						"+++\r"																//  4글자 
#define AT_ATH						"ATH\r"																//  4글자 
#define AT_ATO						"ATO\r"																//  4글자 
#define AT_ATZ						"ATZ\r"																//  4글자 
#define AT_BTINFO					"AT+BTINFO?\r"												// 11글자 
#define AT_BTNAME				 	"AT+BTNAME=\"eleMon_000000000000\"\r"	// 32글자 
#define AT_UARTCONFIG_921600		"AT+UARTCONFIG,921600,N,1,0\r"				// 27글자 
#define AT_UARTCONFIG_9600			"AT+UARTCONFIG,9600,N,1,1\r"					// 25글자 
#define AT_BTCANCEL					"AT+BTCANCEL\r"												// 12글자 
#define AT_BTMODE3					"AT+BTMODE,3\r"												// 12글자 
#define AT_FACTORY_RESET			"AT&F\r"								//  5글자 


uint8_t doAT(uint8_t* RaDataBuffer);
uint8_t doAT_PLUS(uint8_t* RaDataBuffer);
uint8_t doAT_ATH(uint8_t* RaDataBuffer);
uint8_t doAT_ATZ(uint8_t* RaDataBuffer);
uint8_t doAT_BTINFO(uint8_t* RaDataBuffer);
uint8_t doAT_BTNAME(uint8_t* RaDataBuffer, uint8_t* newName);
uint8_t doAT_UARTCONFIR_921600(uint8_t* RaDataBuffer);
uint8_t doAT_BTCANCEL(uint8_t* RaDataBuffer);
uint8_t doAT_BTMODE_3(uint8_t* RaDataBuffer);
uint8_t doAT_FACTORY_RESET(uint8_t* RxDataBuffer);


#endif
