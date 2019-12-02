/*********************************************************************
*	IS31FL3726 dirver
*	16-BIT COLOR LED DRIVER WITH PWM CONTROL
*	clock 와 data의 신호로 16개의 led 제어 
*	16-Segment 제어를 위해 사용 함 
**********************************************************************/
#include "0_16Segment.h"
#include "0_Util.h"
#include "main.h"

/*********************************************************************
*	16-Segment Font
**********************************************************************/
uint16_t SG_NUMBER[10] = {		0xFF09,		// 0
															0x3000,		// 1
															0xEEC0,		// 2
															0xFCC0,		// 3
															0x31C0,		// 4
															0xCD84,		// 5
															0xDFC0,		// 6
															0xF000,		// 7
															0xFFC0,		// 8
															0xFDC0 };	// 9
uint16_t SG_ALPHABET[26] = {	0xF3C0,		// A
															0xFC52,		// B
															0xCF00,		// C
															0xFC12,		// D
															0xCFC0,		// E
															0xC380,		// F
															0xDF40,		// G
															0x33C0,		// H
															0xCC12,		// I
															0x3E00,		// J
															0x038C,		// K
															0x0F00,		// L
															0x3328,		// M
															0x3324,		// N
															0xFF00,		// O
															0xE3C0,		// P
															0xFF04,		// Q
															0xE3C4,		// R
															0xDDC0,		// S
															0xC012,		// T
															0x3F00,		// U
															0x0F09,		// V
															0x3305,		// W
															0x002D,		// X
															0x002A,		// Y
															0xCC09 };	// Z
uint16_t SG_SPECIAL[8] = {		0x0000,		// NULL
															0x0C00,		// SSP_UNDERBAR
															0x00C0,		// SSP_HYPHEN
															0x0024, 	// SSP_BACKSLASH
															0x0012,		// SSP_OR
															0x0009,		// SSP_SLASH
															0x00D2,		// SSP_PLUS
															0x00FF };	// SSP_STAR


/*********************************************************************
*	SegmentDisplay
*	digit : 1, 2로 보내며 첫번ㅈ재 segment와 2번ㅈ재 segment 를 선택함 
*	text : ascii 문자를 입력 하면 해당하는 문자 표시 
**********************************************************************/	
void SegmentDisplay(uint8_t digit, uint16_t data)
{
	uint8_t i;
	
	GPIO_TypeDef * 	LATCH_PORT;
	uint16_t				LATCH_PIN;
	GPIO_TypeDef * 	ENABLE_PORT;
	uint16_t 				ENABLE_PIN;
	GPIO_TypeDef * 	SCL_PORT;
	uint16_t 				SCL_PIN;
	GPIO_TypeDef * 	SDA_PORT;
	uint16_t 				SDA_PIN;

	if(digit == 1)
	{
		LATCH_PORT 	= DIGI1_LATCH_GPIO_Port;
		LATCH_PIN 	= DIGI1_LATCH_Pin;
		ENABLE_PORT = DIGI1_ENABLE_GPIO_Port;
		ENABLE_PIN 	= DIGI1_ENABLE_Pin;
		SCL_PORT		= DIGI1_SCL_GPIO_Port;
		SCL_PIN			= DIGI1_SCL_Pin;
		SDA_PORT		= DIGI1_SDA_GPIO_Port;
		SDA_PIN			= DIGI1_SDA_Pin;
	}
	else
	{
		LATCH_PORT 	= DIGI2_LATCH_GPIO_Port;
		LATCH_PIN 	= DIGI2_LATCH_Pin;
		ENABLE_PORT = DIGI2_ENABLE_GPIO_Port;
		ENABLE_PIN 	= DIGI2_ENABLE_Pin;
		SCL_PORT		= DIGI2_SCL_GPIO_Port;
		SCL_PIN			= DIGI2_SCL_Pin;
		SDA_PORT		= DIGI2_SDA_GPIO_Port;
		SDA_PIN			= DIGI2_SDA_Pin;
	}

	HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_SET);
	doNOP(1);

	for(i = 0; i < 16; i++)
	{
		HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_RESET);
		doNOP(1);

		if((data & 0x0001) == 0x0000)
		{
			HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(SDA_PORT, SDA_PIN, GPIO_PIN_SET);
		}
		doNOP(1);
		HAL_GPIO_WritePin(SCL_PORT, SCL_PIN, GPIO_PIN_SET);		
		doNOP(1);
		data = data >> 1;		
	}
	
	HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_SET);
	doNOP(1);
	HAL_GPIO_WritePin(LATCH_PORT, LATCH_PIN, GPIO_PIN_RESET);
	doNOP(1);
	HAL_GPIO_WritePin(ENABLE_PORT, ENABLE_PIN, GPIO_PIN_RESET);	  
}

/*********************************************************************
*	doTextToDigitdata
*	text : ascii 문자를 입력 
* 	해당 문자를 Font 로 치환 
**********************************************************************/	
uint16_t doTextToDigitdata(char text)
{
	if((text >= 'a') && (text <= 'z')) 
		text -= 0x20;

	if((text >= 'A') && (text <= 'Z'))
	{
		return SG_ALPHABET[text - 'A'];
	}
	else if((text >= '0') && (text <= '9'))
	{
		return SG_NUMBER[text - '0'];
	}
	else		// 특수문자 처리 
	{
		switch(text)
		{
			case ' ':
				return SG_SPECIAL[SSP_NULL];
			case '_':
				return SG_SPECIAL[SSP_UNDERBAR];
			case '-':
				return SG_SPECIAL[SSP_HYPHEN];
			case '|':
				return SG_SPECIAL[SSP_OR];
			case '/':
				return SG_SPECIAL[SSP_SLASH];
			case '\\':
				return SG_SPECIAL[SSP_BACKSLASH];
			case '+':
				return SG_SPECIAL[SSP_PLUS];
			case '*':
				return SG_SPECIAL[SSP_STAR];
		}
	}
  
  return 0;
}

/*********************************************************************
*	doNumberToDigitdata
*	number : 숫자를 적는다. 해당 문자열을 반환한다.  
**********************************************************************/	
uint16_t doNumberToDigitdata(SEGMENT_SPECIAL_FONT number)
{
	return SG_SPECIAL[number];
}
