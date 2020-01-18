#include "context.h"
#include "stm32f4xx_hal.h"
#include <time.h>

extern RTC_HandleTypeDef hrtc;

struct context {
    time_t now;
};

/* static struct context context; */

/* void context_init() */
/* { */
/*     return &context; */
/* } */

time_t datetime_to_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time)
{
	struct tm t = { 0 };

	t.tm_year = date->Year - 1900;
	t.tm_mon = date->Month - 1;
	t.tm_mday = date->Date;
	t.tm_hour = time->Hours;
	t.tm_min = time->Minutes;
	t.tm_sec = time->Seconds;
	t.tm_isdst = -1;

	return mktime(&t);
}

/* from https://community.st.com/s/question/0D50X00009XkgJE/unix-epoch-timestamp-to-rtc */
void update_rtc(time_t now)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;
    struct tm time_tm;

    time_tm = *(localtime(&now));

    sTime.Hours = (uint8_t)time_tm.tm_hour;
    sTime.Minutes = (uint8_t)time_tm.tm_min;
    sTime.Seconds = (uint8_t)time_tm.tm_sec;
    if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    if (time_tm.tm_wday == 0) { time_tm.tm_wday = 7; } // the chip goes mon tue wed thu fri sat sun
    sDate.WeekDay = (uint8_t)time_tm.tm_wday;
    sDate.Month = (uint8_t)time_tm.tm_mon+1; //momth 1- This is why date math is frustrating.
    sDate.Date = (uint8_t)time_tm.tm_mday;
    sDate.Year = (uint16_t)(time_tm.tm_year+1900-2000); // time.h is years since 1900, chip is years since 2000

    /*
    * update the RTC
    */
    if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK) {
        Error_Handler();
    }

    // lock it in with the backup registers. not used in this time.
    // HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, 0x32F2);
}
