#ifndef APP_CTX_H
#define APP_CTX_H

#include "fatfs.h"

#include <stdint.h>
#include <stdbool.h>
#include <time.h>

#define MAX_SLOT_NUM        4
#define CHANNEL_NBR         32

typedef enum {
	SLOT_TYPE_NONE,
	SLOT_TYPE_NTC,
	SLOT_TYPE_RELAY,
	SLOT_TYPE_RTD,
	SLOT_TYPE_MULTI,
	SLOT_TYPE_BT,
} slot_type_t;

enum CHANNEL_STATE {
    CHANNEL_STATE_NORMAL_CONNECTED = 0, // 정상
    CHANNEL_STATE_OVER_TEMP,	    // 경고온도 초과
    CHANNEL_STATE_SENSOR_CLOSE,	    // 센서 쇼트
    CHANNEL_STATE_DISCONNECTED	    // 센서 없음
};
typedef uint8_t CHANNEL_STATE_E;

struct slot_ntc_s {
	float           temperatures[CHANNEL_NBR];
	float           old_temperatures[CHANNEL_NBR];
	CHANNEL_STATE_E channel_states[CHANNEL_NBR];
	float           thresholds[CHANNEL_NBR];

	struct compensated_s {
		uint8_t applied;
		float tr1;
		float tr2;
		float contact_const;
	} compensated;

	float           calibration_tbl[CHANNEL_NBR];
	/* float           calibration_const; */

	float           variation;
};

struct slot_s {
	uint8_t id;
	slot_type_t type;
	bool inserted;

	union {
		struct slot_ntc_s ntc;
	};
};

#define INIT_SLOTS(s, n) do {						\
		for (int i = 0; i < n; i++) {				\
			struct slot_s *p = s + i;			\
			p->id = i; p->type = SLOT_TYPE_NONE;		\
			p->inserted = false;				\
			memset(p->ntc.temperatures, 0, sizeof(float) * CHANNEL_NBR); \
			memset(p->ntc.channel_states, CHANNEL_STATE_DISCONNECTED, CHANNEL_NBR); \
		}							\
	} while (0)

#define FOREACH(item, array)						\
	for (int keep = 1,						\
		     count = 0,						\
		     size = sizeof (array) / sizeof *(array);		\
	     keep && count != size;					\
	     keep = !keep, count++)					\
		for (item = (array) + count; keep; keep = !keep)

struct rtd_s {
	bool     attached;
	uint32_t adc_val;
	float    temperature;
	float    calibration_const;
};

typedef struct app_ctx {
	bool time_synced;
	time_t now;

	FATFS *sd_ff;
	char *sd_root;
	bool sd_inserted;
	FIL metafd, logfd;
	SD_RET_E sd_last_error;

	bool heavy_job_processing;

	float battery;
	struct rtd_s rtd;
	float temperature;
	float humidity;

	struct slot_s slots[MAX_SLOT_NUM];
	uint8_t last_slot_id;
} app_ctx_t;

void app_ctx_init(app_ctx_t *);
const char *firmware_version();
time_t datetime_to_timestamp(RTC_DateTypeDef *date, RTC_TimeTypeDef *time);
void update_rtc(time_t now);

#endif /* APP_CTX_H */
