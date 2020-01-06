#ifndef APP_CTX_H
#define APP_CTX_H

#include "fatfs.h"

#include <stdint.h>
#include <stdbool.h>

typedef struct app_ctx {
	bool time_synced;
	FATFS *sd_ff;
	char *sd_root;
} app_ctx_t;

void app_ctx_init(app_ctx_t *);

#endif /* APP_CTX_H */
