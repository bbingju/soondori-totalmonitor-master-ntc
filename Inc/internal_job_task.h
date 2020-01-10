#ifndef INTERNAL_JOB_TASK_H
#define INTERNAL_JOB_TASK_H

#include <stdio.h>

typedef enum {
    INTERNAL_JOB_TYPE_NONE,
    INTERNAL_JOB_TYPE_TO,
    INTERNAL_JOB_TYPE_FROM,
    INTERNAL_JOB_TYPE_MAX
} INTERNAL_JOB_TYPE_E;

#ifdef __cplusplus
extern "C" {
#endif

int post_internal_job(INTERNAL_JOB_TYPE_E type, void *data, size_t datalen);
void internal_job_task(void const *arg);

#ifdef __cplusplus
}
#endif

#endif /* INTERNAL_JOB_TASK_H */
