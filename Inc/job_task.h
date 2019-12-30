#ifndef JOB_TASK_H
#define JOB_TASK_H

#include <stdio.h>

typedef enum {
    JOB_TYPE_NONE,
    JOB_TYPE_TO_EXTERNAL,
    JOB_TYPE_FROM_EXTERNAL,
    JOB_TYPE_TO_INTERNAL,
    JOB_TYPE_FROM_INTERNAL,
    JOB_TYPE_MAX
} JOB_TYPE_E;

#ifdef __cplusplus
extern "C" {
#endif

int post_job(JOB_TYPE_E type, void *data, size_t datalen);
void job_task(void const *arg);

#ifdef __cplusplus
}
#endif

#endif /* JOB_TASK_H */
