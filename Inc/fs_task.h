#ifndef FS_TASK_H
#define FS_TASK_H

#include <stdio.h>
#include <stdbool.h>

typedef enum {
    FS_JOB_TYPE_NONE,
    FS_JOB_TYPE_SAVE_LOG,
    FS_JOB_TYPE_QUERY_FILELIST,
    FS_JOB_TYPE_DOWNLOAD_FILE_HEADER,
    FS_JOB_TYPE_DOWNLOAD_FILE,
    FS_JOB_TYPE_DELETE_FILE,
    FS_JOB_TYPE_FORMAT,
    FS_JOB_TYPE_MAX
} FS_JOB_TYPE_E;

extern char request_filename[];

#ifdef __cplusplus
extern "C" {
#endif

int post_fs_job(FS_JOB_TYPE_E type);
void fs_task(void const *arg);

#ifdef __cplusplus
}
#endif


#endif /* FS_TASK_H */
