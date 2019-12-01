#ifndef __SDCARD_H__
#define __SDCARD_H__

#include "string.h"


#include "ff.h"
#include "fatfs.h"
#include "ffconf.h"		/* FatFs configuration options */

#include "0_GlobalValue.h"
#include "0_Util.h"



///////////////////////////////////////////////////////////////////////////
// SD CARD
///////////////////////////////////////////////////////////////////////////
FRESULT MountSDIO(void);
FRESULT UnMountSDIO(void);
FRESULT DoFolderCheck(void);
void DoFileCheck(void);
void DoMakeFile(void);
void DoFileClose(void);
void DoMakeLoadFileName(void);
void DoWriteFileHeader(void);
void DoDataWrite(void);
FRESULT scan_files (char* path);


#endif
