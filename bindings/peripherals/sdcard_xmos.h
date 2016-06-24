#ifndef __SDCARD_XMOS_H__
#define __SDCARD_XMOS_H__

extern "C"
{
    #include "sdcard.h"
}

void SDCardTask(volatile struct SDCard* unsafe card);

#endif
