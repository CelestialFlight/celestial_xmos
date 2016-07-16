#include <stdio.h>

extern "C"
{
    #define va_list __VALIST
    #include "sdcard.h"
    #include "ff.h"
    #include "common/common.h"
}

void SDCardTask(volatile struct SDCard* unsafe card)
{
    unsafe
    {
        while(1==1)
        {
            int i;
            for (i = 0; i < _SD_MAX_FILES; i++)
            {
                // If a force save was requested, or 5 seconds has ellasped
                // since the last save.
                if (card->files[i].shouldSendBuffer == 1
                    || SystemTime() - card->files[i].lastUpdate > 5000000)
                {
                    __SDCardSendBuffer(&card->files[i]);
                    card->files[i].shouldSendBuffer = 0;
                    card->files[i].lastUpdate = SystemTime();
                }
            }
        }
    }
}

