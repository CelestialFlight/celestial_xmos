#include "ff.h"
#include "sdcard.h"
#include <stdio.h>

static void _SDCardPrintCallback(struct SDCard* card);
static void _SDCardForceSendCallback(struct SDCard* card);

int SDCardInit(struct SDCard* card)
{
    card->shouldSendBuffer = 0;
    SerialBufferInit(&card->buf);
    card->buf.sendPrintf = &_SDCardPrintCallback;
    card->buf.forceSend = &_SDCardForceSendCallback;
    return 0;
}

// A callback for everytime a printf statement is called.
// Note: this implementation is blocking.
// Another Note: This callback takes in a SDCard pointer, while
// serial buffer will give it back as a SerialBuffer pointer.  We can
// get away with this as the SerialBuffer is the first item in our
// SDCard structure (ie, the SDCard pointer is the exact same as
// the SerialBuffer pointer).  This might be a bit sketchy, care
// should be taken...
static void _SDCardPrintCallback(struct SDCard* card)
{
    // It's not worth sending small amounts of data to sd card as
    // there is a large overhead for writing to the card.
    if (SerialBufferSize(&card->buf) < 1024) return;

    card->shouldSendBuffer = 1;
}

static void _SDCardForceSendCallback(struct SDCard* card)
{
    SDCardForceWriteBuffer(card);
}

int SDCardMount(struct SDCard* card)
{
    FRESULT rc;

    f_mount(0, &card->Fatfs);
    {
      FATFS *fs;
      DWORD fre_clust;

      // Get volume information and free clusters of drive 0
      rc = f_getfree("0:", &fre_clust, &fs);
      if(rc) return rc;

      card->freeSpace = (fre_clust * fs->csize) / 2;
      card->totalSpace = ((fs->n_fatent - 2) * fs->csize) /2;
    }

    return 0;
}

int SDCardUnlink(struct SDCard* card, const char* fileName)
{
    return f_unlink(fileName);
}

int SDCardOpen(struct SDCard* card, const char* fileName)
{
    return f_open(&card->Fil, fileName, FA_WRITE | FA_OPEN_ALWAYS);
}

int SDCardPrintf(struct SDCard* card, char* c, ...)
{
    va_list ap;
    va_start(ap, c);
    SerialBufferPrintfVargs(&card->buf, c, ap);
    va_end(ap);

    return 0;
}

// "Closes" a file temporary, but is reactivated by the next write/read.
// This makes the code after this call non-
int SDCardSync(struct SDCard* card)
{
    return f_sync(&card->Fil);
}

int SDCardWrite(struct SDCard* card, void* buf, int num)
{
    UINT wb;
    return f_write(&card->Fil, buf, num, &wb);
}

// Forces a write to the sd card.  Good for last data to
// ensure no data is left in the serial buffer without getting
// written.
void SDCardForceWriteBuffer(volatile struct SDCard* card)
{
    card->shouldSendBuffer = 1;

    // Blocking until buffer is written to sd card.
    while (card->shouldSendBuffer == 1);
}

int SDCardClose(struct SDCard* card)
{
    return f_close(&card->Fil);
}
