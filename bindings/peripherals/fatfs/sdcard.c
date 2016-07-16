#include "ff.h"
#include "sdcard.h"
#include <stdio.h>

static void _SDCardPrintCallback(struct SDCardFatFSFile* card);
static void _SDCardForceSendCallback(struct SDCardFatFSFile* card);

int SDCardInit(struct SDCard* card, int bufferSize)
{
    error(card != 0);

    int i;
    for (i = 0; i < _SD_MAX_FILES; i++)
    {
        card->files[i].shouldSendBuffer = 0;

        // Initialize buffers.
        SerialBufferInit(&card->files[i].buf, bufferSize);

        // Set Callback functions
        card->files[i].buf.sendPrintf = &_SDCardPrintCallback;
        card->files[i].buf.forceSend = &_SDCardForceSendCallback;
    }

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
static void _SDCardPrintCallback(struct SDCardFatFSFile* card)
{
    // It's not worth sending small amounts of data to sd card as
    // there is a large overhead for writing to the card.
    // TODO: Don't hardcode the 1024 value like that.
    if (SerialBufferSize(&card->buf) < 1024) return;

    card->shouldSendBuffer = 1;
}

static void _SDCardForceSendCallback(struct SDCardFatFSFile* card)
{
    card->shouldSendBuffer = 1;
}

int SDCardMount(struct SDCard* card)
{
    error(card != 0);

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
    error(card != 0);

    return f_unlink(fileName);
}

int SDCardOpen(struct SDCard* card, int fileNum, const char* fileName)
{
    error(card != 0);

    return f_open(&card->files[fileNum].Fil, fileName, FA_WRITE | FA_OPEN_ALWAYS);
}

int SDCardPrintf(struct SDCard* card, int fileNum, char* c, ...)
{
    error(card != 0);

    va_list ap;
    va_start(ap, c);
    SerialBufferPrintfVargs(&card->files[fileNum].buf, c, ap);
    va_end(ap);

    return 0;
}

// "Closes" a file temporary, but is reactivated by the next write/read.
// This makes the code after this call non-
int SDCardSync(struct SDCardFatFSFile* card)
{
    error(card != 0);

    return f_sync(&card->Fil);
}

int SDCardWrite(struct SDCardFatFSFile* card, void* buf, int num)
{
    error(card != 0);

    UINT wb;
    return f_write(&card->Fil, buf, num, &wb);
}

// Forces a write to the sd card.  Good for last data to
// ensure no data is left in the serial buffer without getting
// written. (Blocking).
void SDCardForceWriteBuffer(volatile struct SDCard* card, int fileNum)
{
    error(card != 0);

    __SDCardSendBuffer(&card->files[fileNum]);
/*    card->files[fileNum].shouldSendBuffer = 1;

    // Blocking until buffer is written to sd card.
    while (card->files[fileNum].shouldSendBuffer == 1);*/
}

int SDCardClose(struct SDCard* card, int file)
{
    error(card != 0);

    return f_close(&card->files[file].Fil);
}

struct SerialBuffer* SDCardGetBuffer(struct SDCard* card, int fileNum)
{
    error(card != 0);

    return &card->files[fileNum].buf;
}

// TODO: Make this not a define statement as this is crap.
#define BUF_SIZE 2024

void __SDCardSendBuffer(volatile struct SDCardFatFSFile* card)
{
    volatile static int lock = 0;

    // Ensure multiple files can't send something to the buffer at the same time!
    while (lock == 1);
    lock = 1;

    uint8_t buf[BUF_SIZE];
    uint16_t index = 0;

    while (SerialBufferIsEmpty(&card->buf) == 0)
    {
        buf[index] = SerialBufferPop(&card->buf);
        index++;

        if (index >= BUF_SIZE || SerialBufferIsEmpty(&card->buf))
        {
            // START CRITICAL SECTION.
            // If power were to be cut here, the data could be corrupt!
            SDCardWrite((struct SDCardFatFSFile*)card, buf, index);
            SDCardSync((struct SDCardFatFSFile*)card);
            // END CRITICAL SECTION
            index = 0;
        }
    }

    lock = 0;
}
