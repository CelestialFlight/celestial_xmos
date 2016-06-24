#include <stdio.h>

extern "C"
{
    #define va_list __VALIST
    #include "sdcard.h"
    #include "ff.h"
}

static void __SDCardSendBuffer(volatile struct SDCard* unsafe card);

void SDCardTask(volatile struct SDCard* unsafe card)
{
    unsafe
    {
        while(1==1)
        {
            if (card->shouldSendBuffer == 1)
            {
                __SDCardSendBuffer(card);
                card->shouldSendBuffer = 0;
            }
        }
    }
}

#define BUF_SIZE 2048

static void __SDCardSendBuffer(volatile struct SDCard* unsafe card)
{
    unsafe
    {
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
                SDCardWrite(card, buf, index);
                SDCardSync(card);
                // END CRITICAL SECTION
                index = 0;
            }
        }
    }
}
