#include <xs1.h>
#include <stdio.h>

extern "C"
{
    // XMOS hack to get va_list to compile on .xc files correctly.
    #define va_list __VALIST
    #define __FUNCTION__ "Unknown"

    #include "serialBuffer.h"
    #include "uartOutput.h"
    #include "common/error.h"

    void UartOutputInit(struct UartOutput* output, uint32_t baud)
    {
        // Set baud rate.
        UartOutputSetBaud(output, baud);

        // Allocate memory for the buffer.
        int result = SerialBufferInit(&output->buf, 256);
        if (error(result == 0)) verbose("Ran out of memory!");
    }

    void UartOutputDeinit(struct UartOutput* output)
    {
        SerialBufferDeInit(&output->buf);
    }

    void UartOutputSetBaud(struct UartOutput* output, uint32_t baud)
    {
        if (baud == 0) return;

        output->baud = baud;
    }
}

// Outputs a UART stream to a given port, where the port should
// be 1 bit wide.  This UART stream is buffered and non-blocking.
// Since the uart task only pops from the buffer, and all other
// tasks should only push to the buffer, there should be no
// race conditions.
// TODO: Is this absolutely true?
[[combinable]]
void UartOutputTask(struct UartOutput* output, port outputPort)
{
    timer tim;
    uint32_t time;
    tim :> time;

    // By default, the signal is high until the start bit.
    outputPort <: 1;

    UartOutputSetBaud(output, 115200);
    int currentBit = -1;
    uint8_t curByte = 0;

    while (1)
    {
        select
        {
            case tim when timerafter(time) :> void: unsafe
            {
                // Update timer
                uint32_t waitTime = 100000000 / output->baud;
                time += waitTime;

                // Start bit (if data is available)
                if (currentBit == -1)
                {
                    int16_t popped =
                        SerialBufferPop((struct SerialBuffer*)&output->buf);

                    // Start Bit
                    if (popped != -1)
                    {
                        curByte = (uint8_t)popped;
                        currentBit = 0;
                        outputPort <: 0;
                    }
                }
                // Data Bits
                else if (currentBit < 8)
                {
                    int bit = (curByte >> currentBit) & 0x01;
                    currentBit++;
                    outputPort <: bit;
                }
                // Stop Bit
                else
                {
                    currentBit = -1;
                    outputPort <: 1;
                }
                break;
            }
        }
    }
}
