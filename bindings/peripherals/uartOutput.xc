#include <xs1.h>
#include <stdio.h>

extern "C"
{
    #include "uartOutput.h"
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
            case tim when timerafter(time) :> void:

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

extern "C"
{
    void UartOutputSetBaud(struct UartOutput* output, uint32_t baud)
    {
        if (baud == 0) return;

        output->baud = baud;
    }

    void UartOutputSendString(struct UartOutput* output, char* c)
    {
        while (*c != 0)
        {
            SerialBufferPush(&output->buf, *c);
            c++;
        }
    }

    void UartOutputSendChar(struct UartOutput* output, char c)
    {
        SerialBufferPush(&output->buf, c);
    }

    void UartOutputSendInt(struct UartOutput* output, int n)
    {
        char value[30];
        int index = 0;

        if (n < 0)
        {
            UartOutputSendChar(output, '-');
            n = -n;
        }
        else if (n == 0)
        {
            UartOutputSendChar(output, '0');
            return;
        }

        while (n != 0)
        {
            // Convert integer to character.
            value[index] = (char)(n % 10) + '0';
            index++;
            n /= 10;
        }

        while (index != 0)
        {
            index--;
            UartOutputSendChar(output, value[index]);
        }
    }

    void UartOutputSendDouble(struct UartOutput* output, double n)
    {
        char value[30];
        int index = 0;

        if (n < 0)
        {
            UartOutputSendChar(output, '-');
            n = -n;
        }
        else if (n == 0)
        {
            UartOutputSendChar(output, '0');
            return;
        }

        // Strip the 5 most significant decimals
        int numOfDecimals = 5;
        long noDecimals = (long)n;
        double onlyDecimals = n - noDecimals;

        int i;
        for (i = 0; i < numOfDecimals; i++)
        {
            noDecimals *= 10;
            onlyDecimals *= 10;
        }

        noDecimals += onlyDecimals;

        // Treat the number as a decimal.
        while (noDecimals != 0 || index < numOfDecimals)
        {
            value[index] = (char)(noDecimals % 10) + '0';
            index++;
            if (index == numOfDecimals) value[index++] = '.';
            noDecimals /= 10;
        }

        // Write all the characters to the uart buffer.
        while (index != 0)
        {
            index--;
            UartOutputSendChar(output, value[index]);
        }
    }
}


