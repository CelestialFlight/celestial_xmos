extern "C"
{
    #include "sensors/ultrasonic/HCSR04/HCSR04.h"
    #define va_list __VALIST
    #define __FUNCTION__ "Unkown"
    #include "common/error.h"
    #include "common/common.h"
}

#include "HCSR04_xmos.h"

// Function prototypes (need to implemented in bindings).
void HCSR04Init(struct HCSR04* unsafe ultra, void* unsafe pinData)
{
    error(pinData != 0);
    error(ultra != 0);

    unsafe
    {
        ultra->pinData = pinData;
        ultra->distance = 0;
    }
}

void HCSR04Trigger(struct HCSR04* unsafe ultra)
{
    // Defensive check.
    if (error(ultra != 0)) return;

    unsafe
    {
        struct HCSR04PinData* pins = ultra->pinData;

        if (pins != 0)
        {
            pins->trigPort <: 1;
            delay_microseconds(30);
            pins->trigPort <: 0;
        }
    }
}

[[combinable]]
void HCSR04Task(struct HCSR04* ultra, port echoPin)
{
    // Defensive check.
    if (error(ultra != 0)) return;

    uint8_t pinState;
    echoPin :> pinState;

    uint64_t startTime = SystemTime();

    unsafe { HCSR04Trigger(ultra); }

    while (1)
    {
        select
        {
            case echoPin when pinsneq(pinState) :> pinState: unsafe
            {
                if (pinState == 0)
                {
                    uint64_t endTime = SystemTime();
                    uint64_t splitTime = endTime - startTime;

                    ultra->distance = splitTime / 58.0;
                }
                else
                {
                    startTime = SystemTime();
                }
                break;
            }
        }
    }
}
