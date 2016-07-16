#ifndef __HCSR04_XMOS_H__
#define __HCSR04_XMOS_H__

#include <xs1.h>

extern "C"
{
    #include "sensors/ultrasonic/HCSR04/HCSR04.h"
}

struct HCSR04PinData
{
    port trigPort;
    port echoPort;
};

[[combinable]]
void HCSR04Task(struct HCSR04* ultra, port echoPin);

#endif
