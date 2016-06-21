#ifndef __ULTRA_TRACKERS_XMOS_H__
#define __ULTRA_TRACKERS_XMOS_H__

#include <xs1.h>
#include <stdio.h>

extern "C"
{
    #define va_list __VALIST
    #include "serialBuffer.h"
    #include "ultraTrackers.h"
}

void UltrasonicTrackers(struct SerialBuffer* buf);

#endif
