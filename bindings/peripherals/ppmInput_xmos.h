#ifndef __PPM_INPUT_XMOS_H__
#define __PPM_INPUT_XMOS_H__

extern "C"
{
    #include "ppmInput.h"
}

[[combinable]]
void ppmInputTask(struct PPMInput* ppm, port ppmPort);

#endif
