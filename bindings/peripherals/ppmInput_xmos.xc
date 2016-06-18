#include "ppmInput_xmos.h"
#include "common.h"
#include <xs1.h>

port ppmPort = XS1_PORT_1C;

void PPMCallback(struct PPMInput* ppm)
{
    uint64_t currentTime = SystemTime();
    uint64_t diffTime = currentTime - ppm->_lastEdgeTime;
    uint64_t lastEdgeTime = ppm->_lastEdgeTime;

    ppm->_lastEdgeTime = currentTime;

    if (lastEdgeTime == 0) return;

    // Something went horribly wrong, no wait should be this long (100ms).
    if (diffTime > 100000)
    {
        // If the current frame is corrupt, the next frame (which wouldn't be a real frame anyways)
        // could be corrupt, so be sure to skip it.
        ppm->_dataCorrupt = 2;
        ppm->_corruptDataCounter++;
    }
    // Look for SYNC pulse
    if (diffTime > MIN_PPM_SYNC_PULSE)
    {
        ppm->_edgeTimes[ppm->_index] = diffTime;
        ppm->_edgeTimes[ppm->_index+1] = 0;

        if (ppm->_dataCorrupt <= 0)
        {
            // The final channel length is space between three pulses.
            // eg. v 400us ^ 800us v
            //     where the channel now equals 1200us.
            int i;
            for (i = 0; i < ppm->_index; i+=2)
            {
                ppm->channels[i/2] = ppm->_edgeTimes[i] + ppm->_edgeTimes[i+1];
            }

            ppm->updateCount++;
        }
        else
        {
            ppm->_dataCorrupt--;
        }

        ppm->_index = 0;
    }
    // Save the pulse time.
    else if (ppm->_index < MAX_CHANNELS)
    {
        ppm->_edgeTimes[ppm->_index] = diffTime;
        ppm->_index += 1;
    }
}

[[combinable]]
void ppmInputTask(struct PPMInput* ppm)
{
    uint8_t pinState;
    ppmPort :> pinState;

    ppm->_corruptDataCounter = 0;
    ppm->_index = 0;
    ppm->updateCount = 0;

    while (1)
    {
        select
        {
            // Fired on pin change
            case ppmPort when pinsneq(pinState) :> pinState:
                PPMCallback(ppm);
                //testingPort <: pinState;
                break;
        }
    }
}
