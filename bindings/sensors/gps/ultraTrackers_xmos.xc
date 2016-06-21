#include "ultraTrackers_xmos.h"

in port ultraPort = XS1_PORT_4C;
out port ledPort = XS1_PORT_1A;

void UltrasonicTrackers(struct SerialBuffer* buf)
{
    int data;
    timer tim;
    uint32_t time;
    tim :> time;

    unsafe
    {
        struct UltraTrackers trackers;
        UltraTrackersInit(&trackers, 155, 190, 150);
        UltraTrackersSetOutput(&trackers, buf);

        while (1==1)
        {
            select
            {
                case tim when timerafter(time) :> void:
                    time += 100;
                    ultraPort :> data;

                    int result = UltraTrackersUpdate(&trackers,
                        (data & 0x01) == 0,
                        (data & 0x02) == 0,
                        (data & 0x04) == 0,
                        0);

                    // Delay 15ms before next pulse.
                    if (result == 1)
                        time += 1500000;

                break;
            }
        }
    }
}
