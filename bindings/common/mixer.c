#include "mixer.h"

int MixMotorValues(struct MotorMixer* mix,
    int throtValue, int pitchValue, int rollValue, int yawValue)
{
    return mix->throttle*throtValue
        + mix->pitch*pitchValue
        + mix->roll*rollValue
        + mix->yaw*yawValue;
}
