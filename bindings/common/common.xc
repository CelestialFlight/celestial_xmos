#include "common_xmos.h"

#define TIMER_TICKS_PER_USEC 100

// Global system time.
uint64_t ustime;

// The next timer value for when the ustime should update.
uint32_t _nextTimeValue;

uint64_t SystemTime()
{
    unsafe
    {
        uint64_t* unsafe pUsTime;
        pUsTime = &ustime;
        return *pUsTime;
    }
}

void SystemWaitUntil(uint64_t waittime)
{
    // TODO: Implement this function.
}

// While this task can be combined, it causes
// the usec ticks to be really inaccurate
// TODO: Have the timer correct itself when
// a small delay occurs and causes a slightly
// longer delay then 1usec.
[[combinable]]
void SystemTimeTask(void)
{
    uint64_t* unsafe pUsTime;
    timer tim;

    unsafe
    {
        pUsTime = &ustime;
        *pUsTime = 0;
        _nextTimeValue = 0;
    }

    //int state = 0;

    while (1)
    {
        select
        {
            // Increment ustime counter by one every us
            // This is a very inefficient method, but since
            // timer tim can't be accessed globally, there
            // aren't any other options.
            case tim when timerafter(_nextTimeValue) :> void:
                unsafe
                {
                    _nextTimeValue += TIMER_TICKS_PER_USEC;
                    (*pUsTime) += 1;
                    //testPort <: state;
                    //state = !state;
                }
                break;
        }
    }

}
