#include <xs1.h>
#include <stdio.h>

#define TIMER_TICKS_PER_USEC 100

extern "C"
{
    #include "pwmOutput.h"
    #define va_list __VALIST
    #define __FUNCTION__ "Unkown"
    #include "common/error.h"

    void PWMOutputInit(struct PWMOutput* pwm, void* data, uint32_t updateFreq)
    {
        int i;
        for (i = 0; i < MAX_MOTORS; i++)
        {
            pwm->motors[i] = 0;
        }

        pwm->motorData = data;
        pwm->updateFrequency = updateFreq;
    }

    void PWMOutputSetMotor(struct PWMOutput* pwm, uint8_t motor, int16_t value)
    {
        // Defensive check
        if (motor >= MAX_MOTORS) return;

        // Clip value between min and max value
        if (value < MIN_MOTOR_VALUE) value = MIN_MOTOR_VALUE;
        else if (value > MAX_MOTOR_VALUE) value = MAX_MOTOR_VALUE;

        pwm->motors[motor] = value;
    }
}

[[combinable]]
void pwmTask(struct PWMOutput* pwm)
{
    //int16_t motors[MAX_MOTORS];
    timer tim;

    // Used for PWM signals
    uint32_t pwmTime[MAX_MOTORS];
    uint32_t startTime[MAX_MOTORS];
    uint32_t portValue = 0;

    // Reset all motors to default motor values
    int j;
    for (j = 0; j < MAX_MOTORS; j++)
    {
        pwm->motors[j] = MIN_MOTOR_VALUE;
        pwmTime[j] = 0;
        startTime[j] = 0;
    }

    // Test that the updateFrequencies are in spec.
    warning(pwm->updateFrequency > 50);
    warning(pwm->updateFrequency < 450);
    if (pwm->updateFrequency < 50 || pwm->updateFrequency > 450)
        pwm->updateFrequency = 50;

    // The period between each signal = 1,000,000usec / Freq
    // Ex. 1000000usec/50Hz = 20000usecs between pulses.
    uint32_t loopLength = 1000000/pwm->updateFrequency;

    // Determine which port to turn on and off.
    port* PWM_PORT;
    unsafe
    {
        PWM_PORT = (pwm->motorData);
    }

    while (1)
    {
        select
        {
            // Turn signal high or low depending on what state the signal
            // is in.
            case (size_t q = 0; q < MAX_MOTORS; q++)
                tim when timerafter(pwmTime[q]) :> void:

                // If signal is currently low, restart signal
                if ( ((portValue >> q) & 0x01) == 0)
                {
                    startTime[q] = (1000 + pwm->motors[q]) * TIMER_TICKS_PER_USEC;
                    pwmTime[q] += startTime[q];
                    portValue |= 1 << q;
                    *PWM_PORT <: portValue & 0xF;
                }
                else
                {
                    pwmTime[q] += loopLength*TIMER_TICKS_PER_USEC - startTime[q];
                    portValue &= ~(1 << q);
                    *PWM_PORT <: portValue & 0xF;
                }
                break;
        }
    }
}
