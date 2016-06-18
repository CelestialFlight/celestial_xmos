#include <xs1.h>
#include <stdio.h>

#define TIMER_TICKS_PER_USEC 100

port PWM_PORT = XS1_PORT_4A;

extern "C"
{
    #include "pwmOutput.h"

    void PWMOutputInit(struct PWMOutput* pwm, void* data)
    {
        int i;
        for (i = 0; i < MAX_MOTORS; i++)
        {
            pwm->motors[i] = 0;
        }

        pwm->motorData = data;
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

    while (1)
    {
        select
        {
            // Sets a value for a specific motor
            /*case i.setValue(uint8_t motor, int16_t value):

                // Defensive check
                if (motor >= MAX_MOTORS) break;

                // Clip value between min and max value
                if (value < MIN_MOTOR_VALUE) value = MIN_MOTOR_VALUE;
                else if (value > MAX_MOTOR_VALUE) value = MAX_MOTOR_VALUE;

                motors[motor] = value;

                break;*/

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
                    PWM_PORT <: portValue & 0xF;
                }
                else
                {
                    pwmTime[q] += 20000*TIMER_TICKS_PER_USEC - startTime[q];
                    portValue &= ~(1 << q);
                    PWM_PORT <: portValue & 0xF;
                }
                break;
        }
    }
}
