#include <xs1.h>
#include <stdio.h>

// It's important normal header files are included before
// xmos file as the compiler can complain about safe vs
// unsafe pointers (xmos pls).
extern "C"
{
    #define va_list __VALIST
    #include "sensors/imu/MPU/MPU6500.h"
    #include "sensors/magnetometer/MAG3110/MAG3110.h"
    #include "sensors/magnetometer/LSM303/LSM303.h"
    #include "sensors/barometer/BMP280/BMP280.h"
    #include "common/serialBuffer.h"
    #include "common/uartOutput.h"
    #include "filters/IMU/kalmanFilterSingleAxis.h"
    #include "filters/IMU/complementaryFilter.h"
    #include "filters/AHRS/MadgwickFilter.h"
    #include "sensors/gps/ultraTrackers/ultraTrackers.h"
}

// Include xmos specific binding header files.
#include "bindings.h"

// System level variables.
// These variables have to be global so that they can be
// passed between the many different threads.  These
// variables shouldn't be accessed directly in any other
// part of the code, rather passed by pointer.
struct MPU6500 imu;
struct LSM303 mag;
struct BMP280 bar;

struct UartOutput uart1;
port uart1Port = XS1_PORT_1F; // Uart 1 port

struct I2CDevicePortData imuI2cPort = {
    XS1_PORT_1J,
    XS1_PORT_1K,
    1000, // 100Khz
};

void sensorThread(void)
{
    // Global pointers.
    struct UartOutput* unsafe pUart1;
    struct MPU6500* unsafe pImu;

    /*struct ComplementaryFilter cfPitch;
    struct ComplementaryFilter cfRoll;
    ComplementaryFilterInit(&cfPitch, 0.995);
    ComplementaryFilterInit(&cfRoll, 0.995);*/
    struct KalmanFilterSingleAxis kfPitch;
    struct KalmanFilterSingleAxis kfRoll;
    KalmanFilterSingleAxisInit(&kfPitch, 0.001, 0.003, 0.03);
    KalmanFilterSingleAxisInit(&kfRoll, 0.001, 0.003, 0.03);
    struct MadgwickFilter mf;
    MadgwickFilterInit(&mf);

    unsafe
    {
        pUart1 = &uart1;
        pImu = &imu;

        double pitch = 0;
        double roll = 0;
        double yaw = 0;

        while (1==1)
        {
            MPU6500Sample(pImu);

            double ax = pImu->accelData.x;
            double ay = pImu->accelData.y;
            double az = pImu->accelData.z;
            double gx = pImu->gyroData.x;
            double gy = pImu->gyroData.y;
            double gz = pImu->gyroData.z;

            double pitchAngle =
                -atan2(ax, sqrt(ay*ay + az*az)) * 57.2958;

            double rollAngle =
                -atan2(ay, sqrt(ax*ax + az*az)) * 57.2958;

            MadgwickFilterUpdateIMU(&mf, ax, ay, az, 0, 0, 0);

            double q0 = mf.q0;
            double q1 = mf.q1;
            double q2 = mf.q2;
            double q3 = mf.q3;

            double pitch = atan2(2*(q0*q1 + q2*q3), 1-2*(q1*q1 + q2*q2));
            double roll = asin(2 *(q0*q2 - q3*q1));
            double yaw = 0;//atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
            //KalmanFilterSingleAxisUpdate(&kfPitch, pitchAngle, pImu->gyroData.y, 0.01);
            //KalmanFilterSingleAxisUpdate(&kfRoll, rollAngle, -pImu->gyroData.x, 0.01);
            //ComplementaryFilterUpdate(&cfPitch, pitchAngle, pImu->gyroData.y, 0.01);
            //ComplementaryFilterUpdate(&cfRoll, rollAngle, -pImu->gyroData.x, 0.01);

            UartOutputSendString(pUart1, "!ANG:");
            UartOutputSendDouble(pUart1, roll*RADTODEG); // Roll
            UartOutputSendChar(pUart1, ',');
            UartOutputSendDouble(pUart1, pitch*RADTODEG);//kfPitch.state[0]);
            UartOutputSendChar(pUart1, ',');
            UartOutputSendDouble(pUart1, yaw*RADTODEG);
            UartOutputSendChar(pUart1, '\n');

            delay_milliseconds(10);
        }
    }
}

in port ultraPort = XS1_PORT_4C;
out port ledPort = XS1_PORT_1A;
void UltrasonicTrackers(void)
{
    int data;
    struct UartOutput* unsafe pUart1;
    /*uint64_t timeLeft = 0, timeRight = 0;

    int leftTrigger = 0;
    int rightTrigger = 0;
    int shouldPrint = 0;*/

    printf("Ultrasonic Trackers\n");

    timer tim;
    uint32_t time;
    tim :> time;

    unsafe
    {
        pUart1 = &uart1;
        //UartOutputSendString(pUart1, "Start\n");

        struct UltraTrackers trackers;
        UltraTrackersInit(&trackers, 155, 190, 150);
        UltraTrackersSetOutput(&trackers, &pUart1->buf);

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

            /*if ((data & 0x01) == 0)
            {
                if (leftTrigger == 0)
                {
                    timeRight = SystemTime();
                    leftTrigger = 1;
                    shouldPrint++;
                }
            }

            if ((data & 0x04) == 0)
            {
                if (rightTrigger == 0)
                {
                    timeLeft = SystemTime();
                    shouldPrint++;
                    rightTrigger = 1;
                }
            }

            if (shouldPrint > 1)
            {
                int diff = timeRight - timeLeft;

                if (diff < 1000 && diff > -1000)
                {
                    // 0.34029 mm / sec * timeDiff sec / 140mm between sensors;
                    double yaw = asin(0.34029 * diff / 150) * RADTODEG;

                    UartOutputSendString(pUart1, "!ANG:0.0,0.0,");
                    UartOutputSendDouble(pUart1, yaw);
                    UartOutputSendChar(pUart1, '\n');
                    delay_milliseconds(20);
                }

                rightTrigger = 0;
                leftTrigger = 0;
                shouldPrint = 0;
            }*/
        }
    }
}

int main()
{
    // To pass pointers around different threads, we have to
    // use unsafe pointers (required by XMOS).
    struct UartOutput* unsafe pUart1;
    struct LSM303* unsafe pMag;
    struct BMP280* unsafe pBar;
    struct MPU6500* unsafe pImu;

    unsafe
    {
        pUart1 = &uart1;
        pMag = &mag;
        pBar = &bar;
        pImu = &imu;
    };

    // Initiate hardware.
    int mpuResult = MPU6500Init(pImu, &imuI2cPort);
    int magResult = LSM303Init(pMag, &imuI2cPort);
    int barResult = BMP280Init(pBar, &imuI2cPort);
    BMP280Enable(pBar);
    LSM303Enable(pMag);

    if (mpuResult == SENSOR_CONNECTED)
        printf("MPU6500 Connected\n");
    else
        printf("MPU6500 Missing!\n");

    if (magResult == SENSOR_CONNECTED)
        printf("LSM303 Connected\n");
    else
        printf("LSM303 Missing!\n");

    if (barResult == SENSOR_CONNECTED)
        printf("BAR280 Connected\n");
    else
        printf("BAR280 Missing!\n");

    unsafe
    {
        par
        {
            // Updates the sensors at a given frequency.
            //sensorThread();

            // Updates System Time
            SystemTimeTask();

            UltrasonicTrackers();

            // Outputs UART data to a specific pin.
            UartOutputTask((struct UartOutput*)pUart1, uart1Port);
        }
    }
    return 0;
}
