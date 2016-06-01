#include <xs1.h>

// It's important normal header files are included before
// xmos file as the compiler can complain about safe vs
// unsafe pointers (xmos pls).
extern "C"
{
    #include "sensors/MPU/MPU6500.h"
    #include "common/uartOutput.h"
}

// Include xmos specific binding header files.
#include "bindings.h"

// System level variables.
// These variables have to be global so that they can be
// passed between the many different threads.  These
// variables shouldn't be accessed directly in any other
// part of the code, rather passed by pointer.
struct MPU6500 imu;
struct UartOutput uart1;
port uart1Port = XS1_PORT_1F; // Uart 1 port

struct I2CDevicePortData imuI2cPort = {
    XS1_PORT_1J,
    XS1_PORT_1K,
    1000, // 100Khz
};

void sensorThread(void)
{
    struct UartOutput* unsafe pUart1;

    unsafe
    {
        pUart1 = &uart1;

        double pitch = 0;
        double roll = 0;
        double yaw = 0;

        while (1==1)
        {
            UartOutputSendString(pUart1, "!ANG:");
            UartOutputSendDouble(pUart1, pitch);
            UartOutputSendChar(pUart1, ',');
            UartOutputSendDouble(pUart1, roll);
            UartOutputSendChar(pUart1, ',');
            UartOutputSendDouble(pUart1, yaw);
            UartOutputSendChar(pUart1, '\n');

            pitch += 0.1;
            roll += 0.05;
            yaw += 0.01;

            delay_milliseconds(300);
        }
    }
}

int main()
{
    // To pass pointers around different threads, we have to
    // use unsafe pointers (required by XMOS).
    struct UartOutput* unsafe pUart1;
    unsafe { pUart1 = &uart1; };

    // Initiate hardware.
    MPU6500Init(&imu, &imuI2cPort);
    MPU6500Enable(&imu);

    par
    {
        // Updates the sensors at a given frequency.
        sensorThread();

        // Outputs UART data to a specific pin.
        UartOutputTask((struct UartOutput*)pUart1, uart1Port);
    }

    return 0;
}
