#include <xs1.h>
#include <stdio.h>

// It's important normal header files are included before
// xmos file as the compiler can complain about safe vs
// unsafe pointers (xmos pls).
extern "C"
{
    #define va_list __VALIST // XMOS hack for getting va_list to work correctly.
    #define __FUNCTION__ "Unknown"

    #include "common/serialBuffer.h"
    #include "common/uartOutput.h"
    #include "common/mixer.h"
    #include "peripherals/pwmOutput.h"
    #include "filters/IMU/kalmanFilterSingleAxis.h"
    #include "filters/IMU/complementaryFilter.h"
    #include "filters/AHRS/MadgwickFilter.h"
    #include "filters/recursive/biquadLPF.h"
    #include "feedback/PIDBasic.h"
    #include "feedback/rate/rateModeBasic.h"
    #include "feedback/angle/angleModeBasic.h"
    #include "sensors/imu/MPU/MPU6500.h"
    #include "sensors/magnetometer/MAG3110/MAG3110.h"
    #include "sensors/magnetometer/LSM303/LSM303.h"
    #include "sensors/barometer/BMP280/BMP280.h"
    #include "peripherals/sdcard.h"

    #include "ff.h"
}

// Include xmos specific binding header files.
#include "bindings.h"

struct UartOutput uart1;
struct PPMInput ppm;
struct SDCard card;
struct LSM303 mag;
struct BMP280 bar;
struct MPU6500 imu;
struct PWMOutput pwm;
struct HCSR04 ultra;

port uart1Port = XS1_PORT_1F;
port pwmPort = XS1_PORT_4E;
port ppmPort = XS1_PORT_1G;
port ledPort = XS1_PORT_1A;

struct I2CDevicePortData imuI2cPort = {
    XS1_PORT_1J,
    XS1_PORT_1K,
    1000, // 100Khz
    };

struct HCSR04PinData hcsrPorts = {
    XS1_PORT_1P,
    XS1_PORT_1I
};

void stablizeFlightMode(
    struct MPU6500* pImu,
    struct PPMInput* pPpm,
    struct PWMOutput* pwm,
    struct UartOutput* pUart1,
    struct SerialBuffer* pDl){ unsafe
{
    // Maps the controller channels to the commands they represent.
    const int RC_THROTTLE_CH = 2;
    const int RC_PITCH_CH = 1;
    const int RC_ROLL_CH = 0;
    const int RC_YAW_CH = 3;
    const int RC_FLIGHT_MODE_CH = 5;

    // Maps axises with angular velocity max rotations (when the
    // controller stick is at max value)
    const double RC_PITCH_VEL = 1.74533*2;
    const double RC_ROLL_VEL = 1.74533*2;
    const double RC_YAW_VEL = 1.74533*2;

    //
    const double RC_PITCH_POS = 3.00;
    const double RC_ROLL_POS = 3.00;
    const double RC_YAW_POS = 1.00;

    // Maps the direction multiplier to the direction of turn of the
    // quadcopter. Ie. a 1 will say "Pitch foward" is the correct
    // direction, whereas -1 will say "pitch forward" is going the
    // opposite direction.
    const int RC_PITCH_FORWARD_DIR = 1;
    const int RC_ROLL_RIGHT_DIR = 1;
    const int RC_YAW_RIGHT_DIR = -1;

    // Maps the direction multiplier to the direction the acceleration.
    // Ie. if the quadcopter's accelerometer returns a positive value
    // when accelerating upwards, RC_ACC_UP_DIR will be 1.  If the
    // acccelerometer returns a negative value, -1.
    const int RC_ACC_FORWARD_DIR = 1;
    const int RC_ACC_RIGHT_DIR = -1;

    // Maps the gyro x,y,z output to pitch,roll,yaw.
    const int RC_GYRO_PITCH = VECTOR_Y;
    const int RC_GYRO_ROLL = VECTOR_X;
    const int RC_GYRO_YAW = VECTOR_Z;

    // Maps the accelerometer x,y,z output to up, forward, right.
    const int RC_ACC_UP = VECTOR_Z;
    const int RC_ACC_FORWARD = VECTOR_X;
    const int RC_ACC_RIGHT = VECTOR_Y;

    // What values the sticks return when centered.
    const int RC_STICK_BIAS_PITCH = 1503;
    const int RC_STICK_BIAS_ROLL = 1494;
    const int RC_STICK_BIAS_YAW = 1489;

    // Props to PWM motor number.
    const int PWM_PROP_TR = 0;
    const int PWM_PROP_TL = 1;
    const int PWM_PROP_BL = 2;
    const int PWM_PROP_BR = 3;

    // Maps the commands to the prop values.
    struct MotorMixer QUAD_MIXER[4] = {
        // Throttle, pitch, roll, yaw
        {1,    1,    -1,    -1}, // BR prop
        {1,   -1,    -1,     1}, // FR prop
        {1,    1,     1,     1}, // BL prop
        {1,   -1,     1,    -1}  // FL prop
    };

    SerialBufferPrintf(pDl, "Initializing rate mode controllers\n");
    SerialBufferForceSend(pDl);

    // Kalman filter for estimating true quadcopter angle.
    struct KalmanFilterSingleAxis kfPitch;
    struct KalmanFilterSingleAxis kfRoll;
    KalmanFilterSingleAxisInit(&kfPitch, 0.001, 0.003, 0.03);
    KalmanFilterSingleAxisInit(&kfRoll, 0.001, 0.003, 0.03);

    // Rate Mode Feedback
    struct RateModeBasic rateMode;
    RateModeBasicInit(&rateMode,
        35, 80, 0, // Pitch PID
        35, 60, 0, // Roll PID
        100, 30, 0); // Yaw PID

    struct BiquadLPF gyroPitchFilter, gyroRollFilter;
    BiquadLPFInit(&gyroPitchFilter, 333.33, 40);
    BiquadLPFInit(&gyroRollFilter, 333.33, 40);

    // Angle stablize feedback.
    struct AngleModeBasic angleMode;
    AngleModeBasicInit(&angleMode,
        2, 0, 0, // Pitch PID
        2, 0, 0, // Roll PID
        0, 0, 0); // Yaw PID

    // Allow user to plug in battery and not mess up calibration
    delay_milliseconds(3000);

    // Calibrate gyro.
    int i;
    double gyroXBias = 0, gyroYBias = 0, gyroZBias = 0;
    double accXBias = 0, accYBias = 0, accZBias = 0;
    for (i = 0; i < 100; i++)
    {
        MPU6500Sample(pImu);
        gyroXBias += pImu->gyroData.x;
        gyroYBias += pImu->gyroData.y;
        gyroZBias += pImu->gyroData.z;

        // This assumes that the accelerometer is perfectly up&down.
        accXBias += pImu->accelData.x;
        accYBias += pImu->accelData.y;
        accZBias += pImu->accelData.z - 1;

        delay_milliseconds(10);
    }

    // Set gyro biases.  Everytime the gyroscope is read again, it will have
    // these biases subtracted from the sensor value.
    MPU6500SetGyroBias(pImu, gyroXBias/100.0, gyroYBias/100.0, gyroZBias/100.0);
    //MPU6500SetAccelBias(pImu, accXBias/100.0, accYBias/100.0, accZBias/100.0);

    // Calibrate controller stick's "Zero" point by using predefined values.
    PPMInputSetBias(pPpm, RC_THROTTLE_CH, 1000);
    PPMInputSetBias(pPpm, RC_PITCH_CH, RC_STICK_BIAS_PITCH);
    PPMInputSetBias(pPpm, RC_ROLL_CH, RC_STICK_BIAS_ROLL);
    PPMInputSetBias(pPpm, RC_YAW_CH, RC_STICK_BIAS_YAW);

    double dT = 0.003;

    int tmp = 0;

    uint64_t targetTime = SystemTime();

    // Main Update Loop
    while (1==1)
    {
        uint64_t loopStart = SystemTime();

        // Get values from ppm input.
        int throttleValue = PPMInputGetValue(pPpm, RC_THROTTLE_CH);
        int pitchValue = PPMInputGetValue(pPpm, RC_PITCH_CH);
        int rollValue = PPMInputGetValue(pPpm, RC_ROLL_CH);
        int yawValue = PPMInputGetValue(pPpm, RC_YAW_CH);
        int flightModeValue = PPMInputGetValue(pPpm, RC_FLIGHT_MODE_CH);

        unsafe
        {
           struct HCSR04* x = &ultra;
           UartOutputPrintf(pUart1, "%f\n", x->distance);
        }

        // Quadcopter is "Armed" when flight mode switch is triggered high.
        if (flightModeValue > 1700)
        {
            MPU6500Sample(pImu);

            // Get values from gyroscope (rad/sec)
            double gyroPitchVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_PITCH);
            double gyroRollVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_ROLL);
            double gyroYawVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_YAW);

            gyroPitchVel = BiquadLPFSample(&gyroPitchFilter, gyroPitchVel);
            gyroRollVel = BiquadLPFSample(&gyroRollFilter, gyroRollVel);

            // Get values from accelerometer (g's)
            double accUp = Vector3DGetValue(&pImu->accelData, RC_ACC_UP);
            double accFor = Vector3DGetValue(&pImu->accelData, RC_ACC_FORWARD);
            double accRight = Vector3DGetValue(&pImu->accelData, RC_ACC_RIGHT);

            // Calculate pitch/roll angles from accelerometer data. (radians)
            double pitchForwardAngle =
                -atan2(accFor * RC_ACC_FORWARD_DIR,
                    sqrt(accFor * accFor + accUp * accUp));
            double rollRightAngle =
                -atan2(accRight * RC_ACC_RIGHT_DIR,
                    sqrt(accRight * accRight + accUp * accUp));

            // Use Kalman filter to combine the gyro and accelerometer data
            KalmanFilterSingleAxisUpdate(&kfPitch, pitchForwardAngle, gyroPitchVel, dT);
            KalmanFilterSingleAxisUpdate(&kfRoll, rollRightAngle, gyroRollVel, dT);

            double pitchAngle = kfPitch.state[0];
            double rollAngle = kfRoll.state[0];

            // Calculate target angular velocities from controller stick values.
            // (rad/sec)
            double targetPitchForwardVel = pitchValue / 500.0 * RC_PITCH_VEL;
            double targetRollRightVel = rollValue / 500.0 * RC_ROLL_VEL;
            double targetYawRightVel = yawValue / 500.0 * RC_YAW_VEL;

            double targetPitchForwardPos = pitchValue / 500.0 * 0.7;
            double targetRollRightPos = rollValue / 500.0 * 0.7;

            AngleModeBasicUpdate(&angleMode,
                targetPitchForwardPos,
                targetRollRightPos,
                0,
                pitchAngle,
                rollAngle,
                0,
                dT);

            RateModeBasicUpdate(&rateMode,
                //targetPitchForwardVel,
                //targetRollRightVel,
                angleMode.pitchResult,
                angleMode.rollResult,
                targetYawRightVel,
                gyroPitchVel * RC_PITCH_FORWARD_DIR,
                gyroRollVel * RC_ROLL_RIGHT_DIR,
                gyroYawVel * RC_YAW_RIGHT_DIR,
                dT);

            int pitchPWM = rateMode.pitchResult;
            int rollPWM = rateMode.rollResult;
            int yawPWM = rateMode.yawResult;

            MixMotorValues(QUAD_MIXER,
                sizeof(QUAD_MIXER)/sizeof(struct MotorMixer),
                throttleValue, pitchPWM, rollPWM, yawPWM);

            PWMOutputSetMotor(pwm, PWM_PROP_TR, QUAD_MIXER[MIXER_PROP_FR].result);
            PWMOutputSetMotor(pwm, PWM_PROP_TL, QUAD_MIXER[MIXER_PROP_FL].result);
            PWMOutputSetMotor(pwm, PWM_PROP_BR, QUAD_MIXER[MIXER_PROP_BR].result);
            PWMOutputSetMotor(pwm, PWM_PROP_BL, QUAD_MIXER[MIXER_PROP_BL].result);

            uint64_t loopEnd = SystemTime();
            SerialBufferPrintf(pDl, "%f %f %f %f %f %f %f %f %f %f %f %d\n",
                targetRollRightVel, gyroRollVel, targetRollRightPos, rollAngle, pitchAngle, angleMode.rollResult, rateMode.rollResult,
                rateMode.rollPID.error * rateMode.rollPID.kP, rateMode.rollPID.integral*rateMode.rollPID.kI, rateMode.yawPID.integral, (int)(loopEnd - loopStart));
        }
        // Kill the motors if something goes wrong and the switch is switched off.
        else
        {
            int i;
            for (i = 0; i < 4; i++) PWMOutputSetMotor(pwm, i, 0);
        }

        unsafe
        {
            struct HCSR04* pUltra;
            pUltra = &ultra;
            HCSR04Trigger(pUltra);
        }

        // Determine how long to wait until the next loop should execute.
        targetTime += dT * 1000000;
        int delayAmount = targetTime - SystemTime();

        // If the delay is less then zero, then it's time to execute again.
        if (delayAmount > 0)
            delay_microseconds(targetTime - SystemTime());
    }
}}

int main()
{
    printf("mFlight Started\n");

    struct SerialBuffer* unsafe file0;

    // Setup SD card set to data.txt as a datalogger
    SDCardInit(&card, 4096);
    file0 = SDCardGetBuffer(&card, 0);

    UartOutputInit(&uart1, 115200);

    if (!SDCardMount(&card))
    {
        printf("F = %ld, %ld\n", card.freeSpace, card.totalSpace);

        // Delete and re-create data.txt
        SDCardUnlink(&card, "data.txt");
        SDCardUnlink(&card, "error.txt");
        SDCardOpen(&card, 0, "data.txt");
        SDCardOpen(&card, 1, "error.txt");

        // Sets the global error output to the sd card error.txt file.
        SetErrorOutput(SDCardGetBuffer(&card, 1));
    }
    else
    {
        printf("SD Card Error\n");
    }

    // Initiate hardware.
    int mpuResult = MPU6500Init(&imu, &imuI2cPort);
    int magResult = LSM303Init(&mag, &imuI2cPort);
    int barResult = BMP280Init(&bar, &imuI2cPort);
    BMP280Enable(&bar);
    LSM303Enable(&mag);

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

    PWMOutputInit(&pwm, &pwmPort, 400);
    PPMInputInit(&ppm, &ppmPort);
    HCSR04Init(&ultra, &hcsrPorts);

    unsafe
    {
        // Casting uart1 to a pointer gets around the "parallel usage rules"
        // error in the XCC compiler.
        struct UartOutput* unsafe pUart1;
        struct PWMOutput* unsafe pPwm;
        struct PPMInput* unsafe pPpm;
        struct SerialBuffer* unsafe pDl;
        struct SDCard* unsafe pCard;
        struct HCSR04* unsafe pUltra;
        pUart1 = &uart1;
        pPwm = &pwm;
        pPpm = &ppm;
        pCard = &card;
        pDl = file0;
        pUltra = &ultra;

        par
        {
            // Outputs UART data to a specific pin.
            UartOutputTask((struct UartOutput*)pUart1, uart1Port);

            // PPM Input Task
            ppmInputTask((struct PPMInput*)pPpm, ppmPort);

            // PWM Task
            pwmTask(&pwm);

            // Ultrasonics
            HCSR04Task((struct HCSR04*)pUltra, hcsrPorts.echoPort);

            // Updates System Time
            SystemTimeTask();

            SDCardTask((struct SDCard*)pCard);

            stablizeFlightMode(
                &imu,
                (struct PPMInput*)pPpm,
                (struct PWMOutput*)pPwm,
                (struct UartOutput*)pUart1,
                (struct SerialBuffer*)pDl);
        }
    }
    return 0;
}
