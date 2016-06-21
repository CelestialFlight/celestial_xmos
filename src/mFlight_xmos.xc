#include <xs1.h>
#include <stdio.h>

// It's important normal header files are included before
// xmos file as the compiler can complain about safe vs
// unsafe pointers (xmos pls).
extern "C"
{
    #define va_list __VALIST // XMOS hack for getting va_list to work correctly.
    #include "common/serialBuffer.h"
    #include "common/uartOutput.h"
    #include "common/mixer.h"
    #include "peripherals/pwmOutput.h"
    #include "filters/IMU/kalmanFilterSingleAxis.h"
    #include "filters/IMU/complementaryFilter.h"
    #include "filters/AHRS/MadgwickFilter.h"
    #include "feedback/PIDBasic.h"
    #include "sensors/imu/MPU/MPU6500.h"
    #include "sensors/magnetometer/MAG3110/MAG3110.h"
    #include "sensors/magnetometer/LSM303/LSM303.h"
    #include "sensors/barometer/BMP280/BMP280.h"
}

// Include xmos specific binding header files.
#include "bindings.h"

struct UartOutput uart1;
struct PPMInput ppm;

port uart1Port = XS1_PORT_1F;
port pwmPort = XS1_PORT_4E;
port ppmPort = XS1_PORT_1G;

struct I2CDevicePortData imuI2cPort = {
    XS1_PORT_1J,
    XS1_PORT_1K,
    1000, // 100Khz
};

void stablizeFlightMode(
    struct MPU6500* pImu,
    struct PPMInput* pPpm,
    struct PWMOutput* pwm,
    struct UartOutput* pUart1){ unsafe
{
    // Maps the controller channels to the commands they represent.
    const int RC_THROTTLE_CH = 2;
    const int RC_PITCH_CH = 1;
    const int RC_ROLL_CH = 0;
    const int RC_YAW_CH = 3;
    const int RC_FLIGHT_MODE_CH = 5;

    // Maps axises with angular velocity max rotations (when the
    // controller stick is at max value)
    const double RC_PITCH_VEL = 1.74533;
    const double RC_ROLL_VEL = 1.74533;
    const double RC_YAW_VEL = 1.74533;

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

    /*struct ComplementaryFilter cfPitch;
    struct ComplementaryFilter cfRoll;
    ComplementaryFilterInit(&cfPitch, 0.995);
    ComplementaryFilterInit(&cfRoll, 0.995);*/

    // Kalman filter for estimating true quadcopter angle.
    struct KalmanFilterSingleAxis kfPitch;
    struct KalmanFilterSingleAxis kfRoll;
    KalmanFilterSingleAxisInit(&kfPitch, 0.001, 0.003, 0.03);
    KalmanFilterSingleAxisInit(&kfRoll, 0.001, 0.003, 0.03);

    // Rate PID's for angular velocity control.
    struct PIDBasic ratePitchPID;
    struct PIDBasic rateRollPID;
    struct PIDBasic rateYawPID;
    PIDBasicInit(&ratePitchPID, 50, 0, 0);
    PIDBasicInit(&rateRollPID, 50, 0, 0);
    PIDBasicInit(&rateYawPID, 50, 0, 0);

    UartOutputPrintf(pUart1, "Update Thread Loaded\n");

    // Allow user to plug in battery and not mess up calibration
    delay_milliseconds(500);

    // Calibrate gyro.
    int i;
    double xBias = 0, yBias = 0, zBias = 0;
    for (i = 0; i < 100; i++)
    {
        MPU6500Sample(pImu);
        xBias += pImu->gyroData.x;
        yBias += pImu->gyroData.y;
        zBias += pImu->gyroData.z;

        delay_milliseconds(10);
    }

    // Set gyro biases.  Everytime the gyroscope is read again, it will have
    // these biases subtracted from the sensor value.
    MPU6500SetGyroBias(pImu, xBias/100.0, yBias/100.0, zBias/100);

    // Calibrate controller stick's "Zero" point by using predefined values.
    PPMInputSetBias(pPpm, RC_THROTTLE_CH, 1000);
    PPMInputSetBias(pPpm, RC_PITCH_CH, RC_STICK_BIAS_PITCH);
    PPMInputSetBias(pPpm, RC_ROLL_CH, RC_STICK_BIAS_ROLL);
    PPMInputSetBias(pPpm, RC_YAW_CH, RC_STICK_BIAS_YAW);

    double dT = 0.01;

    int tmp = 0;

    // Main Update Loop
    while (1==1)
    {
        MPU6500Sample(pImu);

        // Get values from ppm input.
        int throttleValue = PPMInputGetValue(pPpm, RC_THROTTLE_CH);
        int pitchValue = PPMInputGetValue(pPpm, RC_PITCH_CH);
        int rollValue = PPMInputGetValue(pPpm, RC_ROLL_CH);
        int yawValue = PPMInputGetValue(pPpm, RC_YAW_CH);
        int flightModeValue = PPMInputGetValue(pPpm, RC_FLIGHT_MODE_CH);

        // Calculate target angular velocities from controller stick values.
        // (rad/sec)
        double targetPitchForwardVel = pitchValue / 500.0 * RC_PITCH_VEL;
        double targetRollRightVel = rollValue / 500.0 * RC_ROLL_VEL;
        double targetYawRightVel = yawValue / 500.0 * RC_YAW_VEL;

        // Get values from gyroscope (rad/sec)
        double gyroPitchVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_PITCH);
        double gyroRollVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_ROLL);
        double gyroYawVel = Vector3DGetValue(&pImu->gyroData, RC_GYRO_YAW);

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

        // Calculate angular velocity errors.
        double angVelPitchError =
            targetPitchForwardVel - gyroPitchVel * RC_PITCH_FORWARD_DIR;
        double angVelRollError =
            targetRollRightVel - gyroRollVel * RC_ROLL_RIGHT_DIR;
        double angVelYawError =
            targetYawRightVel - gyroYawVel * RC_YAW_RIGHT_DIR;

        // Quadcopter is "Armed" when flight mode switch is triggered high.
        if (flightModeValue > 1700)
        {
            int pitchPWM = PIDBasicUpdate(&ratePitchPID, angVelPitchError, dT);
            int rollPWM = PIDBasicUpdate(&rateRollPID, angVelRollError, dT);
            int yawPWM = PIDBasicUpdate(&rateYawPID, angVelYawError, dT);

            if (tmp++ % 4 == 0)
            /*UartOutputPrintf(pUart1, "!ANG:%f,%f,%f\n",
                    kfRoll.state[0] * 57.2958, kfPitch.state[0] * 57.2958, 0);*/
            UartOutputPrintf(pUart1, "::%f %f %f\n", angVelPitchError, angVelRollError, angVelYawError);

            int brPWM = MixMotorValues(&QUAD_MIXER[MIXER_PROP_BR],
                throttleValue, pitchPWM, rollPWM, yawPWM);
            int blPWM = MixMotorValues(&QUAD_MIXER[MIXER_PROP_BL],
                throttleValue, pitchPWM, rollPWM, yawPWM);
            int frPWM = MixMotorValues(&QUAD_MIXER[MIXER_PROP_FR],
                throttleValue, pitchPWM, rollPWM, yawPWM);
            int flPWM = MixMotorValues(&QUAD_MIXER[MIXER_PROP_FL],
                throttleValue, pitchPWM, rollPWM, yawPWM);

            PWMOutputSetMotor(pwm, PWM_PROP_TR, frPWM);
            PWMOutputSetMotor(pwm, PWM_PROP_TL, flPWM);
            PWMOutputSetMotor(pwm, PWM_PROP_BR, brPWM);
            PWMOutputSetMotor(pwm, PWM_PROP_BL, blPWM);
        }
        // Kill the motors if something goes wrong and the switch is switched off.
        else
        {
            int i;
            for (i = 0; i < 4; i++) PWMOutputSetMotor(pwm, i, 0);
        }

        delay_milliseconds(dT*1000);
        /*

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

        delay_milliseconds(10);*/
    }
}}

int main()
{
    struct LSM303 mag;
    struct BMP280 bar;
    struct MPU6500 imu;
    struct PWMOutput pwm;

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

    PWMOutputInit(&pwm, &pwmPort);
    PPMInputInit(&ppm, &ppmPort);

    unsafe
    {
        // Casting uart1 to a pointer gets around the "parallel usage rules"
        // error in the XCC compiler.
        struct UartOutput* unsafe pUart1;
        struct PWMOutput* unsafe pPwm;
        struct PPMInput* unsafe pPpm;
        pUart1 = &uart1;
        pPwm = &pwm;
        pPpm = &ppm;

        par
        {
            // Outputs UART data to a specific pin.
            UartOutputTask((struct UartOutput*)pUart1, uart1Port);

            // PPM Input Task
            ppmInputTask((struct PPMInput*)pPpm, ppmPort);

            // PWM Task
            pwmTask(&pwm);

            // Updates System Time
            SystemTimeTask();

            stablizeFlightMode(
                &imu,
                (struct PPMInput*)pPpm,
                (struct PWMOutput*)pPwm,
                (struct UartOutput*)pUart1);
        }
    }
    return 0;
}
