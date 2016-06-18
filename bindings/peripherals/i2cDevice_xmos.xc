#include "i2cDevice_xmos.h"

#include <xs1.h>
#include <xclib.h>
#include <stdio.h>

extern "C"
{
    #include "i2cDevice.h"

    int I2CDeviceInit(struct I2CDevice* device, uint8_t deviceAddress,
        uint32_t frequency, void* pinData)
    {
        device->frequency = frequency;
        device->deviceAddress = deviceAddress;
        xmosI2CDeviceInit(device, pinData);
        return 0;
    }

    int I2CDeviceReadRegister(
        struct I2CDevice* device, uint8_t reg, uint8_t data[], int nBytes)
    {
        return xmosI2CDeviceReadRegister(device, reg, data, nBytes);
    }

    int I2CDeviceWriteRegister(
        struct I2CDevice* device, uint8_t reg, uint8_t data[], int nBytes)
    {
        return xmosI2CDeviceWriteRegister(device, reg, data, nBytes);
    }

}

void xmosI2CDeviceInit(struct I2CDevice* unsafe device, void* unsafe pinData)
{
    unsafe
    {
        if (device == 0) return;
        if (pinData == 0) return;

        device->pinData = pinData;

        struct I2CDevicePortData* data = pinData;

        if (data == 0) return;
        data->scl :> void;
        data->sda :> void;

        // TODO: Make this changeable
        data->clockTicks = 1000; // 100Khz
    }
}

static void waitQuarter(struct I2CDevicePortData* unsafe i2c)
{
    unsafe
    {
        timer gt;
        int time;

        gt :> time;
        time += (i2c->clockTicks+3) >> 2;      // Round time upwards prior to division by 4.
        gt when timerafter(time) :> int _;
    }
}

static void waitHalf(struct I2CDevicePortData* unsafe i2c)
{
    unsafe
    {
        waitQuarter(i2c);
        waitQuarter(i2c);
    }
}

static int highPulse(struct I2CDevicePortData* unsafe i2c, int doSample)
{
    unsafe
    {
        int temp;
        if (doSample)
        {
            i2c->sda :> int _;
        }
        waitQuarter(i2c);
        i2c->scl when pinseq(1) :> void;
        waitQuarter(i2c);
        if (doSample)
        {
            i2c->sda :> temp;
        }
        waitQuarter(i2c);
        i2c->scl <: 0;
        waitQuarter(i2c);
        return temp;
    }
}

static void startBit(struct I2CDevicePortData* unsafe i2c, int waitForQuiet)
{
    unsafe
    {
        if (waitForQuiet)
        {
            timer t;
            int time;
            int done = 0;
            int sdaState = 1;
            int sclState = 1;
            t :> time;
            while(!done)
            {
                select
                {
                    case i2c->sda when pinsneq(sdaState) :> sdaState:
                        t :> time;
                        break;
                    case i2c->scl when pinsneq(sclState) :> sclState:
                        t :> time;
                        break;
                    case sdaState && sclState => t when timerafter(time+i2c->clockTicks) :> void:
                        done = 1;
                        break;
                }
            }
        }
        else
        {
            waitQuarter(i2c);
        }

        i2c->sda  <: 0;
        waitHalf(i2c);
        i2c->scl  <: 0;
        waitQuarter(i2c);
    }
}

static void stopBit(struct I2CDevicePortData* unsafe i2c)
{
    unsafe
    {
        i2c->sda <: 0;
        waitQuarter(i2c);
        i2c->scl when pinseq(1) :> void;
        waitHalf(i2c);
        i2c->sda :> void;
        waitQuarter(i2c);
    }
}

static int floatWires(struct I2CDevicePortData* unsafe i2c)
{
    unsafe
    {
        i2c->scl :> void;
        waitQuarter(i2c);
        i2c->sda :> void;
        return 0;
    }
}

static int tx8(struct I2CDevicePortData* unsafe i2c, unsigned data)
{
    unsafe
    {
        unsigned CtlAdrsData = ((unsigned) bitrev(data)) >> 24;
        for (int i = 8; i != 0; i--)
        {
            if (CtlAdrsData & 1)
            {
                if (highPulse(i2c, 1) == 0) // Somebody else pulling low.
                {
                    return 0;
                }
            }
            else
            {
                i2c->sda <: 0;
                highPulse(i2c, 0);
            }
            CtlAdrsData >>= 1;
        }
        return highPulse(i2c, 1) == 0;
    }
}

static int i2c_master_do_rx(int device, unsigned char data[], int nbytes, struct I2CDevicePortData* unsafe i2c)
{
    unsafe
    {
       int i;
       int rdData = 0;

       if (!tx8(i2c, device<<1 | 1)) return floatWires(i2c);
       for(int j = 0; j < nbytes; j++) {
           for (i = 8; i != 0; i--) {
               int temp = highPulse(i2c, 1);
               rdData = (rdData << 1) | temp;
           }
           if (j != nbytes - 1) {
               i2c->sda <: 0;
               highPulse(i2c, 0);
           } else {
               highPulse(i2c, 1);
           }
           data[j] = rdData;
       }
       stopBit(i2c);
       return 1;
    }
}

int xmosI2CDeviceReadRegister(
    struct I2CDevice* unsafe device, uint8_t reg, uint8_t data[], int nbytes)
{
    unsafe
    {
        if (device == 0) return -1;

        struct I2CDevicePortData* i2c;
        i2c = device->pinData;

        startBit(i2c, 1);
        if (!tx8(i2c, device->deviceAddress<<1)) return floatWires(i2c);
        if (!tx8(i2c, reg)) return floatWires(i2c);

        i2c->sda :> void;
        waitQuarter(i2c);
        i2c->scl :> void;       // stop bit, but not as we know it - restart.
        waitQuarter(i2c);
        startBit(i2c, 0);      // Do not wait on start-bit - just do it.
        return i2c_master_do_rx(device->deviceAddress, data, nbytes, i2c);
    }
}

int xmosI2CDeviceWriteRegister(
    struct I2CDevice* unsafe device, uint8_t reg, uint8_t data[], int nbytes)
{
    unsafe
    {
        if (device == 0) return -1;

        struct I2CDevicePortData* i2c;
        i2c = device->pinData;

        startBit(i2c, 1);
           if (!tx8(i2c, device->deviceAddress<<1)) return floatWires(i2c);
        #ifndef I2C_NO_REGISTER_ADDRESS
        #ifdef I2C_TI_COMPATIBILITY
           if (!tx8(i2c, addr << 1 | (s_data[0] >> 8) & 1)) return floatWires(i2c);
        #else
           if (!tx8(i2c, reg)) return floatWires(i2c);
        #endif
        #endif
           for(int j = 0; j < nbytes; j++) {
               if (!tx8(i2c, data[j])) return floatWires(i2c);
           }
           stopBit(i2c);
           return 1;
    }
}
