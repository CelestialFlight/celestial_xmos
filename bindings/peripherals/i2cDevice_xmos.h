#ifndef __I2C_DEVICE_XMOS_H__
#define __I2C_DEVICE_XMOS_H__

#include <xs1.h>

extern "C"
{
    #include "i2cDevice.h"
}

struct I2CDevicePortData
{
    port scl;
    port sda;
    uint32_t clockTicks;
};

void xmosI2CDeviceInit(struct I2CDevice* unsafe device, void* unsafe pinData);

int xmosI2CDeviceReadRegister(
    struct I2CDevice* unsafe device, uint8_t reg, uint8_t data[], int nbytes);

int xmosI2CDeviceWriteRegister(
    struct I2CDevice* unsafe device, uint8_t reg, uint8_t data[], int nbytes);

#endif
