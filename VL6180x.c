#include "VL6180x.h"
#include <stdio.h>
#include "ShiftReg.h"
#include <avr/pgmspace.h>

#include "UsartAsFile.h"

// These are registers that I want to change at initialization
const uint8_t VL6180XCustomInitData[NUM_PUBLIC_REGS][3] =
{
    {0x00, 0x11, 0x10},     // Turns on GPIO1 as interrupting when data is available
    {0x01, 0x0a, 0x30},     // Sets how long it averages range measurements
    {0x00, 0x3f, 0x46},     // Sets the light and dark gain
    {0x00, 0x31, 0x64},     // Sets how often temperature measurements are done (100)
    {0x00, 0x40, 0x63},     // Set ALS integration time to 100ms
    {0x00, 0x2e, 0x02},     // Performs a single temp calibration of the ranging sensor
    {0x00, 0x1b, 0x08},     // Minimizes time between ranging measurements in continuous mode
    {0x00, 0x3e, 0x31},     // Sets the time between ALS measurements to .5 seconds
    {0x00, 0x14, 0x04},     // Configures interrupt on new sample ready threshold event for only ranging sensor
    {0x00, 0x2d, 0x00},     // Disables Range checks
    // {0x00, 0x16, 0x00},  // Sets "Fresh_out_of_reset" to 0, so that we can check later if reset occurred
    // {0x00, 0x10, 0x00},  // Disables XSHUT
    
    {0x00, 0x18, 0x03}      // Enables and initiates continuous operation
};

// These are all private registers that you are required to initialize to these values
const uint8_t VL6180XRequiredInitData[NUM_PRIVATE_REGS][3] = {
    {0x02, 0x07, 0x01},
    {0x02, 0x08, 0x01},
    {0x00, 0x96, 0x00},
    {0x00, 0x97, 0xfd},
    {0x00, 0xe3, 0x00},
    {0x00, 0xe4, 0x04},
    {0x00, 0xe5, 0x02},
    {0x00, 0xe6, 0x01},
    {0x00, 0xe7, 0x03},
    {0x00, 0xf5, 0x02},
    {0x00, 0xd9, 0x05},
    {0x00, 0xdb, 0xce},
    {0x00, 0xdc, 0x03},
    {0x00, 0xdd, 0xf8},
    {0x00, 0x9f, 0x00},
    {0x00, 0xa3, 0x3c},
    {0x00, 0xb7, 0x00},
    {0x00, 0xbb, 0x3c},
    {0x00, 0xb2, 0x09},
    {0x00, 0xca, 0x09},
    {0x01, 0x98, 0x01},
    {0x01, 0xb0, 0x17},
    {0x01, 0xad, 0x00},
    {0x00, 0xff, 0x05},
    {0x01, 0x00, 0x05},
    {0x01, 0x99, 0x05},
    {0x01, 0xa6, 0x1b},
    {0x01, 0xac, 0x3e},
    {0x01, 0xa7, 0x1f},
    {0x00, 0x30, 0x00}
};

/* Confirms the VL6180x id reg has the correct value */
static uint8_t VL6180xModelIDRegAdd[2] = {0x00, 0x00};
static int VL6180xConfirmId(int devAddress)
{
    fprintf(usartStream_Ptr, "conf: ");
    while(!I2CBufferAddInstruction(devAddress, I2C_WRITE, VL6180xModelIDRegAdd, 2))
    {
        I2CTask();
    }

    uint8_t recvdModelID;
    I2CInstruction_ID modelIDInstr = 0;
    while(!modelIDInstr)
    {
        modelIDInstr = I2CBufferAddInstruction(devAddress, I2C_READ, &recvdModelID, 1);
        I2CTask();
    }

    while (I2CBufferContains(modelIDInstr))
    {
        I2CTask();
    }
    
    if (recvdModelID == 0xb4)
    {
        fprintf(usartStream_Ptr, "Yep\n");
        return 1;
    }
    fprintf(usartStream_Ptr, "Nop\n");
    return 0;
}

/* This function initializes a VL6180x sensor with all of the required
 * and user data */
void VL6180xInitSensor(int devAddress)
{
#ifdef DEBUG
    VL6180xConfirmId(0x29);
#endif /*DEBUG*/

    // Set the new device address
    uint8_t addressReg[3] = {0x02, 0x12, 0x29};
    addressReg[2] = devAddress;
    I2CInstruction_ID addId;
    while (!(addId = I2CBufferAddInstruction(0x29, I2C_WRITE, addressReg, 3)))
    {
        I2CTask();
    }
    while (I2CBufferContains(addId))
    {
        I2CTask();
    }
    
#ifdef DEBUG
    VL6180xConfirmId(devAddress);
#endif /*DEBUG*/

    // Send all of the custom data
    for (int i = 0; i < NUM_PUBLIC_REGS; i++)
    {
        while (!I2CBufferAddInstruction(devAddress, I2C_WRITE, VL6180XCustomInitData[i], 3))
        {
            I2CTask();
        }
    }

    // Send all of the required data
    for (int i = 0; i < NUM_PRIVATE_REGS; i++)
    {
        
        while (!I2CBufferAddInstruction(devAddress, I2C_WRITE, &(VL6180XRequiredInitData[i][0]), 3))
        {
            I2CTask();
        }
    }
}

/* This function initializes all of the VL6180x sensors with all of the required
 * and user data */
void VL6180xInit(int startAddress)
{
    // Disable all of the sensors
    srClrData();
    for (int i = 0; i < 10; i++)
    {
        srShift();
    }

    // Initialize them one at a time
    srSetData();
    for (int i = 0; i < 8; i++)
    {
        srShift();
        VL6180xInitSensor(startAddress + i);

    }
    
}


static uint8_t VL6180XRangeResultLocation[2] = {0x00, 0x62};
static uint8_t VL6180XIntClear[3] = {0x00, 0x15, 0x07};
/* Adds the instructions to the I2C bus to perform a read from a
 * VL6180x distance sensor */
I2CInstruction_ID VL6180xAddRead(int devAddress, uint8_t * result)
{
    int timeOutCounter = 0;
    while(timeOutCounter < 100 && !I2CBufferAddInstruction(devAddress, I2C_WRITE, VL6180XRangeResultLocation, 2))
    {
        I2CTask();
        timeOutCounter++;
    }
    I2CInstruction_ID ret;
    while(timeOutCounter < 100 && !(ret=I2CBufferAddInstruction(devAddress, I2C_READ, result, 1)))
    {
        I2CTask();
        timeOutCounter++;
    }
    while(timeOutCounter < 100 && !I2CBufferAddInstruction(devAddress, I2C_WRITE, VL6180XIntClear, 3))
    {
        I2CTask();
        timeOutCounter++;
    }
    return ret;
}