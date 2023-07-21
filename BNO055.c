
// Custom includes
#include "I2CInstruction.h"
#include "I2CDriver.h"
#include "Defines.h"
#include "BNO055.h"

// Other includes
#include <util/delay.h>
#include <stdint.h>

// Heading data (integer from 0 to 5760)
static volatile uint8_t headingArr[2] = {0};

uint16_t bno055GetCurrHeading(void)
{
    uint16_t currHeading = 0;

    currHeading = (headingArr[0] | ((uint16_t)(headingArr[1]) << 8));
    return currHeading;
}

/* Sends the instructions to the IMU to enter NDOF fusion mode
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055EnterNDOF()
{
    // Necessary for the device to initialize itself
    _delay_ms(600);
    
    return bno055WriteReg(0b00011100, BNO055_OPR_MODE_ADDR);
}

/* Sends the instructions to the IMU to read the current pitch
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055GetPitch(uint8_t * out)
{
    return bno055MultiRegRead(out, BNO055_EULER_PITCH_LSB_ADDR, 2);
}

/* Sends the instructions to the IMU to read the current roll
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055GetRoll(uint8_t * out)
{
    return bno055MultiRegRead(out, BNO055_EULER_ROLL_LSB_ADDR, 2);
}


/* Sends the instructions to the IMU to read the current heading
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055GetHeading(uint8_t * out)
{
    return bno055MultiRegRead(out, BNO055_EULER_HEADING_LSB_ADDR, 2);
}

/* Sends the instructions to the IMU to read all orientations
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055GetAllEuler(uint8_t * out)
{
    return bno055MultiRegRead(out, BNO055_EULER_HEADING_LSB_ADDR, 6);
}

/* Sends the instructions to the IMU to write reg with data toWrite 
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055WriteReg(uint8_t toWrite, uint8_t reg)
{
    uint8_t writeBuf[2] = {reg, toWrite};
    I2CInstruction_ID ret = 0;
    int timeOutCounter = 0;

    while((!ret) && timeOutCounter < 100)
    {
        I2CTask();
        ret = I2CBufferAddInstruction(BNO055_I2C_ADDR, I2C_WRITE, writeBuf, 2);
        timeOutCounter++;
    }

    return ret;
}

/* Sends the instructions to the IMU to read reg into out 
 * Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055ReadReg(uint8_t * out, uint8_t reg)
{
    I2CInstruction_ID ret = 0;
    int timeOutCounter = 0;
    while(timeOutCounter < 100 &&
        (!I2CBufferAddInstruction(BNO055_I2C_ADDR, I2C_WRITE, &reg, 1)))
    {
        I2CTask();
        timeOutCounter++;
    }

    while((!ret) && timeOutCounter < 100)
    {
        I2CTask();
        ret = I2CBufferAddInstruction(BNO055_I2C_ADDR, I2C_READ, out, 1);
        timeOutCounter++;
    }
    return ret;
}

/* Sends instructions to the IMU to read numOfRegsToRead from firstReg to
 * out. Returns an I2CInstruction ID for the final instruction sent */
I2CInstruction_ID bno055MultiRegRead(uint8_t * out,
                                     uint8_t firstReg,
                                     size_t numOfRegsToRead)
{
    int timeOutCounter = 0;
    I2CInstruction_ID ret = 0;
    while(timeOutCounter < 100 && 
        (!I2CBufferAddInstruction(BNO055_I2C_ADDR, I2C_WRITE, &firstReg, 1)))
    {
        I2CTask();
        timeOutCounter++;
    }
    
    while((!ret) && timeOutCounter < 100)
    {
        I2CTask();
        ret = I2CBufferAddInstruction(BNO055_I2C_ADDR, I2C_READ, out, numOfRegsToRead);
        timeOutCounter++;
    }
    return ret;
}

// Sends an I2CInstruction to get the current heading, returns the id
I2CInstruction_ID bno055Task(void)
{
    return bno055GetHeading((uint8_t *)headingArr);
}

/* Converts an euler angle uint8_t pair into a float */
void fusionRawToFormatted(uint8_t  * raw, double * formatted)
{
    /*
        The raw data is 16 x the angle in degrees, with the MSB in raw[1]
        and LSB in raw[0].
    */
    formatted[0] = (raw[0] | (((int16_t) (*(raw+1))) << 8)) / 16.0;
}