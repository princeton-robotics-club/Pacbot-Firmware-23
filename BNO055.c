/*
 * BNO055.c
 *
 * Created: 8/28/2022 7:24:16 PM
 *  Author: jack2
 */ 

#ifndef F_CPU
#define F_CPU 16000000
#endif

#include <util/delay.h>
#include <stdint.h>

#include "I2CInstruction.h"
#include "I2CDriver.h"
// #include "Usart.h"
#include "BNO055.h"

I2CInstruction_ID bno055EnterNDOF(I2CBuffer_pT buf)
{
    // Necessary for the device to initialize itself!
    _delay_ms(600);
    
    return bno055WriteReg(buf, 0b00011100, BNO055_OPR_MODE_ADDR);
}

I2CInstruction_ID bno055GetPitch(I2CBuffer_pT buf, uint8_t * out)
{
    return bno055MultiRegRead(buf, out, BNO055_EULER_PITCH_LSB_ADDR, 2);
}

I2CInstruction_ID bno055GetRoll(I2CBuffer_pT buf, uint8_t * out)
{
    return bno055MultiRegRead(buf, out, BNO055_EULER_ROLL_LSB_ADDR, 2);
}

I2CInstruction_ID bno055GetHeading(I2CBuffer_pT buf, uint8_t * out)
{
    return bno055MultiRegRead(buf, out, BNO055_EULER_HEADING_LSB_ADDR, 2);
}

I2CInstruction_ID bno055GetAllEuler(I2CBuffer_pT buf, uint8_t * out)
{
    return bno055MultiRegRead(buf, out, BNO055_EULER_HEADING_LSB_ADDR, 6);
}

I2CInstruction_ID bno055WriteReg(I2CBuffer_pT buf, uint8_t toWrite, uint8_t reg)
{
    uint8_t writeBuf[2] = {reg, toWrite};
        
    I2CInstruction_ID ret = 0;
    int timeOutCounter = 0;
    while((!ret) && timeOutCounter < 100)
    {
        ret = I2CBufferAddInstruction(buf, BNO055_I2C_ADDR, I2C_WRITE, writeBuf, 2);
        timeOutCounter++;
    }
    return ret;
}

I2CInstruction_ID bno055ReadReg(I2CBuffer_pT buf, uint8_t * out, uint8_t reg)
{
    while(!I2CBufferAddInstruction(buf, BNO055_I2C_ADDR, I2C_WRITE, &reg, 1));
    
    I2CInstruction_ID ret = 0;
    int timeOutCounter = 0;
    while((!ret) && timeOutCounter < 100)
    {
        ret = I2CBufferAddInstruction(buf, BNO055_I2C_ADDR, I2C_READ, out, 1);
        timeOutCounter++;
    }
    return ret;
}

I2CInstruction_ID bno055MultiRegRead(I2CBuffer_pT buf, uint8_t * out, uint8_t firstReg, size_t numOfRegsToRead)
{
    while(!I2CBufferAddInstruction(buf, BNO055_I2C_ADDR, I2C_WRITE, &firstReg, 1));
    
    I2CInstruction_ID ret = 0;
    int timeOutCounter = 0;
    while((!ret) && timeOutCounter < 100)
    {
        ret = I2CBufferAddInstruction(buf, BNO055_I2C_ADDR, I2C_READ, out, numOfRegsToRead);
        timeOutCounter++;
    }
    return ret;
}

void fusionRawToFormatted(uint8_t  * raw, double * formatted)
{
    formatted[0] = (raw[0] | (((int16_t) (*(raw+1))) << 8)) / 16.0;
    formatted[1] = (raw[2] | (((int16_t) (*(raw+3))) << 8)) / 16.0;
    formatted[2] = (raw[4] | (((int16_t) (*(raw+5))) << 8)) / 16.0;
}