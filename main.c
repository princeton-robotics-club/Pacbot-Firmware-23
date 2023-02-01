/*
 * NonBlockingI2CLib.c
 *
 * Created: 7/19/2022 1:45:52 AM
 * Author : jack2
 */ 

#include "Defines.h"
#include "Control.h"
#include "I2CInstruction.h"
#include "I2CDriver.h"
#include "UsartAsFile.h"
#include "BNO055.h"
#include "ShiftReg.h"
#include "VL6180x.h"
#include "Encoder.h"
#include "Motor.h"

#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <inttypes.h>

// Here is where we currently store sensor data
volatile uint8_t g_s_fusionResult[6] = {0};
volatile double g_s_fusionFormatted[3] = {0};
volatile uint64_t g_s_encoderResult[NUM_ENCODERS] = {0};
volatile uint8_t g_s_distResult[8] = {0};

int motors_on = 0;
double goalAngle = 0.0;
double currAngle = 0.0;

// Milliseconds since initialization
volatile static unsigned long g_s_millis = 0;

// This handles our millisecond counter overflow
ISR(TIMER0_OVF_vect)
{
    // Increment milliseconds
    g_s_millis++;

    // Ask for IMU data on every 10 milliseconds
    if (!(g_s_millis % 10))
    {
        bno055GetAllEuler(&g_s_fusionResult[0]);
        currAngle = g_s_fusionFormatted[0];
        if (!motors_on)
            goalAngle = currAngle;
    }

    // Run PID every 10 milliseconds (offset by 2)
    if (!((g_s_millis+2) % 10))
    {
        pidStraightLine(motors_on);
    }

    // Ask for Encoder data every 5 milliseconds (offset by 3)
    if (!((g_s_millis-3) % 5))
    {
        getEncoderDistances(g_s_encoderResult);
    }

    // Ask for Distance data on every 10 milliseconds (offset by 1)
    if (!((g_s_millis+1) % 10))
    {
        VL6180xAddRead(0x50, &g_s_distResult[0]);
        VL6180xAddRead(0x51, &g_s_distResult[1]);
        VL6180xAddRead(0x52, &g_s_distResult[2]);
        VL6180xAddRead(0x53, &g_s_distResult[3]);
        VL6180xAddRead(0x54, &g_s_distResult[4]);
        VL6180xAddRead(0x55, &g_s_distResult[5]);
        VL6180xAddRead(0x56, &g_s_distResult[6]);
        VL6180xAddRead(0x57, &g_s_distResult[7]);
    }
}

/* This initializes timer0 to overflow about once every 1.02ms and
 * interrupt on overflow */
void millisInit(void)
{
    // Enables timer0 w/ prescaler div 64
    TCCR0B |= (1<<CS01) | (1<<CS00);

    // Enable interrupt on overflow
    TIMSK0 |= (1<<TOIE0);
}

int main(void)
{
    // This disables the JTAG debugging interface
    MCUCR |= (1<<JTD);

    // This disables clkdiv8 (use clkdiv1)
    CLKPR = (1<<CLKPCE);
    CLKPR = 0;

    // Various initializations
    I2CInit(200000);
    usartInit(115200);
    encoderInit();
   
    bno055EnterNDOF();

    sei();

    srInit();
    VL6180xInit(0x50);

    millisInit();
    motorsInit();

    // Main loop
    while (1) 
    {
        /* This call converts the first two uint8_t's in g_s_fusionResult
         * to floats for printing */
        fusionRawToFormatted(g_s_fusionResult, g_s_fusionFormatted);

        // Print the sensor data
        fprintf(usartStream_Ptr, "%lf, %lf, %d, %d; %d\n",
            currAngle, goalAngle, OCR1B, OCR1C, motors_on);

        // Print any data we've received (loopback testing)
        int readBufSize = getReceiveBufSize();
        if (readBufSize)
        {
            char * read = malloc(readBufSize+1);
            fgets(read, readBufSize+1, usartStream_Ptr);                
            fprintf(usartStream_Ptr, read);
            free(read);

            DDRE |= (1 << PE6);
            PORTE |= (1 << PE6);

            motors_on ^= 1;
        }

        if (g_s_distResult[0] < 200) motors_on = 0;

        // This runs some clks to give the prints time to send
        for (long i = 0; i < 2000L; i++)
        {
            I2CTask();
        }

    }
}