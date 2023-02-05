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
#include <string.h>

// Here is where we currently store sensor data
volatile uint8_t g_s_fusionResult[6] = {0};
volatile double g_s_fusionFormatted[3] = {0};
volatile uint64_t g_s_encoderResult[NUM_ENCODERS] = {0};
volatile uint8_t g_s_distResult[8] = {0};

int motors_on = 0;
double goalAngle = 0.0;
double currAngle = 0.0;
double err_strength;

// Milliseconds since initialization
volatile static unsigned long g_s_millis = 0;

// This handles our millisecond counter overflow
ISR(TIMER0_OVF_vect)
{
    sei();

    // Increment milliseconds
    g_s_millis++;

    // Ask for IMU data on every 10 milliseconds
    if (!(g_s_millis % 10))
    {
        bno055GetAllEuler(&g_s_fusionResult[0]);
    }

    // Run PID every 10 milliseconds (offset by 2)
    if (!((g_s_millis+2) % 10))
    {
        fusionRawToFormatted(g_s_fusionResult, g_s_fusionFormatted);
        currAngle = g_s_fusionFormatted[0];
        err_strength = pidStraightLine(motors_on);
        //if (err_strength != 0.0) fprintf(usartStream_Ptr, "%lf\n", err_strength);
        if (!motors_on)
            goalAngle = currAngle;
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
    
    char k_inp[15];
    int inp_size;
    char k_name = 0;
    double k_val = 0;
    char * k_tmp;
    int read_rdy = 0;
    int index;

    // Main loop
    while (1) 
    {
        /* This call converts the first two uint8_t's in g_s_fusionResult
         * to floats for printing */
        fusionRawToFormatted(g_s_fusionResult, g_s_fusionFormatted);

        // Print the sensor data
        //fprintf(usartStream_Ptr, "%lf\n",
        //    g_s_fusionFormatted[0]);

        // Print any data we've received (loopback testing)
        int readBufSize = getReceiveBufSize();
        if (readBufSize)
        {
            int data = 0;
            
            for (; (data = fgetc(usartStream_Ptr)) != -1; index++)
            {
                k_inp[index] = data;
                if (data == '\n')
                {
                    read_rdy = 1;
                }   
            }            
        }

        if (read_rdy)
        {
            k_inp[index] = 0;
            index = 0;
            k_name = k_inp[0] | 32;
            k_val = strtod(k_inp+1, &k_tmp);
            
            switch (k_name) {
                case 'p': 
                    kp = k_val;
                    motors_on = 0;
                    fprintf(usartStream_Ptr, "[c] kp changed to %lf\n", kp);
                    break;
                case 'i':
                    ki = k_val;
                    motors_on = 0;
                    fprintf(usartStream_Ptr, "[c] ki changed to %lf\n", ki);
                    break;
                case 'd':
                    kd = k_val;
                    motors_on = 0;
                    fprintf(usartStream_Ptr, "[c] kd changed to %lf\n", kd);
                    break;
                case 'm':
                    av_pwm = (int)(40.95 * k_val);
                    motors_on = 1;
                    fprintf(usartStream_Ptr, "[c] motor resting speed changed to %d\n[c] activated motors\n", (int)av_pwm);
                    break;
                case 'r':
                    goalAngle = ((int)(currAngle + k_val) % 360 + 360) % 360;
                    fprintf(usartStream_Ptr, "[c] new goal angle set\n[c] activated motors\n");
                    motors_on = 1;
                    av_pwm = 0;
                    break;
                case '+':
                    ramp_sp = (int)k_val;
                    fprintf(usartStream_Ptr, "[c] ramp_sp = %d", ramp_sp);
                    break;
                case '?':
                    fprintf(usartStream_Ptr, "[c] kp = %lf, ki = %lf, kd = %lf, 12-bit pwm = %d, ramp = %d", kp, ki, kd, av_pwm, ramp_sp);
                    motors_on = 0;
                    break;
                default:
                    fprintf(usartStream_Ptr, "[c] invalid input - cut motors");
                    motors_on = 0;
            }

            DDRE |= (1 << PE6);
            PORTE |= (1 << PE6);

            read_rdy = 0;
        }

        //if (g_s_distResult[0] < 200) motors_on = 0;

        // This runs some clks to give the prints time to send
        for (long i = 0; i < 2000L; i++)
        {
            I2CTask();
        }

    }
}