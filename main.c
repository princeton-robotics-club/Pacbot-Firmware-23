// Custom Includes
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
#include "comms.h"

// Library Includes
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <stdio.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include <string.h>

// Tick buffer - low-pass filtering for encoders using a ring buffer
#define TICK_BUFF_SIZE 4
volatile int32_t tickBuf[TICK_BUFF_SIZE];
volatile int tickBufIdx = 0;

// Tpp = ticks per period, a measure of how fast the encoders are going
volatile int16_t currTpp = 0;
volatile int16_t goalTpp = 0;

/* Add anything you want to print every 50ms */
void debug_print(void)
{
    //if (!motors_on)
    //    fprintf(usartStream_Ptr, "%d\n", VL6180xGetDist(FRONT_LEFT));
    return;
}

// Milliseconds since initialization
volatile static uint32_t g_s_millis = 0;
volatile static int8_t g_s_milliFlag = 0;

// This is run every ~1 ms
void millisTask(void)
{
    // Throws an error if millisecond timer fails
    static unsigned long lastMilli;
    if (lastMilli != g_s_millis && lastMilli != g_s_millis - 1)
    {
        fprintf(usartStream_Ptr, "last %lu", lastMilli);
        fprintf(usartStream_Ptr, "curr %lu\n", g_s_millis);
    }
    lastMilli = g_s_millis;
    
    if (!g_s_milliFlag)
    {
        return;
    }
    g_s_milliFlag = 0;

    // Ask for IMU data on every 10 milliseconds
    if (!(g_s_millis % 10))
    {
        bno055Task();
    }

    // Run PID every 10 milliseconds (offset by 2)
    if (!((g_s_millis+2) % 10))
    {
        pidStraightLine();
    }

    // Ask for Encoder data every 5 milliseconds (offset by 3)
    if (!((g_s_millis+3) % 5))
    {
        getAverageEncoderTicks((int32_t *) (tickBuf + tickBufIdx));
        currTpp = tickBuf[tickBufIdx] - tickBuf[(tickBufIdx + 1) % TICK_BUFF_SIZE];
        tickBufIdx = (tickBufIdx + 1) % TICK_BUFF_SIZE;
    }

    // Ask for Distance data on every 10 milliseconds (offset by 1)
    if (!((g_s_millis+1) % 20))
    {
        VL6180xTask();
    }

    // DEBUG PRINT EVERY 50 ms
    if (!(g_s_millis % 20))
    {
        debug_print();
    }
    
}

// This handles our millisecond counter overflow
ISR(TIMER0_OVF_vect)
{
    // Allow other interrupts to preempt this interrupt
    sei();

    // Increment milliseconds, then run the millisecond task
    g_s_millis++;
    g_s_milliFlag = 1;
    millisTask();
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
   
    // Configures IMU
    bno055EnterNDOF();

    // Enables interrupts
    sei();

    // Initializes shift register
    srInit();

    // Initializes distance sensors
    VL6180xInit();

    // Initializes millisecond timer
    millisInit();

    // Initializes motors
    motorsInit();
    
    // Main loop
    while (1) 
    {
        debug_comms_task();
        I2CTask();
    }
}