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
#define TICK_BUFF_SIZE_BITS 2
#define TICK_BUFF_SIZE (1 << TICK_BUFF_SIZE_BITS)
volatile int16_t tickBuf[TICK_BUFF_SIZE] = {0};
volatile int tickBufIdx = 0;

// Tpp = ticks per period, a measure of how fast the encoders are going
volatile int16_t currTpp = 0;
volatile int16_t goalTpp = 0;

/* Stores the current action state. See defines.h. DO NOT UPDATE ON
 * IT'S OWN UNLESS YOU KNOW WHAT YOU'RE DOING... USE THE FN */
volatile Action g_action_mode = ACT_OFF;

/* Add anything you want to print every 50ms */
#ifdef DEBUG
void debug_print(void)
{
    // fprintf(usartStream_Ptr, "Enc: %ul\n", getRightEncoderDist());

    return;
}
#endif // DEBUG

// Milliseconds since initialization
volatile static uint32_t g_s_millis = 0;
volatile static int8_t g_s_milliFlag = 0;

// Set's the action mode (intelligently)
void setActionMode(Action mode)
{
    if (mode == ACT_PUSH_FW || mode == ACT_PUSH_BW)
    {
        // fprintf(usartStream_Ptr, "HERE2\n");
        if (!testPush())
        {
            if (mode == ACT_PUSH_BW)
            {
                // fprintf(usartStream_Ptr, "HERE: mode: %d\n", mode);
                mode = ACT_MOVE_COR_BW;
            }
            else
            {
                mode = ACT_MOVE_COR;
            }   
        }
        else
        {
            adjustHeading(1000);
        }
    }
    if (mode == ACT_MOVE_COR || mode == ACT_MOVE_COR_BW)
    {
        wallAlignTest();
    }

    g_action_mode = mode;
}

// Gets the action mode
Action getActionMode()
{
    return g_action_mode;
}


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

    // Run the comms task every 5 ms
    if (!(g_s_millis % 5))
    {
        commsTask();
        // commsReceiveTask();
        // commsUpdateModeTask();
    }

    // Run PID every 10 milliseconds (offset by 4)
    if (!((g_s_millis+4) % 10))
    {
        switch (g_action_mode)
        {
        case ACT_ROTATE:
        case ACT_MOVE_COR:
        case ACT_MOVE_COR_BW:
            pidRotate();
            break;
        case ACT_MOVE:
            pidStraightLine();
            break;
        case ACT_MOVE_BW:
            pidStraightLine();
            break;
        case ACT_STOP:
            pidStop();
            break;
        case ACT_PUSH_FW:
            pidRotate();
            break;
        case ACT_PUSH_BW:
            pidRotate();
            break;
        default:
            pidOff();
            break;
        }
    }

    // Ask for Encoder data every 5 milliseconds (offset by 3)
    if (!((g_s_millis+3) % 5))
    {
        // Uses a ring buffer to low-pass filter the encoder speeds
        tickBuf[tickBufIdx] = getAverageEncoderTicks();
        currTpp = tickBuf[tickBufIdx] - tickBuf[(tickBufIdx + 1) % TICK_BUFF_SIZE];
        if (!((goalTpp >= 0) ^ (tickBuf[tickBufIdx] >= goalTicksTotal)))
            goalTpp = 0;
        tickBufIdx = (tickBufIdx + 1) % TICK_BUFF_SIZE;
    }

    // Ask for Distance data on every 10 milliseconds (offset by 1)
    if (!((g_s_millis+1) % 20))
    {
        VL6180xTask();
    }

#ifdef DEBUG
    // DEBUG PRINT EVERY 50 ms
    if (!(g_s_millis % 20))
    {
        debug_print();
    }
#endif // DEBUG
    
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

    // Start the goal heading at the starting angle
    I2CInstruction_ID firstAngBack = bno055Task();
    while (I2CBufferContains(firstAngBack))
    {
        I2CTask();
    }
    setGoalHeading(bno055GetCurrHeading());

    // Main loop
    while (1) 
    {
        #ifdef DEBUG
        debug_comms_task();
        #endif // DEBUG
        I2CTask();
    }
}