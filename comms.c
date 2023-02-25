// Custom Includes
#include "BNO055.h"
#include "UsartAsFile.h"
#include "Control.h"
#include "comms.h"

// Library Includes
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

typedef struct Command
{
    uint8_t commType;
    uint8_t commData;
    uint32_t commNum;
} PacbCommand;

static volatile PacbCommand g_s_commandBuf[2];

uint8_t gameState = GS_ON;
volatile uint32_t g_lastCommandSent;

/* This function uses fputc(char, usartStream_Ptr) to send the
 * following messages to the offboard computer 
 * '|' g_lastCommandSent[3:0] '\n' */
// Should have six calls to function fputc
void commsSendTask(void)
{
    return;
}

/* First, this function checks the number of received bytes using
 * getReceiveBufSize(). If the return value >= 8 then this function
 * uses fgetc(usartStream_Ptr) to read in the following messasge from
 * the offboard computer
 * '|' commandNumber[3:0] gameState Action '\n''
 * Fill gameState with gameState 
 * If g_s_commandBuf[0].commNum = 0 fill g_s_commandBuf[0] with Action
 * Otherwise fill g_s_commandBuf[1] with Action 
 * in the buffer, commType is the highest order bit of Action, commData
 * if the other 7 bits. */
// Should have 8 calls to fputc
void commsReceiveTask(void)
{
    return;
}

void commsTask(void)
{
    commsReceiveTask();
    commsSendTask();
    return;
}



void debug_comms_task(void)
{
    static char k_inp[15];
    static int myindex = 0;

    char k_name = 0;
    int k_val = 0;
    int read_rdy = 0;
    //char * k_tmp;

    // Print the sensor data
    // fprintf(usartStream_Ptr, "%d\n", currHeading);
    // Print any data we've received (loopback testing)
    int readBufSize = getReceiveBufSize();
    if (readBufSize)
    {
        int data = 0;
        
        for (; (data = fgetc(usartStream_Ptr)) != -1; myindex++)
        {
            k_inp[myindex] = data;
            if (data == '\n')
            {
                read_rdy = 1;
            }   
        }            
    }

    if (read_rdy)
    {
        k_inp[myindex] = 0;
        myindex = 0;
        k_name  = k_inp[0] | 32;
        k_val = strtod(k_inp+1, NULL);
        
        switch (k_name) {
            case 'p': 
                kpW = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] kp changed to %d\n", kpW);
                break;
            case 'i':
                kiW = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] ki changed to %d\n", kiW);
                break;
            case 'd':
                kdW = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] kd changed to %d\n", kdW);
                break;
            case 'm':
                goalTpp = k_val;
                motors_on = 1;
                fprintf(usartStream_Ptr, "[c] motor set speed changed to %d\n[c] activated motors\n", goalTpp);
                break;
            case 'r':
                setGoalHeading(bno055GetCurrHeading() + (k_val << 4));
                fprintf(usartStream_Ptr, "[c] new goal angle set\n[c] activated motors\n");
                motors_on = 1;
                goalTpp = 0;
                break;
            case '?':
                fprintf(usartStream_Ptr, "[c] kp = %d, ki = %d, kd = %d, 12-bit pwm = %d", kpW, kiW, kdW, goalTpp);
                motors_on = 0;
                break;
            default:
                fprintf(usartStream_Ptr, "[c] invalid input - cut motors");
                motors_on = 0;
        }
        DDRE |= (1 << PE6);
        PORTE |= (1 << PE6);
    }
}