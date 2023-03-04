// Custom Includes
#include "BNO055.h"
#include "UsartAsFile.h"
#include "Control.h"
#include "comms.h"
#include "Defines.h"

// Library Includes
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <util/atomic.h>

typedef struct Command
{
    uint8_t commType;
    uint8_t commData;
    uint32_t commNum;
} PacbCommand;

static volatile PacbCommand g_s_commandBuf[2] = {};

static Gamestate gameState = GS_ON;
static volatile uint32_t g_lastCommandSent = 0b00010000000100000001000000010000;
// 1 if failed, 0 if succeeded
static volatile uint8_t g_lastCommandFailed = TRUE;
Gamestate getGameState()
{
    return gameState;
}
uint8_t getCurrentInstructionType()
{
    return g_s_commandBuf[0].commType;
}
uint8_t getCurrentInstructionData()
{
    return g_s_commandBuf[0].commData;
}
uint32_t getCurrentInstructionNum()
{
    return g_s_commandBuf[0].commNum;
}
void moveToNextInstruction()
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) 
    {
        g_s_commandBuf[0].commData = g_s_commandBuf[1].commData;
        g_s_commandBuf[0].commType = g_s_commandBuf[1].commType;
        g_s_commandBuf[0].commNum = g_s_commandBuf[1].commNum;
        g_s_commandBuf[1].commNum = 0;
    }
}

/* This function uses fputc(char, usartStream_Ptr) to send the
 * following messages to the offboard computer 
 * '|' g_lastCommandSent[3:0] S/F '\n' */
// Should have six calls to function fputc
void commsSendTask(void)
{
    fputc('|', usartStream_Ptr);
    for (int8_t i = 24; i >= 0; i-=8)
    {
        uint8_t nb = g_lastCommandSent >> i;
        fputc(nb, usartStream_Ptr);
    }
    fputc((uint8_t)getReceiveBufSize(), usartStream_Ptr);
    fputc('\n', usartStream_Ptr);

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
    if (getReceiveBufSize() < 8)
    {
        return;
    }

    int b = fgetc(usartStream_Ptr);
    // fputc(b, usartStream_Ptr);
    if (b != '|')
    {
        return;
    }

    uint32_t tempLastCommand = 0;
    for (int8_t i = 24; i >= 0; i-=8)
    {
        b = fgetc(usartStream_Ptr);
        // fputc(b, usartStream_Ptr);
        if (b == '\n')
        {
            return;
        }
        tempLastCommand |= ((uint32_t)b) << i;
    }

    uint8_t tempGameState = 0;
    b = fgetc(usartStream_Ptr);
    // fputc(b, usartStream_Ptr);
    if (b == '\n')
    {
        return;
    }
    tempGameState = b;

    uint8_t tempAction = 0;
    b = fgetc(usartStream_Ptr);
    // fputc(b, usartStream_Ptr);
    if (b == '\n')
    {
        return;
    }
    tempAction = b;
    
    b = fgetc(usartStream_Ptr);
    // fputc(b, usartStream_Ptr);
    if (b != '\n')
    {
        return;
    }
    
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        // This is faster than filling a struct and copying by value
        uint8_t ind = 1;
        if (!g_s_commandBuf[0].commNum)
        {
            ind = 0;
        }
        g_s_commandBuf[ind].commNum = tempLastCommand;
        if (tempAction & 0b10000000)
        {
            g_s_commandBuf[ind].commType = A_TYPE_MOVE;
        }
        else
        {
            g_s_commandBuf[ind].commType = A_TYPE_FACE;
        }
        g_s_commandBuf[ind].commData = tempAction & 0b1111111;

        gameState = tempGameState;
    }

    // REMOVE THIS IT IS FOR DEBUGGING
#warning REMOVE_THE_NEXT_LINE_AT_SOME_POINT   

    if (tempLastCommand > g_lastCommandSent)
    {
        g_lastCommandSent = tempLastCommand;
    }

    //fprintf(usartStream_Ptr, "HERE\n");

    return;
}

void commsTask(void)
{
    commsSendTask();
    commsReceiveTask();
    return;
}

#define modeChar(mode) (mode ? 'V' : 'A')

void debug_comms_task(void)
{
    static char k_inp[15];
    static int myindex = 0;
    static int mode = 0; // 0 = angle, 1 = speed
    static int modeChar = 'A';

    char k_name = 0;
    int k_val = 0;
    int read_rdy = 0;

    // Initialize the pointers
    static int * kp = &kpA;
    static int * ki = &kiA;
    static int * kd = &kdA;

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
        k_val = strtol(k_inp+1, NULL, 10);
        modeChar = mode ? 'A' : 'V';
        
        switch (k_name) {
            case 'p': 
                *kp = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "kp%c = %d\n", modeChar(mode), *kp);
                break;
            case 'i':
                *ki = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "ki%c = %d\n", modeChar(mode), *ki);
                break;
            case 'd':
                *kd = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "kd%c = %d\n", modeChar(mode), *kd);
                break;
            case 'm':
                goalTpp = k_val;
                motors_on = 1;
                fprintf(usartStream_Ptr, "goaltpp = %d\n", goalTpp);
                break;
            case 'r':
                adjustHeading(k_val << 4);
                fprintf(usartStream_Ptr, "rotation started\n");
                motors_on = 1;
                goalTpp = 0;
                break;
            case 'w':
                wallAlignRight();
                fprintf(usartStream_Ptr, "wall-adjust started\n");
                motors_on = 1;
                goalTpp = 0;
                break;
            case '/':
                fprintf(usartStream_Ptr, "switched PID --> %c to %c", modeChar(mode), modeChar(1 - mode));
                mode = 1 - mode;
                kp = mode ? (&kpV) : (&kpA);
                ki = mode ? (&kiV) : (&kiA);
                kd = mode ? (&kdV) : (&kdA);
                break;
            case '?':
                fprintf(usartStream_Ptr, "kp%c = %d, ki%c = %d, kd%c = %d", modeChar(mode), *kp, modeChar(mode), *ki, modeChar(mode), *kd);
                motors_on = 0;
                break;
            default:
                fprintf(usartStream_Ptr, "invalid - cut motors");
                motors_on = 0;
        }
        DDRE |= (1 << PE6);
        PORTE |= (1 << PE6);
    }
}