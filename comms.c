// Custom Includes
#include "BNO055.h"
#include "UsartAsFile.h"
#include "Control.h"

// Library Includes
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>

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
                kpV = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] kp changed to %d\n", kpV);
                break;
            case 'i':
                kiV = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] ki changed to %d\n", kiV);
                break;
            case 'd':
                kdV = k_val;
                motors_on = 0;
                fprintf(usartStream_Ptr, "[c] kd changed to %d\n", kdV);
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
                fprintf(usartStream_Ptr, "[c] kp = %d, ki = %d, kd = %d, 12-bit pwm = %d", kpV, kiV, kdV, goalTpp);
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