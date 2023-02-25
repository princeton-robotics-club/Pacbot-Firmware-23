
#ifndef _COMMS_H_
#define _COMMS_H_

void comms_task(void);
void debug_comms_task(void);

typedef enum gs {
    GS_ON,
    GS_OFF,
} Gamestate;

#endif // _COMMS_H_