
#ifndef _COMMS_H_
#define _COMMS_H_

void commsTask(void);
void debug_comms_task(void);

typedef enum gs {
    GS_ON,
    GS_OFF,
} Gamestate;

Gamestate getGameState();
uint8_t getCurrentInstructionType();
uint8_t getCurrentInstructionData();
uint32_t getCurrentInstructionNum();
void moveToNextInstruction();

#endif // _COMMS_H_