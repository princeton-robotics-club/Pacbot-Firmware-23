
#ifndef _COMMS_H_
#define _COMMS_H_

void commsTask(void);
void commsReceiveTask(void);
void debug_comms_task(void);
void commsUpdateModeTask(void);

typedef enum gs {
    GS_ON,
    GS_OFF,
} Gamestate;

Gamestate getGameState();
uint8_t getCurrentInstructionType();
uint8_t getCurrentInstructionData();
uint32_t getCurrentInstructionNum();
void moveToNextInstruction();

extern int16_t avTicksStart;
extern int16_t goalTicksTotal;
extern volatile Direction g_s_targetCardinalDir;

#endif // _COMMS_H_