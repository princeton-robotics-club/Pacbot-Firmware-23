#ifndef _DEFINES_H_
#define _DEFINES_H_

#ifndef F_CPU
#define F_CPU 16000000
#endif//F_CPU

#define TRUE 1
#define FALSE 0

#define A_TYPE_MOVE 1
#define A_TYPE_FACE 0

// #define DIR_NORTH 0b00
// #define DIR_SOUTH 0b01
// #define DIR_EAST  0b10
// #define DIR_WEST  0b11

typedef enum dir {
    DIR_NORTH,
    DIR_EAST,
    DIR_SOUTH,
    DIR_WEST,
} Direction;

typedef enum act {
    ACT_OFF,
    ACT_ROTATE,
    ACT_MOVE,
    ACT_MOVE_COR,
    ACT_STOP,
} Action;

void setActionMode(Action mode);
Action getActionMode();

#endif /*_DEFINES_H_*/