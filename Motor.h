#include <avr/io.h>
#include <avr/interrupt.h>

// Two motors
#define NUM_MOTORS 2

// TOP value
#define TOP 4095

// Motor #1
#define M1_DDR  DDRD
#define M1_PORT PORTD
#define M11 PORTD6
#define M12 PORTD7

// Motor #2
#define M2_DDR  DDRB
#define M2_PORT PORTB
#define M21 PORTB2
#define M22 PORTB3

// Motor directions
#define DIR_BW 0
#define DIR_FW 1

// Get PWM port number for motor 1
#define M1_PWM_PORT   ((leftMotorDir == DIR_FW) ? M11 : M12)
#define M2_PWM_PORT   ((rightMotorDir == DIR_FW) ? M21 : M22)
#define M1_CONST_PORT ((leftMotorDir == DIR_FW) ? M12 : M11)
#define M2_CONST_PORT ((rightMotorDir == DIR_FW) ? M22 : M21)

// Initializes the motors
void motorsInit();

// Sets the power of the motors
void setLeftMotorPower(int power_12bit_signed);
void setRightMotorPower(int power_12bit_signed);

// Getter methods
int getLeftMotorPower();
int getLeftMotorDir();
int getRightMotorPower();
int getRightMotorDir();