// SerialBusServo.h

#if !defined(SERIAL_BUS_SERVO_h) 
#define SERIAL_BUS_SERVO_h

#include <stdint.h>
#include "Arduino.h"

// Waveshare board: The UART used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// Instructions
#define INST_PING   0x01
#define INST_READ   0x02
#define INST_WRITE  0x03


// Function prototypes
void initializeServoSerialBus();

int readServoMemory(uint8_t servoId, uint8_t startAddress, uint8_t length, uint8_t* readBuffer);
int writeServoReg8(uint8_t servoId, uint8_t regAddress, uint8_t data);

uint16_t parse16BitUInt(uint8_t highByte, uint8_t lowByte);
void packU16toButter(uint16_t byteToPack, uint8_t* buffer);

void sendServoPacket(uint8_t servoID, uint8_t instruction, uint8_t* parameters, uint8_t length);
int readServoPacket(uint8_t* readBuffer, uint8_t length);

void sendAtomPacket(uint8_t instruction, uint8_t* parameters, uint8_t length);

#endif
