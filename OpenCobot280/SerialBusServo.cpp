// SerialBusServo.cpp
// Low-level functions for implementing the Serial Bus Servo protocol on the Waveshare ESP32 servo driver. 
// https://www.waveshare.com/wiki/Servo_Driver_with_ESP32
// Peter Jansen, 2023

#include "SerialBusServo.h"

HardwareSerial *pSerial;
void initializeServoSerialBus() {
  Serial1.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);
  pSerial = &Serial1;
  while(!Serial1) {
    // Wait until initialized.
    // TODO: Add timeout and verbose error message?
  }
}


// Read a block of memory from a given servo. 
//   servoID:         The ID of the servo to read from.
//   startAddress:    The start address to begin reading from
//   numBytesToRead:  The number of bytes to read, starting from the start address
//   readBuffer:      the 8-bit buffer to store the information that's been read
// NOTE THAT READBUFFER SHOULD ALWAYS INCLUDE THE 6 EXTRA BYTES FOR THE HEADER.
int readServoMemory(uint8_t servoId, uint8_t startAddress, uint8_t numBytesToRead, uint8_t* readBuffer) {

  // First, send the read request
  uint8_t parameters[] = {startAddress, numBytesToRead};
  sendServoPacket(servoId, INST_READ, (uint8_t*)parameters, 2);

  // Then, read the response
  int totalBytesToRead = numBytesToRead + 6;    // 5 in the header, +1 checksum
  int totalBytesRead = readServoPacket(readBuffer, numBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;    
  } 

  return 1;
}

int writeServoReg8(uint8_t servoId, uint8_t regAddress, uint8_t data) {
  // First, send the read request
  uint8_t parameters[] = {regAddress, data};
  sendServoPacket(servoId, INST_WRITE, (uint8_t*)parameters, 2);

  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];
  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;    
  } 

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    Serial.printf("SERVO RETURNED NON-ZERO WORKING STATUS (%02X)", readBuffer[5]);
    return -1;
  }

  // Success
  return 1;
}


// Convert two bytes into a 16-bit integer
uint16_t parse16BitUInt(uint8_t highByte, uint8_t lowByte) {
  return (((uint16_t)highByte) << 8) + lowByte;
}

// Pack an unsigned 16 bit integer into an 8-bit array, low byte first.
void packU16toButter(uint16_t byteToPack, uint8_t* buffer) {
  // Low byte first
  uint8_t lowByte = (uint8_t)(0x00FF & byteToPack);
  uint8_t highByte = (uint8_t)((0xFF00 & byteToPack) >> 8);

  buffer[0] = lowByte;
  buffer[1] = highByte;  
}

#define MAX_WRITE_BYTES 32

// Low-level function to send a packet to a given servo
void sendServoPacket(uint8_t servoID, uint8_t instruction, uint8_t* parameters, uint8_t length) {
  uint8_t bufferOut[MAX_WRITE_BYTES];
  // Clear buffer
  for (int i=0; i<MAX_WRITE_BYTES; i++) {
    bufferOut[i] = 0;
  }  

  // Step 1: Pack the message into a transmit buffer
  // Header
  bufferOut[0] = 0xff;
  bufferOut[1] = 0xff;
  bufferOut[2] = servoID;
  bufferOut[3] = length + 2;    // +2 because the length includes the parameters (=length), plus the instruction byte (+1), plus the checksum (+1)
  bufferOut[4] = instruction;

  // Parameters
  for (int i=0; i<length; i++) {
    bufferOut[5+i] = parameters[i];
  }

  // Checksum
  int transmitLength = length + 6;  // Includes the full header
  // Note: Checksum calculation does not include the first two bytes (0xFF 0xFF)
  uint8_t checksum = 0;
  for (int i=2; i<transmitLength; i++) {
    checksum += bufferOut[i];
  }
  bufferOut[5+length] = ~checksum;  // Add the checksum byte

  // Step 2: Transmit the message
  pSerial->write(bufferOut, transmitLength);  

}

// Low-level function to wait for a read packet from a servo
#define READ_TIMEOUT_MILLIS     100
#define SERVO_PACKET_HEADER_CHAR  0xFF
int readServoPacket(uint8_t* readBuffer, uint8_t length) {
  int numBytesRecieved = 0;
  unsigned long startTime = millis();       // Keep track of the time, in case there's a read timeout

  int numFFs = 0;


  while (1) {
    // Try to read a byte
    int byteIn = pSerial->read();

    // Check to see if a byte was available to read
    if (byteIn != -1) {

      // Header (0xFF 0xFF)
      if (numFFs < 2) {
        // Sync to header (nominally 0xFF's)
        if (byteIn == SERVO_PACKET_HEADER_CHAR) {
          numFFs += 1;
        } else {
          numFFs = 0;
        }

        // If we've received the header, then add that to the buffer
        readBuffer[0] = SERVO_PACKET_HEADER_CHAR;
        readBuffer[1] = SERVO_PACKET_HEADER_CHAR;
        numBytesRecieved = 2;

      } else {
        // Non-header
        // Store the byte
        readBuffer[numBytesRecieved] = (uint8_t)byteIn;
        numBytesRecieved += 1;
      }
    }

    // Stop condition 1: Check to see if we've received the expected number of bytes
    if (numBytesRecieved > length) {
      break;
    }

    // Stop condition 2: Check to see if we've timed out
    unsigned long deltaTime = millis() - startTime;
    if (deltaTime > READ_TIMEOUT_MILLIS) {
      break;
    }
  }

  // Return the number of bytes that were successfully read
  return numBytesRecieved;
}

/*
 * ATOM
 */ 

// Low-level function to send a packet to a given servo
void sendAtomPacket(uint8_t instruction, uint8_t* parameters, uint8_t length) {
  uint8_t bufferOut[MAX_WRITE_BYTES];
  // Clear buffer
  for (int i=0; i<MAX_WRITE_BYTES; i++) {
    bufferOut[i] = 0;
  }  

  // Step 1: Pack the message into a transmit buffer
  // Header
  bufferOut[0] = 0xfe;
  bufferOut[1] = 0xfe;  
  bufferOut[2] = length + 2;    // +2 because the length includes the parameters (=length), plus the instruction byte (+1), plus the checksum (+1)
  bufferOut[3] = instruction;

  // Parameters
  for (int i=0; i<length; i++) {
    bufferOut[4+i] = parameters[i];
  }

  // Checksum
  int transmitLength = length + 5;  // Includes the full header
  /*
  // Note: Checksum calculation does not include the first two bytes (0xFF 0xFF)
  uint8_t checksum = 0;
  for (int i=2; i<transmitLength; i++) {
    checksum += bufferOut[i];
  }
  bufferOut[4+length] = ~checksum;  // Add the checksum byte
  */
  
  bufferOut[4+length] = 0xFA;  // Add static 0xFA to the end of packets

  // Step 2: Transmit the message
  pSerial->write(bufferOut, transmitLength);  
}




