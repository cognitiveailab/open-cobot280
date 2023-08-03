// Class representing a single ST3215 serial bus servo. 
// Peter Jansen, 2023

#include "ST3215.h"

/*
 *  Constructor/Destructor
 */ 

// Constructor
ServoST3215::ServoST3215(uint8_t id) {
  this->servoId = id;
  this->setErrorState(0);       // Initialize to no error state
  this->setNotExists();         // By default, doesn't exist until it's found by ping()

  // Load blank values in variables
  this->load = 0;
  this->currentSpeed = 0;
  this->voltage = 0;
  this->current = 0;
  this->currentPosition = 0;
  this->mode = 0;
  this->temp = 0;
  this->isMoving = 0;
}

// Destructor
ServoST3215::~ServoST3215() {

}


/*
 * Servo existence getters/setters
 */

// Set that the servo appears to exist
void ServoST3215::setExists() {
  this->doesExist = 1;
}

// Set that the servo does not appear to exist
void ServoST3215::setNotExists() {
  this->doesExist = 0;
}

// Returns true if this servo exists, false otherwise;
bool ServoST3215::exists() {
  if (this->doesExist == 1) return true;
  return false;
}

/*
 * Error state
 */
void ServoST3215::setErrorState(uint8_t state) {
  this->errorState = state;
}

// Non-zero error state means an error
uint8_t ServoST3215::hasError() {
  return this->errorState;
}


/*
 *  Accessors
 */ 

void printHex(uint8_t num) {
  char hexCar[2];

  sprintf(hexCar, "%02X", num);
  Serial.print(hexCar);
}

int ServoST3215::ping() {
   // First, send the write (ping packets have no parameters)
  const uint8_t numBytesToWrite = 0;
  uint8_t parameters[numBytesToWrite];  
  sendServoPacket(servoId, INST_PING, (uint8_t*)parameters, numBytesToWrite);
  
  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];
    
  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);
  if (totalBytesRead < totalBytesToRead) {
    // Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);    
    this->setNotExists();
    return -1;    
  } 

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    Serial.printf("ERROR: SERVO RETURNED NON-ZERO WORKING STATUS (%02X)", readBuffer[5]);
    this->setErrorState(readBuffer[5]);    
    this->setExists();
    return -1;
  }

  // Success
  this->setExists();
  return 1;
}


// Poll method: Update the internal state variables by reading their values from the servo
#define POLL_ZERO_OFFSET SMS_STS_PRESENT_POSITION_L
#define POLL_BUFFER_SIZE SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + 1
#define PACKET_HEADER_SIZE  5
#define PACKET_FOOTER_SIZE  1
void ServoST3215::poll() {

  // First, read the servo memory
  uint8_t servoMemoryPacketBuffer[POLL_BUFFER_SIZE + PACKET_HEADER_SIZE + PACKET_FOOTER_SIZE];    // Add 6 for the header/checksum
  int success = readServoMemory(this->servoId, SMS_STS_PRESENT_POSITION_L, POLL_BUFFER_SIZE, (uint8_t *)servoMemoryPacketBuffer);
  if (!success) {
    Serial.println("READ ERROR");
    this->setErrorState(1);
  }

  /*
  // Display read buffer
  Serial.print("READ BUFFER: ");
  for (int i=0; i<sizeof(servoMemoryPacketBuffer); i++) {
    printHex(servoMemoryPacketBuffer[i]);
    Serial.print(" ");
  }
  Serial.println("");
  */
  
  // Next, parse the values

  // Value: Position
  uint8_t highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_POSITION_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  //Serial.print("highByte: ");
  //Serial.println(highByte, 16);
  uint8_t lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_POSITION_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  //Serial.print("lowByte: ");
  //Serial.println(lowByte, 16);
  uint16_t position = parse16BitUInt(highByte, lowByte);  
  this->currentPosition = position;

  //Serial.print("## Position: ");
  //Serial.println(this->currentPosition);


  // Value: Speed
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_SPEED_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_SPEED_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t speed = parse16BitUInt(highByte, lowByte);
  this->currentSpeed = speed;

  // Value: Load  
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_LOAD_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_LOAD_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t load = parse16BitUInt(highByte, lowByte);
  this->load = load;

  // Value: Current  
  highByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_CURRENT_H - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  lowByte = servoMemoryPacketBuffer[SMS_STS_PRESENT_CURRENT_L - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  uint16_t current = parse16BitUInt(highByte, lowByte);
  this->current = current;

  // Value: Voltage  
  uint8_t voltage = servoMemoryPacketBuffer[SMS_STS_PRESENT_VOLTAGE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->voltage = voltage;

  // Value: Temperature  
  uint8_t temperature = servoMemoryPacketBuffer[SMS_STS_PRESENT_TEMPERATURE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->temp = temperature;

  // Value: isMoving
  uint8_t isMoving = servoMemoryPacketBuffer[SMS_STS_MOVING - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->isMoving = isMoving;

  // Value: Mode    
  uint8_t mode = servoMemoryPacketBuffer[SMS_STS_MODE - POLL_ZERO_OFFSET + PACKET_HEADER_SIZE];
  this->mode = mode;

}


/*
 *  Setters
 */

// Move the servo to a given position
int ServoST3215::movePosition(uint16_t newPosition, uint16_t speed, uint8_t acceleration) {
  // First, send the write  
  const uint8_t numBytesToWrite = 8;
  uint8_t parameters[numBytesToWrite] = {SMS_STS_ACC, 0, 0, 0, 0, 0, 0, 0};

  // Acceleration
  parameters[1] = acceleration;  
  // Position    
  packU16toButter(newPosition, &parameters[2]);
  // Something in between position and speed that should stay at 0?   
  // Speed
  packU16toButter(speed, &parameters[6]);  
  // Send the write
  
  sendServoPacket(servoId, INST_WRITE, (uint8_t*)parameters, numBytesToWrite);
  
  // Then, read the response
  const int totalBytesToRead = 6;    // 0xFF 0xFF, then ID, then length, then status, then checksum
  char readBuffer[totalBytesToRead];
    
  int totalBytesRead = readServoPacket((uint8_t *)readBuffer, totalBytesToRead);

  if (totalBytesRead < totalBytesToRead) {
    Serial.printf("TOO FEW BYTES READ.  expected: %i  actual: %i \n", totalBytesToRead, totalBytesRead);
    return -1;    
  } 

  // TODO: Check for checksum

  // TODO: Also check for status of 0 (byte 5)
  if (readBuffer[4] != 0) {
    Serial.printf("SERVO RETURNED NON-ZERO WORKING STATUS (%02X)", readBuffer[5]);
    this->setErrorState(readBuffer[5]);    
    return -1;
  }

  // Success
  return 1;
}

// Release the servo (i.e. turn off its motor)
void ServoST3215::releaseServo() {
  if (!writeServoReg8(this->servoId, SMS_STS_TORQUE_ENABLE, 0x00)) {
    Serial.println("ERROR RELEASING SERVO");
    this->setErrorState(1);    
  }
}

// Torque the servo motor (i.e. turn on the servo motor, holding its current position)
void ServoST3215::torqueServo() {
  if (!writeServoReg8(this->servoId, SMS_STS_TORQUE_ENABLE, 0x01)) {
    Serial.println("ERROR TORQUING SERVO");
    this->setErrorState(1);    
  }
}

// Stop the servo motor. 
// This is accomplished by releasing it, then quickly torquing it to hold in the current position
void ServoST3215::stopServo() {
  this->releaseServo();
  delay(2);
  this->torqueServo();
}

/*
 * String methods
 * NOTE: The order of the outputs values (e.g. servo, exists, error, position, speed, etc.) should be the same across each method.
 */ 

void ServoST3215::displayStatusHuman() {
  Serial.print("servo:"); Serial.print(this->servoId);
  Serial.print(" exist:"); Serial.print(this->doesExist);
  Serial.print(" error:"); Serial.print(this->errorState);
  Serial.print(" position:"); Serial.print(this->currentPosition);
  Serial.print(" speed:"); Serial.print(this->currentSpeed);
  Serial.print(" load:"); Serial.print(this->load);
  Serial.print(" voltage:"); Serial.print(this->voltage);
  Serial.print(" current:"); Serial.print(this->current);
  Serial.print(" temp:"); Serial.print(this->temp);
  Serial.print(" mode:"); Serial.print(this->mode);  
  Serial.print(" isMoving:"); Serial.print(this->isMoving);  
}

void ServoST3215::displayStatusCSV() {
  Serial.print(""); Serial.print(this->servoId);
  Serial.print(","); Serial.print(this->doesExist);
  Serial.print(","); Serial.print(this->errorState);
  Serial.print(","); Serial.print(this->currentPosition);
  Serial.print(","); Serial.print(this->currentSpeed);
  Serial.print(","); Serial.print(this->load);
  Serial.print(","); Serial.print(this->voltage);
  Serial.print(","); Serial.print(this->current);
  Serial.print(","); Serial.print(this->temp);
  Serial.print(","); Serial.print(this->mode);  
  Serial.print(","); Serial.print(this->isMoving); 
}

void ServoST3215::displayCSVHeader() {
  Serial.print(""); Serial.print("servoId");
  Serial.print(","); Serial.print("doesExist");
  Serial.print(","); Serial.print("errorState");
  Serial.print(","); Serial.print("currentPosition");
  Serial.print(","); Serial.print("currentSpeed");
  Serial.print(","); Serial.print("load");
  Serial.print(","); Serial.print("voltage");
  Serial.print(","); Serial.print("current");
  Serial.print(","); Serial.print("temp");
  Serial.print(","); Serial.print("mode");  
  Serial.print(","); Serial.print("isMoving");
}
