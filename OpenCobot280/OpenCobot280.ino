/*
 *  An early attempt at open firmware for the myCobot280 from Elephant Robotics
 *  Intended for the low-cost Waveshare Servo controller ("Servo Driver with ESP32" https://www.waveshare.com/wiki/Servo_Driver_with_ESP32 )
 *  ("ESP32 Dev Module", with PSRAM enabled)
 *  by Peter Jansen, 2023
 *   
 *  Robotics and electronics can be dangerous on the best of days.  Be careful, and be safe.
 *  No warranty is provided, express or implied.  Use entirely at your own risk.
 *
 *  Released under the BSD 3 Clause license.
 *  
 *  Known bugs/TODOs:
 *    - ST3215 "current" doesn't appear to be reading/updating
 *    - ST3215 "speed" should likely be signed instead of unsigned (movements in the negative direction are rolling over to large values)
 */

#include "SerialBusServo.h"
#include "ST3215.h"
#include "CobotAtom.h"

#define FIRMWARE_NAME     "OPEN_COBOT280"
#define FIRMWARE_VERSION  "0.1"

#define MAX_SERVOS  10

#define READY_RESPONSE    "OK"

// Queued move requests (one per motor, only the most recent request is stored)
typedef struct {
  uint8_t servoId;
  uint8_t isValid;          // 0 if invalid/blank, 1 if valid
  uint8_t torque;           // 0 if motor off, 1 if torque
  uint8_t justTorque;       // 1 if this is just a torque request, 0 otherwise.
  uint16_t position;
  uint16_t speed;
  uint16_t acceleration;
} moveServoRequest;

// Array containing all the servos
ServoST3215 *servos[MAX_SERVOS];

// ATOM (the controller on the end effector)
CobotAtom *cobotAtom;


void setup() {
  // Initialize the serial console
  Serial.begin(115200);  

  // Show the firmware version to the serial console
  printFirmwareVersion();

  // Initialize serial servo bus
  initializeServoSerialBus();

  // Initialize servo objects
  for (int i=0; i<MAX_SERVOS; i++) { 
    servos[i] = new ServoST3215(i);
  }

  // Ping all servos, label which servos exist
  pingAllServos();

  // Torque all servos to hold their current position
  torqueAllServos();

  // Clear servo move queue
  initializeServoQueue();

  // Initialize the Cobot Atom
  cobotAtom = new CobotAtom();
  // Set the Cobot Atom screen to display white/grey
  cobotAtom->setColor(128, 128, 128);  

  // Show the READY_RESPONSE to the serial console, indicating that the firmware is initialized and ready to parse serial commands.
  Serial.println(READY_RESPONSE);

}


// Continually poll the serial port for commands.  Parse them as complete commands are received.
void loop() {    
  readAvailableSerialData();  
}


/*
 *  High-level move commands, including demos
 */ 

void moveToCenterPosition(uint16_t speed, uint8_t accel) {
  uint16_t centerPos = 4096/2;
  // Set all to center position
  for (int i=1; i<=6; i++) {
    servos[i]->movePosition(centerPos, speed, accel);
  }  
}

// Move each of the servos, one at a time
void moveDemo1(uint16_t speed, uint8_t accel) {
  uint16_t centerPos = 4096/2;

  // Center  
  moveToCenterPosition(speed, accel);
  delay(3000);

  // Move axes back/forth
  for (int i=1; i<=6; i++) {
    servos[i]->movePosition(1000, speed, accel);
    delay(3000);
    servos[i]->movePosition(3000, speed, accel);
    delay(3000);
    servos[i]->movePosition(centerPos, speed, accel);
    delay(3000);
  }
  
}


void moveDemo2(uint16_t speed, uint8_t accel) {
  // Center
  const uint16_t centerPos = 4096/2;
  const uint16_t degN90 = 4096/4;
  const uint16_t deg90 = 3*(4096/4);

  // Center  
  moveToCenterPosition(speed, accel);
  delay(3000);

  for (int cycles=0; cycles<2; cycles++) {
    delay(3000);

    servos[1]->movePosition(deg90, speed, accel);
    servos[2]->movePosition(degN90, speed, accel);
    servos[3]->movePosition(degN90, speed, accel);
    servos[4]->movePosition(deg90, speed, accel);
    servos[5]->movePosition(deg90, speed, accel);
    servos[6]->movePosition(deg90, speed, accel);

    delay(3000);

    servos[1]->movePosition(degN90, speed, accel);
    servos[2]->movePosition(deg90, speed, accel);
    servos[3]->movePosition(deg90, speed, accel);
    servos[4]->movePosition(degN90, speed, accel);
    servos[5]->movePosition(degN90, speed, accel);
    servos[6]->movePosition(degN90, speed, accel);
    
  }

}

/*
 *  Servo motion queue
 */

// Queued move requests
// TODO: There's also a declaration at the top that's required for the moveServoRequest structure.
moveServoRequest queuedServoRequests[MAX_SERVOS];

// Make a blank and invalid servo request
moveServoRequest mkBlankServoRequest(int servoId) {
  moveServoRequest request = {.servoId = servoId, .isValid = 0, .torque = 0, .justTorque = 0, .position = 0, .speed = 0, .acceleration = 0};
  return request;
}


// Initialize or reset the servo move queue.
void initializeServoQueue() {
  // Initialize servo queue
  for (int i=0; i<MAX_SERVOS; i++) {
    queuedServoRequests[i] = mkBlankServoRequest(i);
  }
}

// Run all actions currently in the queue
void runServoQueue() {
  for (int i=0; i<MAX_SERVOS; i++) {    
    // Check whether this servo has a valid request that's queued    
    if (queuedServoRequests[i].isValid == 1) {
      // If torque is off, then the request is to release ths motor
      if (queuedServoRequests[i].torque == 0) {
        servos[i]->releaseServo();
      } else {
        if (queuedServoRequests[i].justTorque == 1) {
          // This request is only to torque the motor
          if (queuedServoRequests[i].torque == 1) {
            servos[i]->torqueServo();
          } else {
            servos[i]->releaseServo();
          }
        } else {
          // Otherwise, the request is for a move
          // movePosition(uint16_t newPosition, uint16_t speed, uint8_t acceleration)        
          servos[i]->movePosition(queuedServoRequests[i].position, queuedServoRequests[i].speed, queuedServoRequests[i].acceleration);
        }
      }
    }
  }
}



// Print one moveServoRequest structure to the console
void printMoveServoRequest(moveServoRequest request) {
  // Step 1: Check if valid/active request
  if (request.isValid == 0) {
    Serial.printf("no request");
    return;
  }

  // Print structure
  Serial.printf("servo:%i isValid:%i torque:%i justTorque:%i position:%i speed:%i acceleration:%i", request.servoId, request.isValid, request.torque, request.justTorque, request.position, request.speed, request.acceleration);
}

// Print all the moveServoRequests for all servos
void printServoRequestQueue() {

  // Initialize servo queue
  for (int i=0; i<MAX_SERVOS; i++) {
    Serial.printf("Servo %i  ", i);
    printMoveServoRequest(queuedServoRequests[i]);
    Serial.println("");    
  }

}


/*
 *  Torque/release servos
 */

// Release all servos
void releaseAllServos() {
  for (int i=0; i<MAX_SERVOS; i++) {
    servos[i]->releaseServo();
  }
}

// Torque all servos
void torqueAllServos() {
  for (int i=0; i<MAX_SERVOS; i++) {
    servos[i]->torqueServo();
  }
}

// Torque all servos
void stopAllServos() {
  for (int i=0; i<MAX_SERVOS; i++) {
    servos[i]->stopServo();
  }
}


/*
 *  Read servo information
 */

// Attempt to ping all servos
void pingAllServos() {
  for (int i=0; i<MAX_SERVOS; i++) {
    Serial.printf("Servo %i: ", i);
    servos[i]->ping();
    if (servos[i]->exists()) {
      Serial.println("Found");
    } else {
      Serial.println("No Response");
    }    
  }
}

// Poll all servos, updating their current status information
void pollAllServos() {
  for (int i=0; i<MAX_SERVOS; i++) {
    if (servos[i]->exists()) {
      servos[i]->poll();
    }
  }
}

/*
 * Serial Command Parser
 */

#define SERIAL_BUFFER_SIZE  64
char serialBuffer[SERIAL_BUFFER_SIZE];
int curSerialBufferIdx = 0;
#define UNDEFINED_INT -32767


void parseSerialCommand() {
  // Initialize argument variables
  char command[SERIAL_BUFFER_SIZE];
  memset(command, 0, sizeof(command));
  int arg1 = UNDEFINED_INT;
  int arg2 = UNDEFINED_INT;
  int arg3 = UNDEFINED_INT;
  int arg4 = UNDEFINED_INT;
  int arg5 = UNDEFINED_INT;

  // Scan for arguments
  int numArgs = sscanf(serialBuffer, "%s %i %i %i %i %i", &command, &arg1, &arg2, &arg3, &arg4, &arg5);

/*
  Serial.print("numArgs: "); Serial.println(numArgs);
  Serial.print("command: "); Serial.println(command);
  Serial.print("   arg1: "); Serial.println(arg1);
  Serial.print("   arg2: "); Serial.println(arg2);
  Serial.print("   arg3: "); Serial.println(arg3);
  Serial.print("   arg4: "); Serial.println(arg4);
  Serial.print("   arg5: "); Serial.println(arg5);
  Serial.println("");
*/

  // Parse commands
  if ((strcmp(command, "move") == 0) && (numArgs == 5)) {
    // move servoId position speed
    moveServoRequest request = {.servoId = arg1, .isValid = 1, .torque = 1, .justTorque = 0, .position = arg2, .speed = arg3, .acceleration = arg4};
    queuedServoRequests[arg1] = request;

    // Also print the request to the console
    printMoveServoRequest(request);
    Serial.println("");
    Serial.println("Added move request to queue.");
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "qoff") == 0) && (numArgs == 2)) {
    // queue a command to untorque a servo
    moveServoRequest request = {.servoId = arg1, .isValid = 1, .torque = 0, .justTorque = 1, .position = 0, .speed = 0, .acceleration = 0};
    queuedServoRequests[arg1] = request;

    // Also print the request to the console
    printMoveServoRequest(request);
    Serial.println("");
    Serial.println("Added untorque request to queue.");
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "qtorque") == 0) && (numArgs == 2)) {
    // queue a command to untorque a servo
    moveServoRequest request = {.servoId = arg1, .isValid = 1, .torque = 1, .justTorque = 1, .position = 0, .speed = 0, .acceleration = 0};
    queuedServoRequests[arg1] = request;

    // Also print the request to the console
    printMoveServoRequest(request);
    Serial.println("");
    Serial.println("Added torque request to queue.");
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "go") == 0) && (numArgs == 1)) {    
    // First, run the queue    
    Serial.println("Running move queue.");
    runServoQueue();    
    // Then, reset the queue
    Serial.println("Resetting queue.");
    initializeServoQueue();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "rgb") == 0) && (numArgs == 4)) {
    Serial.printf("RGB: %i, %i, %i\n", arg1, arg2, arg3);
    // queue a command to untorque a servo
    cobotAtom->setColor(arg1, arg2, arg3);
    Serial.println(READY_RESPONSE);
  
  } else if ((strcmp(command, "rainbow") == 0) && (numArgs == 1)) {
    Serial.println("Rainbow");
    cobotAtom->displayRainbow(10, 2); 
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "center") == 0) && (numArgs == 1)) {
    Serial.println("Moving to center position");
    moveToCenterPosition(1000, 100);
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "demo1") == 0) && (numArgs == 1)) {
    Serial.println("Demo 1");
    moveDemo1(1000, 100);
    Serial.println(READY_RESPONSE);

} else if ((strcmp(command, "demo2") == 0) && (numArgs == 1)) {
    Serial.println("Demo 2");
    moveDemo2(1000, 100);
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "off") == 0) && (numArgs == 1)) {
    // Turn off all motors
    Serial.println("Release all servos");
    releaseAllServos();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "torque") == 0) && (numArgs == 1)) {
    // Turn on (torque) all motors
    Serial.println("Torque all");
    torqueAllServos();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "stop") == 0) && (numArgs == 1)) {
    // Stop all servos in their current position
    Serial.println("Stop all");
    stopAllServos();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "clearqueue") == 0) && (numArgs == 1)) {
    // Reset the move queue
    initializeServoQueue();
    Serial.println("Resetting queue.");
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "queue") == 0) && (numArgs == 1)) {
    // Print currently queued servo moves
    printServoRequestQueue();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "ping") == 0) && (numArgs == 1)) {
    // Ping all servos
    pingAllServos();    
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "status") == 0) && (numArgs == 1)) {
    // Poll all servos
    pollAllServos();
    // Print servo status    
    printServoStatusHuman();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "status-csv") == 0) && (numArgs == 1)) {
    // Poll all servos
    pollAllServos();
    // Print servo status    
    printServoStatusCSV();
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "version") == 0) && (numArgs == 1)) {
    // Print firmware version
    printFirmwareVersion();  
    Serial.println(READY_RESPONSE);

  } else if ((strcmp(command, "help") == 0) || (strcmp(command, "?") == 0)) {
    // Print usage
    printUsage();
    Serial.println(READY_RESPONSE);

  } else {
    Serial.println("Unrecognized command. Type 'help' for usage.");
    Serial.println(READY_RESPONSE);

  }

}




/*
 *  String/Information functions
 */ 

void printServoStatusHuman() {
  Serial.println("Servo Status");
  for (int i=0; i<MAX_SERVOS; i++) {
    if (servos[i]->exists()) {
      servos[i]->displayStatusHuman();
      Serial.println("");
    }
  }
}

void printServoStatusCSV() {
  servos[0]->displayCSVHeader();
  Serial.println("");
  for (int i=0; i<MAX_SERVOS; i++) {
    if (servos[i]->exists()) {
      servos[i]->displayStatusCSV();
      Serial.println("");
    }
  }
}

void printFirmwareVersion() {
  Serial.print("Firmware name: ");
  Serial.println(FIRMWARE_NAME);
  Serial.print("Firmware version: ");
  Serial.println(FIRMWARE_VERSION);
}

void printUsage() {
  Serial.println("Commands                                   Description                         Example");
  Serial.println("----------------------------------------------------------------------------------------------------");
  Serial.println("Queue commands");
  Serial.println(" move servoId position speed accel         Queue a move command                move 1 1024 1000 100");
  Serial.println(" qoff servoId                              Queue an untorque command           qoff 1");
  Serial.println(" qtorque servoId                           Queue a torque command              qtorque 1");
  Serial.println(" go                                        Execute queued move commands        go");
  Serial.println(" clearqueue                                Reset the move queue                clearqueue");
  Serial.println("");
  Serial.println("Instant commands");
  Serial.println(" off                                       Turn all servos off                 off");
  Serial.println(" torque                                    Torque (turn on) servos             torque");
  Serial.println(" stop                                      Stop all servos in position         stop");
  Serial.println(" ping                                      Detect/ping all servos              ping");
  Serial.println(" rgb r g b                                 Set color on ATOM display           rgb 0 255 0");
  Serial.println("");
  Serial.println("Demos");
  Serial.println(" rainbow                                   Rainbow on ATOM display             rainbow");
  Serial.println(" center                                    Move all servos to 2048             center");
  Serial.println(" demo1                                     Move all servos back/forth          demo1");
  Serial.println(" demo2                                     Move all servos back/forth          demo2");
  Serial.println("");
  Serial.println("Informational commands");
  Serial.println(" queue                                     Display all queued moves            queue");
  Serial.println(" status                                    Display all servo information       status");
  Serial.println(" status-csv                                As above, but in CSV format         status-csv");
  Serial.println(" version                                   Display firmware version            version");
  Serial.println(" help                                      Display this message                help");
  Serial.println("----------------------------------------------------------------------------------------------------");

}


// Clear (reset) the serial recieve buffer
void clearSerialRecieveBuffer() {
  // Clear the buffer
  memset(serialBuffer, 0, sizeof(serialBuffer)); 
  // Reset the buffer index 
  curSerialBufferIdx = 0;
}


// Read data from the serial stream, if any data is available. 
// Handles calling the serial parser when appropriate. 
// Should be called periodically. 
void readAvailableSerialData() {

  while (Serial.available() > 0) {        
    char inputChar = Serial.read();
    serialBuffer[curSerialBufferIdx] = inputChar;
    curSerialBufferIdx += 1;

    // Condition 1: Check if the last character is a newline -- if so, parse the string. 
    if (inputChar == '\n') {
      // TODO: Parse string
      parseSerialCommand();

      // Clear recieve buffer
      clearSerialRecieveBuffer();
    }

    // Condition 2: Check whether the serial buffer has exceeded its size -- if so, clear it and continue
    if (curSerialBufferIdx >= SERIAL_BUFFER_SIZE) {
      // The buffer is full, but we haven't received a newline.  The transmissionn is likely errorful.
      // Clear the recieve buffer. 
      clearSerialRecieveBuffer();
    }
    
  }

 }


