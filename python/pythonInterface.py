# pythonInterface.py

import serial           # pip install pyserial
import time

# Set the "End of transmission" makeer to be "OK"
EOT = b'OK\r\n'

# Default servo motor speed/acceleration
DEFAULT_SPEED = 1000
DEFAULT_ACCEL = 100

# Open the USB serial port (/dev/cu.usbserial-0001) at 115200 baud
serial = serial.Serial('/dev/cu.usbserial-0001', 115200)


# Send a serial command, and return the response (up to the EOT marker).
# Times out after 2 seconds.
def sendCommand(commandStr:str):
    # First, send the serial command
    serial.write(commandStr.encode())
    serial.write(b'\r\n')

    # Then, wait for the response (which could be multiple lines).
    # The final line will be only the EOT marker.
    # Return everything except the EOT marker.
    return serial.read_until(EOT).decode()
    
    #return serial.read_until(EOT, 1).decode()


# Read the motor positions using the "status-csv" command
def readServoInfo():
    # Send the "status-csv" command to the serial port, and wait for the response (up to the EOT marker)
    response = sendCommand("status-csv")

    # Split the response into lines
    lines = response.splitlines()

    # The first line is the header, which should look like the following:
    # servoId,doesExist,errorState,currentPosition,currentSpeed,load,voltage,current,temp,mode,isMoving

    # Parse the lines into a dictionary (key=servoID) of dictionaries.
    # The servoID will be the outter dictionary key.  For the inner dictionary, the keys will be the column names from the header, the values will be their values (parsed as ints)
    servoInfo = {}
    minFields = 11          # The minimum number of expected fields in a CSV line

    for line in lines[1:]:
        # Split the line into an array of strings
        values = line.split(",")
        # If there are not enough fields, skip this line
        if len(values) < minFields:
            continue

        # Convert the strings to ints
        for i in range(len(values)):
            values[i] = int(values[i])

        # Create a dictionary from the column names and values
        dictOut = dict(zip(lines[0].split(","), values))

        # Get this servos ID
        servoID = dictOut["servoId"]
        # Add this servo to the dictionary of servos
        servoInfo[servoID] = dictOut

    return servoInfo


# Queue up a move for a single servo (using the "move" command)
def queueServoMove(servoID:int, newPosition:int, speed:int = DEFAULT_SPEED, accel:int = DEFAULT_ACCEL):
    # Command format: move <servoID> <newPosition> <speed> <accel>
    commandStr = "move " + str(servoID) + " " + str(newPosition) + " " + str(speed) + " " + str(accel)

    # Send the command
    response = sendCommand(commandStr)

    # Look for the "Added move request to queue." message to be somewhere in the response
    success = False
    if ("Added move request to queue." in response):
        success = True

    return success

# Execute the servo queue using the "go" command:
def executeQueue():
    # Send the "go" command to the serial port, and wait for the response (up to the EOT marker)
    response = sendCommand("go")


# Send the "help" command to the serial port, and wait for the response (up to the EOT marker)
print(sendCommand("help"))

print(sendCommand("status-csv"))


servoInfo = readServoInfo()
print(servoInfo)


# Repeat 5 times
for i in range(5):
        
    queueServoMove(5, 1000)
    executeQueue()

    time.sleep(2)

    queueServoMove(5, 800)
    executeQueue()

    time.sleep(2)
