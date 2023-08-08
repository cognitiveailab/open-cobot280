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

# Turn off all servos
def servosOff():
    # Send the "off" command to the serial port, and wait for the response (up to the EOT marker)
    response = sendCommand("off")

# Torque all servos
def servosTorque():
    # Send the "torque" command to the serial port, and wait for the response (up to the EOT marker)
    response = sendCommand("torque")

# Stop all servos in their current position
def servosStop():
    # Send the "stop" command to the serial port, and wait for the response (up to the EOT marker)
    response = sendCommand("stop")


#
#   High-level functions
#

# Record the motion of the arm.
# This function allows the user to move the arm to waypoints, and records the position of each servo at each waypoint.
# The function exports a series of code blocks that can be used to replay the motion using the queueMove() function.
# At the start of this function, all servos will be untorqued. 
# The user will be prompted to move the arm to a waypoint, and then press enter.
# This repeats until the user enters "done".

def recordMotion():
    # Step 1: Untorque all servos
    servosOff()

    # Step 2: Prompt the user to move the arm to a waypoint, and then press enter
    wayPoints = []
    inputStr = ""
    while (True):
        print("Move the arm to waypoint " + str(len(wayPoints)) + ", and then press enter. (or, type `done` to finish).")
        inputStr = input()

        # If the user entered "done", then stop
        if (inputStr == "done"):
            break

        # Otherwise, record the current servo positions
        wayPoints.append(readServoInfo())

    # Step 3: Convert waypoints to code
    code = ""
    for i in range(len(wayPoints)):
        # Get the waypoint
        wayPoint = wayPoints[i]

        # Add a comment specifying which waypoint this is
        code += "# Waypoint " + str(i) + "\n"

        # Add a code block to move each servo to the position in the waypoint
        for servoID in wayPoint:
            code += "queueServoMove(" + str(servoID) + ", " + str(wayPoint[servoID]["currentPosition"]) + ")\n"

        # Add a code block to execute the queue
        code += "executeQueue()\n"

        # Add a code block to wait for the user to press enter
        code += "input()\n"

        code += "\n"

    # Step 4: Print the code
    print(code)


# Send the "help" command to the serial port, and wait for the response (up to the EOT marker)
print(sendCommand("help"))



# Test1: Read the position of all servos
servoInfo = readServoInfo()
print(servoInfo)


# Test2: Move back and forth 5 times
for i in range(5):
        
    queueServoMove(5, 1000)
    executeQueue()

    time.sleep(2)

    queueServoMove(5, 800)
    executeQueue()

    time.sleep(2)
