#!/usr/bin/env python

import pygame
import time
import os
import serial
import array
import sys
import math

# To check what serial ports are available in Linux, use the bash command: dmesg | grep tty
# To check what serial ports are available in Windows, use the cmd command: wmic path Win32_SerialPort
comPort = 'COM4'
joystickNum = 0

joystickYDrive = 1
joystickRDrive = 3
joystickArm = 4
joystickClaw = 5
joystickAnalogTriggerArm = 2
joystickAnalogTriggerClaw = 5
joystickBed = 0
joystickHorn = 1

deadband = 0.10 # Deadband for the analog joystick.

def main():

    # Initialize the serial port
    ser = serial.Serial(comPort, 57600, timeout=1)

    # Initialize the gamepad
    pygame.init()
    joysticks = []
    for i in range(0, pygame.joystick.get_count()):
        joysticks.append(pygame.joystick.Joystick(i))
        joysticks[-1].init()
        print("Detected joystick '",joysticks[-1].get_name(),"'")

    # Local variables
    prevdriveMtrCmds = {'left':0, 'right':0}
    prevArmCmd = 0
    prevClawCmd = 0
    prevBedCmd = 0
    prevTimeSent = 0
    done = False

    try:
        while (done == False):

            pygame.event.pump() # This line is needed to process the gamepad packets

            # Get the raw values for drive translation/rotation and arm using the gamepad
            yRaw = -joysticks[joystickNum].get_axis(joystickYDrive)  # Y-axis translation comes from the left joystick Y axis
            rRaw = -joysticks[joystickNum].get_axis(joystickRDrive) # Rotation comes from the right joystick X axis

            # Get the drive motor commands for Arcade Drive
            driveMtrCmds = arcadeDrive(yRaw, rRaw)

            # Get the arm motor command
            armCmd = 0
            clawCmd = 0
            bedCmd = 0
            
            # Get arm, claw and bed commands
            armUpBtn = joysticks[joystickNum].get_button(joystickArm)
            clawCloseBtn = joysticks[joystickNum].get_button(joystickClaw)
            analogTriggerRawArm = joysticks[joystickNum].get_axis(joystickAnalogTriggerArm)
            analogTriggerRawClaw = joysticks[joystickNum].get_axis(joystickAnalogTriggerClaw)
            bedDumpBtn = joysticks[joystickNum].get_button(joystickBed)
            hornBtn = joysticks[joystickNum].get_button(joystickHorn)

            # Arm
            if (analogTriggerRawArm + 1 >= deadband):
                armCmd = 1 # armMed
            elif (armUpBtn):
                armCmd = 2 # armHigh
            else:
                armCmd = 0 #armLow

            # Claw
            if (analogTriggerRawClaw + 1 >= deadband):
                clawCmd = 1 # clawMed
            elif (clawCloseBtn):
                clawCmd = 0 # clawClosed
            else:
                clawCmd = 2 # clawOpen

            # Bed
            if (bedDumpBtn):
                bedCmd = 1 # bedDump
            else:
                bedCmd = 0 # bedDown

            # Assemble the bitwise command
            bitwiseCmd = armCmd | clawCmd << 2 | bedCmd << 4 | hornBtn << 5

            # Only send if the commands changed or if 200ms have elapsed
            # if (prevdriveMtrCmds['left'] != driveMtrCmds['left'] or
            #     prevdriveMtrCmds['right'] != driveMtrCmds['right'] or
            #     prevArmCmd != armCmd or prevClawCmd != clawCmd or prevBedCmd != bedCmd or
            #     time.time()*1000 > prevTimeSent + 200):
            if (time.time()*1000 > prevTimeSent + 200):

                print("Sending... L: ", driveMtrCmds['left'], ", R: ", driveMtrCmds['right'], ", A: ", armCmd, ", C: ", clawCmd, ", B: ", bedCmd)
                ser.write(chr(255))  # Start byte
                ser.write(chr(driveMtrCmds['left']))
                ser.write(chr(254-driveMtrCmds['right']))
                ser.write(chr(bitwiseCmd))

                prevdriveMtrCmds = driveMtrCmds
                prevArmCmd = armCmd
                prevClawCmd = clawCmd
                prevBedCmd = bedCmd
                prevTimeSent = time.time()*1000
                time.sleep(.05)
    except KeyboardInterrupt:
        cleanup()


################################################################################
## @brief  Function to compute the drive motor PWM values for Arcade Drive
## @param  yIn - raw joystick input from -1.0 to 1.0 for the Y-axis translation
## @param  rIn - raw joystick input from -1.0 to 1.0 for the rotation
## @return an array containing left and right motor commands
################################################################################
def arcadeDrive(yIn, rIn):
    
    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential functions for each input (y/r)
    endExpConst = 1.44 # don't change this unless you've really looked over the math

    yExpConst = 1.5  # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    yEndpoint = 127  # maximum/minumum (+/-) for the Y-axis translation

    rExpConst = 1.5  # exponential growth coefficient of the rotation -- should be between 1.0-4.0
    rEndpoint = 127  # maximum/minimum (+/-) for the rotation

    # Set a deadband for the raw joystick input


    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    leftMtrBaseCmd = int(2)
    rightMtrBaseCmd = int(3)

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if yIn < 0:
        yNeg = -1
    else:
        yNeg = 1

    if rIn < 0:
        rNeg = -1
    else:
        rNeg = 1

    # Apply a deadband
    if abs(yIn) < deadband:
        yIn = 0
    if abs(rIn) < deadband:
        rIn = 0

    # print("X: ", xIn, " Y: ", yIn, " R: ", rIn)
    
    # Compute the drive commands using the exponential function (zero-based)
    yCmd = int(yNeg*(math.pow(math.e, math.pow(math.fabs(yIn), yExpConst)/endExpConst)-1)*yEndpoint) # zero-based
    rCmd = int(rNeg*(math.pow(math.e, math.pow(math.fabs(rIn), rExpConst)/endExpConst)-1)*rEndpoint) # zero-based

    # Convert the drive commands into motor comands (zero-based)
    leftMtrCmd = yCmd + rCmd   # zero-based
    rightMtrCmd = yCmd - rCmd  # zero-based

    # Add an offset for the minimum command to overcome the gearboxes
    if leftMtrCmd > 0:
        leftMtrCmd = leftMtrCmd + leftMtrBaseCmd
    elif leftMtrCmd < 0:
        leftMtrCmd = leftMtrCmd - leftMtrBaseCmd
    if rightMtrCmd > 0:
        rightMtrCmd = rightMtrCmd + rightMtrBaseCmd
    elif rightMtrCmd < 0:
        rightMtrCmd = rightMtrCmd - rightMtrBaseCmd

    # print("L: ", leftMtrCmd, " R: ", rightMtrCmd)

    # If the commands are greater than the maximum or less than the minimum, scale them back
    maxMtrCmd = max(leftMtrCmd, rightMtrCmd)
    minMtrCmd = min(leftMtrCmd, rightMtrCmd)
    scaleFactor = 1.0
    if maxMtrCmd > maxCommand or minMtrCmd < minCommand:
        if maxMtrCmd > abs(minMtrCmd):
            scaleFactor = float(maxCommand) / float(maxMtrCmd)
        else:
            scaleFactor = float(minCommand) / float(minMtrCmd)

    # print("maxMtrCmd: ", maxMtrCmd, " minMtrCmd: ", minMtrCmd, " maxCommand: ", maxCommand, " minCommand: ", minCommand, " scaleFactor: ", scaleFactor)

    leftdriveMtrCmdScaled = leftMtrCmd * scaleFactor
    rightdriveMtrCmdScaled = rightMtrCmd * scaleFactor

    # print("L scaled: ", leftdriveMtrCmdScaled, " R scaled: ", rightdriveMtrCmdScaled)

    # Shift the commands to be based on the zeroCommand (above)
    leftMtrCmdFinal = int(leftdriveMtrCmdScaled + zeroCommand)
    rightMtrCmdFinal = int(rightdriveMtrCmdScaled + zeroCommand)

    return {'left':leftMtrCmdFinal, 'right':rightMtrCmdFinal}


############################################################
## @brief  Function to compute the arm drive command
## @param  aIn - raw joystick input from -1.0 to 1.0
## @return the arm command
############################################################
def armDrive(aIn):

    # Set output command range constants
    zeroCommand = int(127)  # the default value that corresponds to no motor power
    cmdRange = int(127)     # the maximum amount (+/-) that the command can vary from the zero command
    maxCommand = cmdRange
    minCommand = -cmdRange

    # Set constants for the exponential function
    endExpConst = 1.44 # don't change this unless you've really looked over the math

    expConst = 1.5  # exponential growth coefficient of the Y-axis translation -- should be between 1.0-4.0
    endpoint = 127  # maximum/minumum (+/-) for the Y-axis translation

    # Set a base command (within the command range above) to overcome gearbox resistance at low drive speeds
    baseCmd = int(5)

    # Save the negative-ness, which will be re-applied after the exponential function is applied
    if aIn < 0:
        neg = -1
    else:
        neg = 1

    # Apply a deadband
    if abs(aIn) < deadband:
        aIn = 0
    
    # Compute the motor command using the exponential function (zero-based)
    aCmd = int(neg*(math.pow(math.e, math.pow(math.fabs(aIn), expConst)/endExpConst)-1)*endpoint) # zero-based

    # Add an offset for the minimum command to overcome the gearboxes
    if aCmd > 0:
        aCmd = aCmd + baseCmd
    elif aCmd < 0:
        aCmd = aCmd - baseCmd

    # If the command is greater than the maximum or less than the minimum, scale it back
    if aCmd > maxCommand:
        aCmd = maxCommand
    elif aCmd < minCommand:
        aCmd = minCommand

    # Shift the command to be based on the zeroCommand (above)
    aCmd = aCmd + zeroCommand

    return aCmd


############################################################
## @brief Zero all the commands to the robot and exit
############################################################
def cleanup():

    print("Cleaning up and exiting")
    ser = serial.Serial(comPort, 57600, timeout=1)
    ser.write(b'\xFF')
    ser.write(chr(127))
    ser.write(chr(127))
    ser.write(b'\x00')
    ser.close()
    exit() 


if __name__ == '__main__':
    sys.exit(int(main() or 0))
