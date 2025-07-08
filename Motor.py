import time
import json
import sys
import os
import serial
import logging
import logfunc

fan_status = False

#initialising the raspberry pi UART protocol.....

MotorInterface = serial.Serial(
    port='/dev/serial0',  # uses primary UART on GPIO14/15
    baudrate=57600,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=1
)

#starting the motor on UART Interface
def start_motor():
    global fan_status
    crc = bytearray([0x78, 0x01, 0x00, 0xFD, 0x81])
    logging.info(f"Telling Motor to start via {serial.port}")
    for b in crc:
        MotorInterface.write(bytes([b]))
    logging.info("Sent the Motor start command...")
    fan_status = True

#Stopping the motor on UART Interface
def stop_motor():
    global fan_status
    crc = bytearray([0x78, 0x00, 0x00, 0xFE, 0x81])
    logging.info(f"Telling Motor to stop via {serial.port}")
    for i in crc:
        MotorInterface.write(bytes([i]))
    logging.info("sent the Motor stop command....")
    fan_status = False



#speed  value comes from setup function
def setspeed(speed):
    speed = enforce_min_fan_speed(speed)
    finalspeed = int((speed*16383)/1050)
    crc = bytearray(8)
    crc[3] = 0x79
    crc[4] = finalspeed & 0xFF
    crc[5] = (finalspeed >> 8) & 0xFF
    finalspeed = converter(finalspeed)
    constantValue = 65791
    sum_value = 639 + finalspeed
    result = constantValue - sum_value
    crc[7] = result & 0xFF
    crc[6] = (result >> 8) &0xFF
    logging.info(f"setting fan speed to : {speed}  RPM")
    with serial.Serial('/dev/serial0', baudarate= 57600, timeout = 1) as  MotorInterface:
        for i in crc:
           MotorInterface.write(bytes[i])


def enforce_min_fan_speed(requested_speed):
    if requested_speed <= 500:
        return 500
    elif requested_speed > 1050:
        return 1050
    return requested_speed


def converter(speed: int) ->int:
    return ((speed >> 8)& 0xFF | ((speed & 0xFF) << 8))

def clearMotorWarnings2():
    crc = bytearray([0x01, 0x06, 0x01, 0x86, 0x00, 0x01, 0xFE, 0x72])
    logging.info(f" about to send data to motot on pin 14 GPIO...")
    with serial.Serial('/dev/serial0', baudrate=57600, timeout=1) as MotorInterface:
       for byte in crc:
            MotorInterface.write(bytes([byte]))
    # Print confirmation message
    logging.info("\nData sent to motor to reset warning 1")

def clearMotorWarnings1():
    crc = bytearray([0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFE])
    logging.info(f" About to send data to motor on pin 14...")
    with serial.Serial('/dev/serial0', baudrate=57600, timeout=1) as MotorInterface:
       for byte in crc:
            MotorInterface.write(bytes([byte]))
    logging.info("\nData sent to motor to reset warning 2")

