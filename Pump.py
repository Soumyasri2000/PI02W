import time
import json
import TempHum
import logging
import logfunc
from gpiozero import DigitalInputDevice, DigitalOutputDevice
from TempHum import Outside_DB, Am2108sensorinit

#configuration

WATER_LEVEL_PIN = 5
PUMP_PIN =6
WATER_SUPPLY_PIN = 13
PUMP_FLOW_SWITCH = 19

Set_Point_Temperature = 25
MAX_WATER_FILL_TIMEOUT = 30*1000

#STATES

WTR = False
PMP = False
WS = False
PUMP_FLOW_FlAG = False
FP = FC = COM = HUM_STAT = 0
deviceStatus = 1
no_flow_counter =0
NO_FLOW_LIMIT =20*1000
INITIAL_FLOW_WAIT =20*1000
flowCheckInit = False
flowStartTime = 0

#setup GPIo
water_Sensor = DigitalInputDevice(WATER_LEVEL_PIN, pull_up =  True)
pump_relay = DigitalOutputDevice(PUMP_PIN, initial_value = False)
water_relay = DigitalOutputDevice(WATER_SUPPLY_PIN, initial_value = False)
flow_switch = DigitalInputDevice(PUMP_FLOW_SWITCH, pull_up=True)
isFillingWater = False
waterFillStartTime = 0

#GPIO : True/False
def digital_read(pin):
    return pin.value
def digital_write(device, state):
    device.value = state
def upload_message(msg, *, placeholder=True):
    #send JSOn or string to external system
    if placeholder:
        print(f":[PLACEHOLDER] uploading message : {msg}")
    else:
        raise NotImplementedError("upload_message must be implemengted for real use...")

def checkPumpFlow():
    global PUMP_FLOW_FLAG, PMP, no_flow_counter, flowCheckInit, flowStartTime
    #implement flow check logic here
    if not PMP:
        return #only check when pump is running
    if not flowCheckInit:
        flowStartTime = int(time.time() * 1000)
        flowCheckInit = True
        logging.info("Pump started – waiting for flow to establish")
        return
    elapsed = int(time.time() * 1000) - flowStartTime
    if elapsed < INITIAL_FLOW_WAIT:
        return  # still within initial flow wait period
    # Check flow switch: HIGH means no flow
    if flow_switch.value:
        noFlowCounter += int(time.time() * 100)  # approximate ms increments
        if noFlowCounter % 1000 == 0:
            logging.warning(f"No flow detected for {noFlowCounter} ms")

        if noFlowCounter >= NO_FLOW_LIMIT:
            pump_relay.off()
            PMP = False
            PUMP_FLOW_FLAG = True
            noFlowCounter = 0
            flowCheckInit = False
            logging.error("ALERT: Pump stopped due to no flow condition")
            upload_message(json.dumps({"PFF": 1}))
    else:
        if noFlowCounter > 0:
            logging.info("Flow detected – normal operation")
            noFlowCounter = 0
        if PUMP_FLOW_FLAG:
            PUMP_FLOW_FLAG = False
            upload_message(json.dumps({"PFF": 0}))


def checkWaterLevel(fan_status):
    global PMP, WTR, WS, isFillingWater, waterFillStartTime
    Am2108sensorinit()
    outside_temp = Outside_DB
    #winter mode: do not fill if too cold outside
    if outside_temp is not None and outside_temp < (Set_Point_Temperature - 10):
       water_relay.off()
       WTR = False
       isFillingWater = False
       logging.info(f"Winter Mode: Skipping  water fill too cold .....")
       return
    # SAFETY : Check water level sensor
    if not water_Sensor.value: # LOW --> full tank
       water_relay.off()
       WTR = False
       isFillingWater = False
       logging.info(f"Tank is full water supply off ....")
       if deviceStatus == 1:
           if not PMP:
               pump_relay.on()
               PMP = True
               logging.info(f"pump started (tank full)....")
           checkPumpFlow()
           time.sleep(0.1)
       return
    #START FILL SEEKING
    if not isFillingWater:
        isFillingWater = True
        waterFillStartTime = int(time.time() * 1000)
        WTR - True
        water_relay.on()
        logging.info(f"Tank needs water... starting Fill.....")
        upload_message("WTRF")
        time.sleep(0.1)
    # SAFETY TIMEOUT
    elapsed = int(time.time()*1000) - waterFillStartTime
    if isFillingWater and elapsed >= MAX_WATER_FILL_TIMEOUT:
        WS = True
        water_relay.off()
        WTR = False
        isFillingWater = False
        logging.error(f" Water Fill timeout... water supply stopped....")
        error = {
            "FAN" : fan_status,
            "PMP" : not PMP,
            "WTR" : WTR,
            "PFF" : PUMP_FLOW_FLAG,
            "FP"  : FP,
            "FC"  : FC,
            "WS"  : WS,
            "COM" : COM,
            "HUM" : HUM_STAT
        }
        upload_message(json.dumps(error))








