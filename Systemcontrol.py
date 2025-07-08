import socket
import logging
import busio
import paho.mqtt.client as mqtt
import subprocess
from smbus2 import SMBus
import board
from adafruit_ahtx0 import AHTx0
from i2c_expanders import PCA9555
import time
import serial
import RPi.GPIO as GPIO
from StartupSequnece import Startup_sequence
from MQTTcomm import MqttConnection, mqttClient
from Firmware_ota import newCommandReceived, upload_message,parse_json_data ,Set_Point_Temperature ,is_emergency_message
from Motor import start_motor, stop_motor, setspeed
from Pump import checkPumpFlow, checkWaterLevel
from TempHum import Temperature, Outside_DB, Outside_RH, Supply_DB, Supply_RH , calculate_Wet_bulb,saturation_vapour_pressure
from GSMmodule import init_gsm

import json

Set_Point_Temperature = 25
#  Constants & GPIO pins 
water_level_pin = 12
water_flow_pin = 4
FILTER_PRESENT =27
pump_flow_switch= 11
emergency_button_pin= 14
#communication_in_transition = False
Water_Supply_Solenoid = 12
Pump_Pin = 10
Motor_Enable_Pin = 14
Humidity_On_Solenoid = 13
Drain_Solenoid = 15
Led_Pin = 16
FILTER_CLEAN = 17

ssid =""
password=""
mqtt_server =""
mqtt_port = 1883
mqtt_user =""
mqtt_password =""
sub_topic =""
pub_topic=""
device_id = ""
wifi_ssid =""
wifi_password=""

PMP  = True
PUMP_FLOW_FLAG = False
HUM_STAT = False
FP = False
FC = False
motorStopped = False
emergencyModeActive = False
WTR = False
WS = COM = False
humidityRatio = pulseCount =lastCheckTime = lastUploadTime = lastHumidityPusleTime = lastInitialFillMessageTime = lastCheckWaterTime = lastSafetyCheckTime = 0
humidityPulseState = initialFillDone = False
HUMIDITY_PULSE_INTERVAL = 10000 

communication_in_transition = False
newMessageStatus = False
globalMessage = ""
GSM_MODE = True
deviceStatus = 1
MODE = 1
SETTING = 0
FAN = False
FS = 1040

def SelfCareRoutine():
    global communication_in_transition, GSM_MODE, PMP, HUM_STAT
    communication_in_transition = True
    prev_GSM_mode = GSM_MODE
    logging.info(f"Starting Selfcare Routine - communication chnages restricted......")
    start_time = time.time()
    try:
       #reset everything
       for i in (Water_Supply_Solenoid, Humidity_On_Solenoid, Pump_Pin, Drain_Solenoid):
           GPIO.output(i, GPIO.LOW)
       PMP = HUM_STAT = False
       #full fan speed
       setspeed(1040)
       start_motor()
       #stage 1: drying(5minutes)
       upload_message("SHONGPRC1")
       while time.time() - start_time < 300:
           MqttConnection()
           if newMessageStatus:
               if is_emergency_message(globalMessage):
                   logging.info(f"Emergency stop during selfcare....")
                   break
               newMessageStatus = False
           time.sleep(0.1)
       stop_motor()
       # stage2: Drainage
       upload_message("SHONGPRC2")
       GPIO.output(Drain_Solenoid, GPIO.HIGH)
       GPIO.output(Pump_Pin, GPIO.HIGH)
       PMP = True
       last_status = time.time()
       while GPIO.input(pump_flow_switch) == GPIO.LOW:
           MqttConnection()
           if newMessageStatus and is_emergency_message(globalMessage):
               break
           newMessageStatus = False
           checkPumpFlow()
           if time.time()- last_status >= 60:
               upload_message("SHONGPRC2")
               last_status = time.time()
           time.sleep(0.1)
       GPIO.output(Drain_Solenoid, GPIO.LOW)
       GPIO.output(Pump_Pin , GPIO.LOW)
       PMP = False
       # stage3: Refill
       upload_message("SHONGPRC3")
       GPIO.output(Water_Supply_Solenoid, GPIO.HIGH)
       last_status = time.time()
       while GPIO.input(water_level_pin) == GPIO.HIGH:
           MqttConnection()
           if newMessageStatus and is_emergency_message(globalMessage):
               break
           newMessageStatus = False
           if time.time() - last_status >= 60:
               upload_message("SHONGPRC3")
               last_status = time.time()
       GPIO.output(Water_Supply_Solenoid, GPIO.LOW)
       # stage4: Rinsing(5min)
       upload_message("SHONGPRC4")
       GPIO.output(Drain_Solenoid, GPIO.LOW)
       GPIO.output(Pump_Pin, GPIO.HIGH)
       PMP = True
       rinse_start = time.time()
       last_status = time.time()
       while time.time() - rinse_start < 300:
           MqttConnection()
           if newMessageStatus and is_emergency_message(globalMessage):
               break
           newMessageStatus = False
           checkPumpFlow()
           if time.time() - last_status >= 60:
               upload_message("SHONGPRC4")
               last_status = time.time()
           time.sleep(0.1)
       GPIO.output(Pump_Pin, GPIO.LOW)
       PMP = False
       # stage5: second Drainage
       upload_message("SHONGPRC5")
       GPIO.output(Drain_Solenoid, GPIO.HIGH)
       GPIO.output(Pump_Pin, GPIO.HIGH)
       PMP = True
       last_status = time.time()
       while GPIO.input(pump_flow_switch) == GPIO.LOW:
           MqttConnection()
           if newMessageStatus and is_emergency_message(globalMessage):
               break
           newMessageStatus = False
           checkPumpFlow()
           if time.time() - last_status >= 60:
               upload_message("SHONGPRC5")
               last_status = time.time()
           time.sleep(0.1)
       GPIO.output(Drain_Solenoid, GPIO.LOW)
       GPIO.output(Pump_Pin, GPIO.LOW)
       PMP = False
       # stage 6: Final Refill
       upload_message("SHONGPRC6")
       GPIO.output(Water_Supply_Solenoid, GPIO.HIGH)
       last_status = time.time()
       while GPIO.input(water_level_pin) == GPIO.HIGH:
           MqttConnection()
           if newMessageStatus and is_emergency_message(globalMessage):
               break
           newMessageStatus = False
           if time.time() - last_status >= 60:
               upload_message("SHONGPRC6")
               last_status = time.time()
           time.sleep(0.1)
       GPIO.output(Water_Supply_Solenoid, GPIO.LOW)
       # completion of the self-care routine
       logging.info(f"Self Care Routine Completed, Thankyou for waiting.....")
       upload_message("SHCMP")
       # Restore startup or shutdown state
       if deviceStatus == 1:
           checkWaterLevel()
           Startup_sequence()
       else:
           for i in (Water_Supply_Solenoid, Humidity_On_Solenoid, Pump_Pin, Drain_Solenoid):
               GPIO.output(i, GPIO.LOW)
           PMP = HUM_STAT = False
           stop_motor()
           upload_message("0")
    except Exception as e:
        logging.info(f"Error in self care routine: {e}")
    # Restore communication state
    GSM_MODE = prev_GSM_mode
    communication_in_transition = False
    MqttConnection() 

 #Setup process for AMBIATOR
def setup():
    global PMP, PUMP_FLOW_FLAG, HUM_STAT, FP, FC
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(water_level_pin, GPIO.IN)
    GPIO.setup(water_flow_pin, GPIO.IN)
    GPIO.setup(FILTER_PRESENT, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(FILTER_CLEAN, GPIO.IN)
    GPIO.setup(pump_flow_switch, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.setup(emergency_button_pin, GPIO.IN)

    # Output pins and default states
    outputs = [
        Water_Supply_Solenoid,
        Humidity_On_Solenoid,
        Pump_Pin,
        Led_Pin,
        Drain_Solenoid
    ]
    for pin in outputs:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)

    # Initialize state variables
    PMP = False
    PUMP_FLOW_FLAG = False
    HUM_STAT = False
    # Read initial sensor states
    FP = GPIO.input(FILTER_PRESENT)
    FC = not GPIO.input(FILTER_CLEAN)

    ser = serial.Serial('/dev/serial0', 115200, timeout=1)
    ser.flush()

    #I2C Bus Setup
    i2c_bus = busio.I2C(board.SCL, board.SDA)

    # I2C Multiplexer
    def MultiplexerSelectChannel(channel):
        if 0 <= channel <=7:
            with SMBus(1) as bus:
                bus.write_byte(0x70, 1 << channel)
                logging.info(f"I2C Multiplexer channel {channel} selected... ")
    try: 
        MultiplexerSelectChannel(0)
        aht20 = AHTx0(i2c_bus)
        logging.info(f"AHT20 sensor initialized..")
    except Exception as e:
        logging.info(f"AHT20 sensor error:{e}")
        
    # WIFI
    logging.info("Initializing  wifi connection... ")
    logging.info(f"SSID: {ssid}")
    subprocess.run(['nmcli','ratio','wifi','off'])
    time.sleep(0.5)
    subprocess.run(['nmcli','ratio','wifi','on'])
    time.sleep(0.5)

    logging.info(f"WiFi Address:")
    subprocess.run(['cat', '/sys/class/net/wlan0/address'])

    subprocess.run(['nmcli','ratio','wifi','connect', ssid, 'password',password])
    time.sleep(2)
      
    retryWiFiConnect = 0
    connected = False
   
    while retryWiFiConnect <= 5:
        result = subprocess.run(['nmcli','-t','-f','ACTIVE,SSID','con','show', '--active'],stdout=subprocess.PIPE)
        output = result.stdout.decode()
        if ssid in output:
            connected = True
            break
        time.sleep(1)
        retryWiFiConnect += 1
        logging.info ('.', end='',flush=True)
        if retryWiFiConnect % 5 == 0:
            logging.info(f"\nConnection attempt #{retryWiFiConnect}")


    # MQTT setup          
    def SetupMqtt(connection_type="wifi"):
        try:
            
            global mqttClient
            if mqttClient is not None:
                mqttClient.disconnect()
                mqttClient.loop_stop()
                mqttClient = None
                time.sleep(1)
            mqttClient = mqtt.Client()
            mqttClient.username_pw_set(mqtt_user,mqtt_password)
            mqttClient.subscribe(sub_topic)
            com_mode = 0 if connection_type == "wifi" else 1
            upload_message(mqttClient, pub_topic,"STATUS")
            return True
        except Exception as e:
            logging.info(f"MQTT connection failed: {e}")
            return False

    #gsm fall back
    if connected:
        logging.info(f"\nWiFi connected sucessfully.....")
        WIFI_FLAG = True
        GSM_MODE = False
        time.sleep(3)
        if SetupMqtt("wifi"):
            logging.info("MQTT connected via wifi.")
        else:
            logging.info("MQTT CONNECTION FAILED VIA WIFI.")
    else:
        logging.info("WiFi failed , switching to GSM...")
        WIFI_FLAG = False
        GSM_MODE =True
        gsm_success= False
        
        for attempt in range(1,4):
            logging.info(f"GSM INIT ATTEMPT {attempt}/3...")
            if init_gsm():
 
                logging.info("GSM initialized successfully...")
                gsm_success = True
                break
            else:
                logging.info("GSM INIT FAILED..")
                time.sleep(1)
        if gsm_success:
            time.sleep(2)
            mqtt_connected=False
            for attempt in range(1,4):
                logging.info(f"MQTT (GSM) connect attempt {attempt}/3...")
                if SetupMqtt("gsm"):
                    logging.info(f"MQTT connected  via GSM..")
                    mqtt_connected = True
                    break
                else:
                    logging.info(f"MQTT connection failed via GSM")
                    time.sleep(2)
            if not mqtt_connected:
                print("MQTT failed over GSM after multiple attemts")
        else:
            logging.info(f"GSM failed after multiple attempts , check module or SIM.")

    MessageToUpload=""
    if mqttClient and mqttClient.is_connected():
        upload_message(mqttClient, pub_topic,"STATUS")


          
def LoopModule():
    global motorStopped ,isFillingWater 
    motorStopped =False
    isFillingWater = False
    GPIO.setmode(GPIO.BCM)
    GPIO.setup([Humidity_On_Solenoid, Pump_Pin, Drain_Solenoid, Water_Supply_Solenoid], GPIO.OUT)
    GPIO.setup([water_level_pin, emergency_button_pin,FILTER_PRESENT, FILTER_CLEAN], GPIO.IN)
    global newMessageStatus,globalMessage, FAN, PMP
    global emergencyModeActive, Outside_DB, Outside_RH, Supply_DB, Supply_RH
    global humidityRatio, humidityPulseState, pulseCount, lastCheckTime
    global lastUploadTime, lastHumidityPusleTime, initialFillDone
    global lastInitialFillMessageTime, lastCheckWaterTime, lastSafetyCheckTime
    global deviceStatus, WTR
    MqttConnection()
    newCommandReceived()
    if GSM_MODE and mqttClient and mqttClient.is_connected():
        mqttClient.loop()
        if newMessageStatus:
            logging.info(f"Processing message in GSM Mode: {globalMessage}")
            parse_json_data(globalMessage)
            newMessageStatus = False
    if deviceStatus == 1 and MODE != 0 and not FAN:
        logging.info(f"WARNING: Device is ON but fan is  not running - attempting to restart motor")
        start_motor()
        time.sleep(1)
    #multiplexer.selectChannel(0)
    #if not aht20.begin():
        GPIO.output(Humidity_On_Solenoid, GPIO.LOW)
        HUM_STAT = False
    if deviceStatus == 0:
        if not motorStopped:
            GPIO.output(Pump_Pin, GPIO.LOW)
            PMP = False
            stop_motor()
            motorStopped = True
            upload_message("0")
        return
    else:
        motorStopped = False
    if MODE!= 0:
        if GPIO.input(emergency_button_pin) == GPIO.LOW and not emergencyModeActive:
            logging.info(f"EMERGENCY MODE ACTIVATED.....")
            upload_message("EMERGENCY")
            emergencyModeActive = True
            for i in [Water_Supply_Solenoid, Humidity_On_Solenoid, Pump_Pin, Drain_Solenoid]:
                GPIO.output(i, GPIO.LOW)
            PMP= HUM_STAT = False
            stop_motor()
            deviceStatus = 0
            upload_message("0")
        if not emergencyModeActive:
            if deviceStatus == 1:
                targetSpeed = FS
                if MODE == 1 and SETTING !=1:
                    if Outside_DB >= (Set_Point_Temperature - 6):
                        targetSpeed = 1040
                    else:
                        targetSpeed = 1040 if Supply_DB > Set_Point_Temperature else 500
                if not hasattr(LoopModule, "lastSetSpeed"):
                    LoopModule.lastSetSpeed = 0
                if LoopModule.lastSetSpeed  != targetSpeed:
                    setspeed(targetSpeed)
                    LoopModule.lastSetSpeed = targetSpeed
                    logging.info(f" Setting fan speed directly to {targetSpeed}")
                    upload_message(json.dumps({"FS": targetSpeed}))
            else:
                for i in [Water_Supply_Solenoid, Humidity_On_Solenoid, Pump_Pin, Drain_Solenoid]:
                    GPIO.output(i, GPIO.LOW)
                HUM_STAT = PMP = False
                stop_motor()
                deviceStatus = 0
                upload_message("0")
            if PMP:
                checkPumpFlow()
                time.sleep(0.1)
            if time.time() - lastCheckTime >= 60:
                Temperature()
                Outside_WB = calculate_Wet_bulb(Outside_DB, Outside_RH)
                es = saturation_vapour_pressure(Supply_DB)
                e = Supply_RH * es
                humidityRatio = 0.62197*(e/(101325 - e)) * 1000 + 0.22
                if MODE == 1:
                    tempdiff = abs(Outside_DB - Outside_WB)
                    if tempdiff < 3.0 or humidityRatio > 20.5:
                        if SETTING != 1 and (time.time() - lastHumidityPulseTime >= HUMIDITY_PULSE_INTERVAL/1000):
                            humidityPulseState = not humidityPulseState
                            GPIO.output(Humidity_On_Solenoid, GPIO.HIGH if humidityPulseState else GPIO.LOW)
                            HUM_STAT = humidityPulseState
                            lastHumidityPulseTime = time.time()
                    else:
                        GPIO.output(Humidity_On_Solenoid, GPIO.LOW)
                        HUM_STAT = False
                        humidityPulseState = False
                        lastHumidityPulseTime = 0
                    if Outside_DB < (Set_Point_Temperature - 6):
                        GPIO.output(Pump_Pin, GPIO.LOW)
                        PMP = False
                lastCheckTime = time.time()
                upload_message(json.dumps({
                    "OT": Outside_DB,
                    "OH": Outside_RH,
                    "SL": Supply_DB,
                    "SP": Set_Point_Temperature
                }))
            if time.time() - lastUploadTime >= 60:
                FP = (GPIO.input(FILTER_PRESENT) == GPIO.LOW)
                FC = not GPIO.input(FILTER_CLEAN)
                TWU = pulseCount
                upload_message(json.dumps({"TWU": TWU}))
                pulseCount = 0
                errorMEssage = {
                    "FAN": not FAN,
                    "PMP": not PMP,
                    "WTR": WTR,
                    "PFF": PUMP_FLOW_FLAG,
                    "FP": FP,
                    "FC": FC,
                    "WS": WS,
                    "COM": COM,
                    "HUM": HUM_STAT
                }
                upload_message(json.dumps(errorMEssage))
                lastUploadTime = time.time()
            if not initialFillDone:
                if time.time() - lastInitialFillMessageTime >= 120:
                    lastInitialFillMessageTime = time.time()
                checkWaterLevel()
                if not isFillingWater and GPIO.input(water_level_pin) == GPIO.LOW:
                    initialFillDone = True
                    logging.info(f"Initial Fill Completed... switching to periodic level checks...")
            else:
                if time.time() - lastCheckWaterTime >=300:
                    checkWaterLevel()
                    lastCheckWaterTime = time.time()
            if time.time() - lastSafetyCheckTime >= 30:
                if GPIO.input(water_level_pin) == GPIO.LOW:
                    GPIO.output(Water_Supply_Solenoid, GPIO.LOW)
                    WTR = False
                    isFillingWater = False
                    logging.info(f"Safety check : Tank is full water supply off...")
                lastSafetyCheckTime = time.time()
