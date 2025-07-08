import time
import paho.mqtt.client as mqtt
import RPi.GPIO as GPIO
import os
import sys
import threading
import serial
import GSMmodule
import Firmware_ota
import WifiComm
import logging
import logfunc

#configuration 
mqtt_server =""
mqtt_port = 1883
mqtt_user =""
mqtt_password =""
sub_topic =""
device_id = ""
wifi_ssid =""
wifi_password=""

# Globals (initialize appropriately in your main program)
communication_in_transition = False
GSM_MODE = False
WIFI_FLAG = False
wifi_reconnect_attempts = 0
MAX_WIFI_RECONNECT_ATTEMPTS = 5
WIFI_RECONNECT_INTERVAL = 10
MQTT_RECONNECT_INTERVAL = 30


gsm_Serial = serial.Serial("/dev/ttyUSB2", baudrate = 115200, timeout = 2)
mqtt_client = mqtt.Client()
last_mqtt_attempt = 0
last_wifi_reconnect_time = 0


#call back functions
def on_connect(client, userdata, flags, rc):
    if rc ==0:
      logging.info(f"Connected to the MQTT server of the Ambiator successfuly.....")
      client.subscribe(sub_topic)
      Firmware_ota.upload_message("{\COM\":0}")
    else:
      logging.info(f"Connection failed with code {rc}")

def on_message(client,userdata, msg):
    logging.info(f"Message received : {msg.payload.decode()}")

#WIFI- Connect

def WifiConnect():
    global WIFI_FLAG, GSM_MODE, mqtt_client
    logging.info(f"[WiFi] connecting to wifi......")
    time.sleep(2)
    connected = True
    if connected:
        logging.info(f"[WiFi] connected successfully......")
        WIFI_FLAG = True
        GSM_MODE = False
        mqtt_client = mqtt.Client()
        mqtt_client.username_pw_set(mqtt_user, mqtt_password)
        mqtt_client.on_connect = on_connect
        mqtt_client.on_message = on_message
    else:
        logging.info(f" Wifi connection failed switching into GSM Mode....")
        WIFI_FLAG = False
        GSM_MODE = True
#calling the GSM initialisation function
GSMmodule.init_gsm()
#Wifi connected on the MQTT
def WiFiMqttConnect():
    global last_mqtt_attempt, communication_in_transition, GSM_MODE, WIFI_FLAG
    now = time.time()
    if not WIFI_FLAG:
        logging.info(f"WIfi MQTT not connected to wifi.....")
        return
    if not mqtt_client.is_connected() and now - last_mqtt_attempt > MQTT_RECONNECT_INTERVAL:
        logging.info(f"WIFI MQTT connected to the ambiator server.....")
        try:
           mqtt_client.connect(mqtt_server, mqtt_port, 60)
           mqtt_client.loop_start()
           logging.info(f"Wifi MQTT connected......")
        except Exception as e:
           logging.info(f" Wifi mqtt connection failed", {e})
           GSM_MODE = True
           WIFI_FLAG = False
           communication_in_transition = True
        last_mqtt_attempt = now

def connect_mqtt_via_gsm():
    logging.info(f"[GSM MQTT] attempting the connection via GSM......")
    try:
        mqtt_client.connect(mqtt_server, mqtt_port,60)
        mqtt_client.loop_start()
        logging.info(f"GSM MQTT connected successfully...")
        mqtt_client.subscribe(sub_topic)
    except Exception as e:
        logging.info(f"GSM connection lost.....",{e})

#MQTT connection
def MqttConnection():
    global wifi_reconnect_attempts, last_wifi_reconnect_time,communication_in_transition, GSM_MODE, WIFI_FLAG
    if communication_in_transition:
        mqtt_client.loop()
        return
    if not mqtt_client.is_connected():
        if GSM_MODE:
            logging.info(f"MQTT GSm Mode Activated.....")
            if GSMmodule.init_gsm():
                connect_mqtt_via_gsm()
            else:
                logging.info(f"GSM Initialisation Failed.....")
        if time.time() - last_wifi_reconnect_time > WIFI_RECONNECT_INTERVAL:
            WifiConnect()
            wifi_reconnect_attempts +=1
            last_wifi_reconnect_tiem = time.time()
            if wifi_reconnect_attempts >= MAX_WIFI_RECONNECT_ATTEMPTS:
                logging.info(f"mqtt too many wifi failures switching into GSM......")
                GSM_MODE = True
                WIFI_FLAG = False
                communication_in_transition = True
                mqtt_client.loop_stop()
                mqtt_client = mqtt.Client()
                mqtt_client.username_pw_set(mqtt_user, mqtt_password)
                mqtt_client.on_connect = on_connect
                mqtt_client.on_message = on_message
                connect_mqtt_via_gsm()
                communication_in_transition=False
        else:
            WiFiMqttConnect()
    else:
         mqtt_client.loop()

