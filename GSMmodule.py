import time
import paho.mqtt.client as mqtt
import threading
import logging
import logfunc
import RPi.GPIO as GPIO
import serial
from MQTTcomm import MqttConnection, mqttClient

#adjusting the GPIO pin connected to POWERKEY
POWER_KEY_PIN = 4

def power_on_modem():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(POWER_KEY_PIN, GPIO.OUT, initial=GPIO.HIGH)
    logging.info(f"powering on the modem.....")
    GPIO.output(POWER_KEY_PIN, GPIO.LOW)
    time.sleep(1.0)
    GPIO.output(POWER_KEY_PIN, GPIO.HIGH)
    time.sleep(3.0)
    GPIO.output(POWER_KEY_PIN, GPIO.LOW)
    time.sleep(12.0)
    logging.info(f"power on sequnece complete.....")

#serial interface of the GSM module.......
SERIAL_PORT = "/dev/ttyUSB2"
BAUDRATE = 115200
APN = "www"
USER = ""
PASS = ""

MQTT_SERVER = ""
MQTT_PORT = 1883
MQTT_USER = ""
MQTT_PASS = ""
DEVICE_ID = ""
SUB_TOPIC = ""

mqtt_client = mqtt.Client()

def send_At(Ser, cmd, expected ="OK", timeout = 2):
    Ser.write((cmd + "\r\n").encode())
    time.sleep(timeout)
    resp = Ser.read_All().decode(errors='ignore')
    return expected in resp, resp

def connect_gprs():
    try:
        with serial.Serial(
            SERIAL_PORT,
            BAUDRATE, 
            timeout=1.0,
            xonxoff=False, 
            rtscts=False, 
            dsrdtr=True
        ) as ser:
            #Basic AT check
            ok, _ = send_At(ser, "AT")
            if not ok:
                logging.info(f"Modem not responding........")
                return False
            send_At(ser, f'AT+QICSGP=1,1,"{APN}","{USER}","{PASS}",1')
            send_At(ser, "AT+QIREGAPP")
            for attempt in range(1,4):
                logging.info(f"GPRS attempt {attempt}/3")
                send, resp = send_At(ser, "AT+QIACT=1", expected="OK", timeout=5)
                if send:
                    logging.info(f"GPRS CONNECTED SUCCESSFULLY.....")
                    return True
                logging.info("Failed: Retryinhg in 5sec....", resp.strip())
                time.sleep(5)
            logging.info(f"GPRS failed after 3 attempts......")
            return False
    except Exception as e:
        logging.error(f"GPRS connection error:{e}")
        return False

def init_gsm():
    with serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1) as ser:
         ok, _ = send_At(ser,"AT")
         if not ok:
             logging.info(f"GSM not responding....")
             return False
         if not send_At(ser, f'AT+QICSGP=1,1,"{APN}","{USER}","{PASS}", 1')[0]:
             logging.info(f"Falied to set APN...")
             return False
         if not send_At(ser, "AT+QIREGAPP")[0]:
             logging.info(f"Failed to register PDP contect....")
             return False
         ok = False
         for _ in range(3):
             ok = send_At(ser, "AT+QIACT=1", timeout=5)[0]
             if ok:
                 break
             time.sleep(5)
         if not ok:
             logging.info("GPRS attach failed.....")
         return ok

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        logging.info(f"MQTT connected......")
        client.subscribe(SUB_TOPIC)
    else:
        logging.info(f"MQTT failed, return code", rc)

def on_message(clent, userdata, msg):
    logging.info(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

#GSM+MQTT configuration

def gsm_config():
    logging.info(f"setting up GSM MQTT connection......")
    if not init_gsm():
        logging.info(f"GSM initialisation failed......")
        return None
    client = mqtt_client(DEVICE_ID)
    client.username_pw_Set(MQTT_USER, MQTT_PASS)
    client.on_connect = on_connect
    client.on_message = on_message
    client.socket = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)
    logging.info(f"connecting to MQTT broker......")
    try:
        client.connect(MQTT_SERVER, MQTT_PORT, keepalive=60)
    except Exception as e:
        logging.info(f" MQTT connection failed:", e)
        return None
    threading.Thread(target=client.loop_forever, daemon=True).start()
    return client
