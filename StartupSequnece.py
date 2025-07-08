import time
import RPi.GPIO as GPIO # import RPi.GPIO as GPIO
import logging
import MQTTcomm
import json
import Firmware_ota
from TempHum import (
    Am2108sensorinit, InternalTemperatureinit,
    Outside_DB, Outside_RH,
    Supply_DB, Supply_RH,
    calculate_Wet_bulb, saturation_vapour_pressure
)
from Pump import checkWaterLevel 
from Systemcontrol import Pump_Pin ,Humidity_On_Solenoid, pump_flow_switch, Set_Point_Temperature,MODE,SETTING

# Utility for Arduino-like millis()
def millis():
    return int(time.time() * 1000)

HUMIDITY_INTERVAL_MS = 10_000  # e.g. 10 seconds or configure as before
last_humidity_pulse = 0
humidity_pulse_state = False

def startup_sequence():
    global humidity_pulse_state, last_humidity_pulse
    logging.info(" Entering startup_sequence…")
    MQTTcomm.MqttConnection()
    Firmware_ota.newCommandReceived()

    # Ensure initial water level safe
    while True:
        if not checkWaterLevel(fan_status=False):
            break
        time.sleep(0.5)

    # Initialize sensors
    Am2108sensorinit()
    InternalTemperatureinit()

    # Compute environmental values
    wb = calculate_Wet_bulb(Outside_DB, Outside_RH)
    es = saturation_vapour_pressure(Supply_DB)
    e = Supply_RH * es
    humidity_ratio = 0.62197 * (e / (101325 - e)) * 1000 + 0.22

    # Auto mode logic
    if MODE == 1:
        logging.info("🌡️ MODE 1 (Auto) - Startup control")

        # Humidity control decision
        temp_diff = abs(Outside_DB - wb)
        now = millis()
        if (temp_diff < 3.0 or humidity_ratio > 20.5):
            if SETTING != 1 and (now - last_humidity_pulse >= HUMIDITY_INTERVAL_MS):
                humidity_pulse_state = not humidity_pulse_state
                GPIO.output(Humidity_On_Solenoid, humidity_pulse_state)
                last_humidity_pulse = now
                logging.info(f"Humidity pulse: {humidity_pulse_state}")
        else:
            GPIO.output(Humidity_On_Solenoid, GPIO.LOW)
            humidity_pulse_state = False

        # Start pump and wait for flow
        GPIO.output(Pump_Pin, GPIO.HIGH)
        pump_start = millis()
        flow_ok = False
        timeout_ms = 20_000
        logging.info("Waiting for water flow...")

        while millis() - pump_start < timeout_ms:
            if GPIO.input(pump_flow_switch) == GPIO.LOW:
                flow_ok = True
                logging.info("✅ Flow detected.")
                break
            time.sleep(0.05)

        if flow_ok:
            Firmware_ota.upload_message(json.dumps({"PFF": 0}))
        else:
            GPIO.output(Pump_Pin, GPIO.LOW)
            Firmware_ota.upload_message(json.dumps({"PFF": 1}))

        # Winter shutdown of pump
        if Outside_DB < (Set_Point_Temperature - 6):
            GPIO.output(Pump_Pin, GPIO.LOW)
            logging.info("❄️ Low temperature – pump shut off.")
