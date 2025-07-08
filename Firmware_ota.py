import Motor
import subprocess
import socket
import time
import serial
import json
import RPi.GPIO as GPIO
import logging
import logfunc
import StartupSequnece
from CONFIG import update_wpa_supplicant
from MQTTcomm import MqttConnection , WiFiMqttConnect
from Systemcontrol import SelfCareRoutine
from GSMmodule import gsm_config
from git_clone import clone_repo

def reboot_device():
      logging.info("Rebooting the device now...")
      try:
          # Reboot the system
          subprocess.run(["sudo", "reboot"], check=True)
      except Exception as e:
          logging.error(f"Failed to reboot device: {e}")  


# --- Constants & Globals ---
communication_in_transition = False
Water_Supply_Solenoid = 12
Pump_Pin = 10
Motor_Enable_Pin = 14
Humidity_On_Solenoid = 13
Drain_Solenoid = 15
Led_Pin = 16
FILTER_CLEAN = 17


deviceStatus = 0
MODE = 0
SETTING = 0
HUM = 0
FS = 0
SH = False
FC = False
wifiSSID = ""
wifiPASSWORD = ""
Set_Point_Temperature = 0.0
GSM_MODE = False
WIFI_FLAG = False

mqtt_client = None
pub_topic = "" 

# GPIO Setup 
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(Water_Supply_Solenoid, GPIO.OUT)
GPIO.setup(Humidity_On_Solenoid, GPIO.OUT)
GPIO.setup(Pump_Pin, GPIO.OUT)
GPIO.setup(Drain_Solenoid, GPIO.OUT)
GPIO.setup(Led_Pin, GPIO.OUT)
GPIO.setup(FILTER_CLEAN, GPIO.IN)

def upload_message(msg: str):
      global communication_in_transition, mqtt_client, pub_topic
    
      # Logging instead of Serial.println
      if not communication_in_transition:
          logging.info(f"Uploading messages: {msg}")
      else:
          logging.info(f"Self-Care Messages: {msg}")
    
      if mqtt_client is None:
          logging.info("MQTT client not initialized, cannot upload message")
          return
    
      if communication_in_transition:
          self_care_messages = [
              "SHONGPRC1", "SHONGPRC2", "SHONGPRC3",
              "SHONGPRC4", "SHONGPRC5", "SHONGPRC6",
              "SHCMP", "EMERGENCY", "0", "1"
          ]
          is_allowed = any(allowed_msg in msg for allowed_msg in self_care_messages)
          if not is_allowed:
              logging.info("Blocked message during communication transition")
              return
    
      # Check MQTT connection
      if not mqtt_client.is_connected():
          logging.info("MQTT not connected, attempting to reconnect")
          MqttConnection()
          if not mqtt_client.is_connected():
              logging.info("MQTT reconnection failed, message not sent")
              return
    
      # Publish message
      result = mqtt_client.publish(pub_topic, msg)
      # result is an MQTTMessageInfo object in paho-mqtt
      if result.rc == 0:
          logging.info("Message published successfully")
      else:
          logging.info("Message publish failed")  


def WiFi_status(timeout=3):
      """Check if network is connected (similar to WiFi.status() == WL_CONNECTED)."""
      try:
          socket.setdefaulttimeout(timeout)
          socket.socket(socket.AF_INET, socket.SOCK_STREAM).connect(("8.8.8.8", 53))
          return True
      except Exception:
          return False


def gsm_status(port='/dev/ttyS0', baud=115200, timeout=2):
      with serial.Serial(port, baudrate=baud, timeout=timeout) as ser:
          ser.write(b'AT+CREG?\r')
          time.sleep(0.5)
          resp = ser.read(100).decode(errors='ignore')
          return any(s in resp for s in ['+CREG: 0,1', '+CREG: 0,5'])
        
def network_status():
      if WiFi_status():
          return 'wifi'
      elif gsm_status():
          return 'gsm'
      return None  # no connection
  
def firmware():
      logging.info("Starting custom firmware update process")
    
      status = network_status()
      if status == 'wifi':
          logging.info("Connected via Wi‑Fi")
      elif status == 'gsm':
          logging.info("Connected via GSM")
      else:
          logging.info("No network connection")
          return
      
      logging.info("Network connected")
      # Send acknowledgment like "OTAOK"
      upload_message("OTAOK")
      time.sleep(0.5)
    
      repo_path = "/ProtoDevelopementpi/update_ota"  # Your repo path
      try:
          # Pull latest changes from remote repo
          logging.info(f"Pulling latest changes in {repo_path}")
          proc = subprocess.run(
              ["git", "-C", repo_path, "pull"],
              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
          )
        
          logging.info(proc.stdout)
        
          if proc.returncode != 0:
              logging.info("Git pull failed:")
              logging.info(proc.stderr)
              upload_message("ERROR")
              return
        
          # Optionally verify if new commit pulled
          # For simplicity, assume pull success means update
        
          # Notify success
          logging.info("Update successful!")
          upload_message("OK")
        
          time.sleep(1)
        
          # Restart your app or whole device here
          # Example: restart a systemd service
          restart_proc = subprocess.run(
              ["sudo", "systemctl", "restart", "myapp.service"],
              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True
          )
          if restart_proc.returncode != 0:
              logging.info("Failed to restart app/service:")
              logging.info(restart_proc.stderr)
              upload_message("ERROR")
              return
        
          print("Restarted app/service successfully")
        
      except Exception as e:
          logging.info(f"Exception during firmware update: {e}")
          upload_message("ERROR")
  
# Example of new command received handler (like your C++ code)
newMessageStatus = True
globalMessage = "some_command_or_json"
def newCommandReceived():
      global newMessageStatus, globalMessage
      if newMessageStatus:
          logging.info(globalMessage)
          newMessageStatus = False
          parse_json_data(globalMessage)                 
def parse_json_data(json_data):
      global deviceStatus, MODE, SETTING, HUM, FS, SH, FC
      global wifiSSID, wifiPASSWORD, Set_Point_Temperature, GSM_MODE, WIFI_FLAG

      if communication_in_transition:
          logging.info("Ignoring during transition")
          return

      try:
          doc = json.loads(json_data)
      except Exception as e:
          logging.info("JSON error:", e)
          return

      # STATUS handling
      if "STATUS" in doc:
          deviceStatus=doc["STATUS"]
          upload_message(f'{{"STATUS":{deviceStatus}}}')
          return
      
      if "MODE" in doc:
          MODE = doc["MODE"]
          upload_message("MODEOK")

      if "SETTING" in doc:
          SETTING = doc["SETTING"]
          upload_message("SETTINGOK")
        
      if "HUM" in doc:
          HUM = doc["HUM"]
          upload_message("HUMOK")

      # OTA
      if "OTA" in doc and doc["OTA"]:
          upload_message("OTAOK")
          time.sleep(0.5)
          clone_repo()

      # SP command
      if "SP" in doc:
          Set_Point_Temperature = doc["SP"]
          upload_message("SPOK")

      # SH command
      if "SH" in doc:
          SH = doc["SH"]
          upload_message("SHOK")
          if SH:
              SelfCareRoutine()

      # --- WiFi Credentials Handling ---
      if "wifiSSID" in doc and "wifiPASSWORD" in doc:
          ssid = doc["wifiSSID"].strip()
          pwd = doc["wifiPASSWORD"].strip()

          # Validate
          if not (0 < len(ssid) <= 18 and 0 < len(pwd) <= 19):
              upload_message("WIFI_UPDATE_ERROR_LENGTH")
              return

          if ' ' in ssid or ' ' in pwd:
              upload_message("WIFI_UPDATE_ERROR_SPACES")
              return

          # Try updating system Wi‑Fi
          if update_wpa_supplicant(ssid, pwd):
              wifiSSID, wifiPASSWORD = ssid, pwd
              upload_message("WIFI_UPDATED")
              WIFI_FLAG = True
              GSM_MODE = False
              WiFiMqttConnect()
          else:
              upload_message("WIFI_UPDATE_ERROR_CONN")
              GSM_MODE = True
              gsm_config()

      # FILTER Cleaning
      if "FILTER" in doc:
          FC = bool(doc["FILTER"])
          GPIO.output(Led_Pin, GPIO.HIGH)
          time.sleep(3)
          status = GPIO.input(FILTER_CLEAN)
          upload_message(f'{{"FC":{1 if status else 0}}}')
          time.sleep(5)
          GPIO.output(Led_Pin, GPIO.LOW)

      # RESTART
      if "RESTART" in doc:
          upload_message("RESTARTING")
          time.sleep(4)
          reboot_device()  
def is_emergency_message(message: str) -> bool:
      try:
          data = json.loads(message)
      except json.JSONDecodeError:
          # Fallback: raw string check for emergency keyword
          return '"type":"EMERGENCY"' in message

      # Check for numeric emergency stop
      if isinstance(data.get("STATUS"), int) and data["STATUS"] == 0:
          return True

      # Check for explicitly flagged emergency message
      if data.get("type") == "EMERGENCY":
          return True
      return False  