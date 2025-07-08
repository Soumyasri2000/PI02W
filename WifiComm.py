
import paho.mqtt.client as mqtt
import subprocess
import time
import json
import StartupSequnece


# MQTT settings
BROKER_HOST = "test.mosquitto.org"
BROKER_PORT = 1883
TOPICS = ["pi/wifi/ssid", "pi/wifi/password", "pi/wifi/connect"]

wifi_ssid = ""
wifi_password = ""

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    for topic in TOPICS:
        client.subscribe(topic)

def on_message(client, userdata, msg):
    global wifi_ssid, wifi_password
    topic = msg.topic
    payload = msg.payload.decode()

    try:
        data = json.loads(payload)
    except Exception as e:
        print(f" Invalid JSOn payload on topic {topic}: {payload}")
        return
    
    if topic == "pi/wifi/ssid":
        wifi_ssid = data.get("ssid","")
        print(f"Received SSID: {wifi_ssid}")

    elif topic == "pi/wifi/password":
        wifi_password = data.get("password", "")
        print(f"Received password")

    elif topic == "pi/wifi/connect":
        if wifi_ssid and wifi_password:
            success = configure_wifi(wifi_ssid, wifi_password)
            if success:
                print(" WIFI configuration updated successfully")
            else:
                print("Failed to configure WIFI")

def configure_wifi(ssid, password):
    if "{" in ssid or "}" in ssid:
        try:
            ssid = json.loads(ssid).get("ssid", "")
        except:
            print(" could not parse SSID from JSOn string")
            return False
    if "{" in password or "}" in password:
        try:
            password = json.loads(password).get("password", "")
        except:
            print("could not parse password from the JSON string")
            return False
    try:
        # run wpa_passphrase
        result = subprocess.run(['wpa_passphrase', ssid, password], capture_output=True, text=True, check=True)
        wpa_output = result.stdout
    except Exception as e:
        print(f" Failed to run the wpa_passphrase: {e}")
        return False
    network_block_lines = []
    in_network_block = False
    for line in wpa_output.splitlines():
        if line.strip().startswith("network={"):
            in_network_block = True
            network_block_lines.append(line)
        elif line.strip().startswith("}"):
            network_block_lines.append(line)
            in_network_block = False
        elif in_network_block:
            #skip plain text line password
            if not line.strip().startswith("#psk="):
               network_block_lines.append(line)
    network_block = "\n".join(network_block_lines)
    config = f"""ctrl_interface=DIR=/var/run/wpa_supplicant GROUP=netdev
update_config=1
country=IN

network={{
    ssid="{ssid}"
    psk="{password}"
    key_mgmt=WPA-PSK
}}
"""

    try:
        # Overwrite the wpa_supplicant.conf file
        with open('/etc/wpa_supplicant/wpa_supplicant.conf', 'a') as f:
            f.write(config)
        print(" wpa_supplicant.conf updated")
        print(" Reconfiguring wifi using upa_cli")

        # Apply changes immediately
        subprocess.run(['sudo', 'wpa_cli', '-i', 'wlan0', 'reconfigure'], check=True)
        print(" wifi reconfiguration triggered")
        print("Wi-Fi configuration updated and reconfigured successfully.")
        time.sleep(10)
        result = subprocess.run(["ping", "-c", "3", "8.8.8.8"])
        if result.returncode == 0:
            print("Interent pinging successfully....")
            return True
        else:
            print("failed to connect the internet.....")
            return False 
    except Exception as e:
        print(f"Error configuring WIFI: {e}")
        return False

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message

client.connect(BROKER_HOST, BROKER_PORT, 60)
client.loop_forever()
StartupSequnece.Startup_sequence()

