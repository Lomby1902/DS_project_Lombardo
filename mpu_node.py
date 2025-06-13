#!/usr/bin/env python3
import os
import sys
import time
import random
import paho.mqtt.client as mqtt
import logging
logging.basicConfig(level=logging.DEBUG)


FIRMWARE_PATH = "F446_APP.bin"

# MQTT configuration
BROKER_ADDRESS = "192.168.10.231" 
BROKER_PORT = 1883
TOPIC_FIRMWARE = "firmware"




def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))



def read_firmware(path):
    if not os.path.isfile(path):
        print(f"File error: {path}")
        sys.exit(1)
    chunks = []
    with open(path, "rb") as f:
        firmware = f.read()

    return firmware

def send_firmware_via_mqtt():

    firmware = read_firmware(FIRMWARE_PATH)

    client =  mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message   
    client.connect(BROKER_ADDRESS, BROKER_PORT)
    client.loop_start()


    #Wait for connection
    time.sleep(1)
    result = client.publish(TOPIC_FIRMWARE, payload=firmware, qos=1)
    if result.rc == mqtt.MQTT_ERR_SUCCESS:
        print("Firmware sent")

    time.sleep(1)
    client.loop_stop()
    client.disconnect()

if __name__ == "__main__":
    send_firmware_via_mqtt()


