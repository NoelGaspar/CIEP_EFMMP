#!/usr/bin/python3

import paho.mqtt.client as mqtt
import json
import csv
import sys
import signal
from threading  import Thread
from datetime   import datetime

MQTT_BROKER_ADDR  = "192.168.0.9"
MQTT_BROKER_PORT  = 1883
CHANNEL_SERVER_RX = "test1"
CHANNEL_SERVER_TX = "test2"

#default_msg = "{\"data1\":000 , \"data2\":\"fail\" , \"data3\": 000}"

default_msg = "{'data1':000 , 'data2': 'fail' , 'data3': 000}"
default_dct = json.loads(default_msg)
header      = default_dct.keys()
filename    = datetime.now().strftime("%d%m%Y%I%M%S")+".csv"
file        = open(filename,"w",newline='')
writer      = csv.DictWriter(file,fieldnames = header,lineterminator = '\n')
writer.writeheader()

def signal_handler(signal, frame):
  print("exiting...")
  file.close()
  print("file closed")
  sys.exit(0)

def connected(client, userdata, flag , rc):
  print("server connected")
  client.subscribe(CHANNEL_SERVER_RX)
  client.publish(CHANNEL_SERVER_TX, "HELLO WORD")

def rx_callback(client, userdata, msg):
  print("mensaje recibido ", end="")
  print(msg.payload.decode("utf-8"))
  data = json.loads(msg.payload)
  print(f'data : {data}')
  try:
    writer.writerow(data)
  except:
    writer.writerow(default_msg)


if __name__ == '__main__':
  print("starting test")
    
  client = mqtt.Client("server_test")
  client.on_message = rx_callback      #attach to callback function
  client.on_connect = connected        #attach initial subscribe test

  client.connect(MQTT_BROKER_ADDR,MQTT_BROKER_PORT,60)

  signal.signal(signal.SIGINT,signal_handler)

  while True:
    client.loop()

