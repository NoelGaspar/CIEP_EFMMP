import paho.mqtt.client as mqtt
import json
from threading import Thread


MQTT_BROKER_ADDR = "192.168.0.9"
MQTT_BROKER_PORT = 1883
CHANNEL_SERVER_RX = "test1"
CHANNEL_SERVER_TX = "test2"


def connected(client, userdata, flag , rc):
  print("server connected")
  client.subscribe(CHANNEL_SERVER_RX)
  client.publish(CHANNEL_SERVER_TX, "HELLO WORD")

def rx_callback(client, userdata, msg):
  print("mensaje recibido ", end="")
  data_rx = msg.payload.decode("utf-8")
  data = json.load(data_rx)
  for k in data:
    print(k, end=" ")
  print(" ")  



if __name__ == '__main__':
  print("starting test")
  client = mqtt.Client("server_test")
  client.on_message = rx_callback      #attach to callback function
  client.on_connect = connected        #attach initial subscribe test

  client.connect(MQTT_BROKER_ADDR,MQTT_BROKER_PORT,60)

  while True:
    client.loop()

