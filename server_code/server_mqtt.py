#!/usr/bin/python3

import paho.mqtt.client as mqtt
import json
#import csv
import sys
import signal
import mysql.connector
from threading  import Thread
import datetime


#MQTT BROKER
MQTT_BROKER_ADDR  = "190.121.23.217"
MQTT_BROKER_PORT  =  1883
CHANNEL_SERVER_RX = "CIEP/tx"
CHANNEL_SERVER_TX = "CIEP/rx"

#DATA BASE
USER      = 'wladimir'
PASSWD    = 'Wladimir123-'
HOST      = '190.121.23.218'
DATABASE  = 'filtro'



# mosquitto_pub -h "190.121.23.217" -t "CIEP/tx" -m "{\"id\":\"CIEP-00\", \"type\":\"data\", \"time\": 1511118471, \"pm25\": 34.0, \"tn\": 29.7, \"hn\": 38.06948, \"ts\": 25.5, \"hs\": 25.5}"

dflt_row = "CIEP-00,0,20,25.5,55.5,25.5,55.5"
dflt_msg = "{\"id\":\"CIEP-00\", \"type\":\"data\", \"time\": 1511118471, \"pm25\": 34.0, \"tn\": 29.7, \"hn\": 38.06948, \"ts\": 25.5, \"hs\": 25.5}"
dflt_data = json.loads(dflt_msg)

'''
Comment:      This function recive a json dictionary and convert it to a row of csv file.
Description:  Join as string all values's item in a given dictionary.
'''
def json2csv(d):

  row = ""
  z = list(d.values()) 
  z = [str(e) for e in z]
  row = ",".join(z)
  return row

'''
Comment:      This function recive a json dictionary and convert it to a record format
              to post it in a mysql db.
Description:  Join as tuple all values's item in a json given as dictionary. 
'''
def json2db(d):
  row_list = []
  for key in d:
    if key != "type":
      row_list.append(str(d[key]))
  return tuple(row_list)

def signal_handler(signal, frame):
  print("exiting...")
  sys.exit(0)

def connected(client, userdata, flag , rc):
  print("server connected")
  client.subscribe(CHANNEL_SERVER_RX)
  client.publish(CHANNEL_SERVER_TX, "Server connected")

def rx_callback(client, userdata, msg):
  print("mensaje recibido : ", end="")
  print(msg.payload.decode("utf-8"))

  data = json.loads(msg.payload)  #convert payload to a dic
  
  db_row = json2db(data)
  my_cursor.execute(post_db_data,db_row)
  mydb.commit()
    

if __name__ == '__main__':
  print("starting test")
  
  print("connecting to mySQL database")
  mydb = mysql.connector.connect(user = USER, passwd = PASSWD,host = HOST,database = DATABASE)
  my_cursor = mydb.cursor()
  post_db_data  = "INSERT INTO test (id,time,pm25,tn,hn,ts,hs) VALUES (%s,%s,%s,%s,%s,%s,%s)"

  print("connecting to mqtt client")  
  client = mqtt.Client("server_test")
  client.on_message = rx_callback      #attach to callback function
  client.on_connect = connected        #attach initial subscribe test
    

  #client connect
  client.connect(MQTT_BROKER_ADDR,MQTT_BROKER_PORT,60)
  signal.signal(signal.SIGINT,signal_handler)

  while True:
    client.loop()