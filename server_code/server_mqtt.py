#!/usr/bin/python3

import paho.mqtt.client as mqtt
import json
import csv
import sys
import signal
import mysql.connector
from threading  import Thread
from datetime   import datetime


#MQTT BROKER
MQTT_BROKER_ADDR  = "190.121.23.217"
MQTT_BROKER_PORT  = 1883
CHANNEL_SERVER_RX = "pm/test1"
CHANNEL_SERVER_TX = "pm/test2"
#CHANNEL_SERVER_RX = "pmt/test1"
#CHANNEL_SERVER_TX = "pmt/test2"



#DATA BASE
USER      = 'root'
PASSWD    = 'Efmmp_2019!'
HOST      = 'localhost'
DATABASE  = 'nodes'


default_row = "em-ciep-00,0,20,25.5,55.5,25.5,55.5,12,25.5,55.5"
default_msg = {"id": "em-ciep-01", "type": "data", "time": 000, "count": 0, "data": [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], "sht_t": 29.78752, "sht_h": 38.06948, "dht_t": 25.5, "dht_h": 25.5, "psm_t": 296.7, "psm_h": 299.85}

#CSV
path_csv = "data/"

def json2csv(d):
  row = ""
  z = list(d.values()) 
  z = [str(e) for e in z]
  row = ",".join(z)
  return row

def json2db(d):
  row_list = []
  for key in d:
    if key != "type":
      if key == "data":  
        row_list.append(str(d[key][0]))
      else:
        row_list.append(str(d[key]))
  row = ",".join(row_list)
  print(row)
  return tuple(row_list)

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
  print("mensaje recibido : ", end="")
  print(msg.payload.decode("utf-8"))
  data = json.loads(msg.payload)
  if data['type'] == "data": 
    N = int(data['count'])
    try:                    
      print("guardando local:", end="")
      print(data)
      writer.writerow(data)
    except:
      writer.writerow(default_msg)

    db_row = json2db(data)
    
    my_cursor.execute(post_db_data,db_row)
    mydb.commit()
    

if __name__ == '__main__':
  print("starting test")
  
  print("connecting to mySQL database")
  mydb = mysql.connector.connect(user = USER, passwd = PASSWD,host = HOST,database = DATABASE)
  my_cursor = mydb.cursor()
  post_db_data  = "INSERT INTO ciep_test (node_id,time_,count,pms_25,sht_t,sht_h,dht_t,dht_h,pms_t,pms_h) VALUES (%s,%s,%s,%s,%s,%s,%s,%s,%s,%s)"

  print("open csv file")
  default_json = '{"id":"node_nn", "type":"data","time": 0,"count":0,"data":0,"sht_t":0.0,"sht_t":0.0,"sht_h":0.0,"dht_t":0.0,"dht_h":0.0,"psm_t":0.0,"psm_h":0.0}'
  default_dic = json.loads(default_json)

  #mosquitto_pub -h "190.121.23.217" -t "CIEP/TX" -m "{\"id\":\"node_nn\", \"type\":\"data\",\"time\": 0,"count":0,\"data\": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"sht_t\":0.0,\"sht_t\":0.0,\"sht_h\":0.0,\"dht_t\":0.0,\"dht_h\":0.0,\"pms_t\":0.0,\"pms_h\":0.0}" 
  header      = default_dic.keys() 
  filename    = path_csv+datetime.now().strftime("%d%m%y_%H%M%S")+".csv"
  file        = open(filename,"w",newline='')
  writer      = csv.DictWriter(file,fieldnames = header,lineterminator = '\n')
  writer.writeheader()


  print("connecting to mqtt client")  
  client = mqtt.Client("server_test")
  client.on_message = rx_callback      #attach to callback function
  client.on_connect = connected        #attach initial subscribe test
  
  
  #client connect
  client.connect(MQTT_BROKER_ADDR,MQTT_BROKER_PORT,60)

  signal.signal(signal.SIGINT,signal_handler)

  while True:
    client.loop()