#!/usr/bin/python3

import paho.mqtt.client as mqtt
import json
import csv
import sys
import signal
import mysql.connector
from threading  import Thread
from datetime import datetime, date, time, timedelta


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

dflt_msg      = "{\"id\":\"em-ciep-01\", \"type\":\"data\", \"time\": 0, \"count\": 0, \"data\": [1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0], \"sht_t\": 29.78752, \"sht_h\": 38.06948, \"dht_t\": 25.5, \"dht_h\": 25.5, \"psm_t\": 296.7, \"psm_h\": 299.85}"
recv_dic_dflt     = json.loads(dflt_msg)
dic_dflt = {"s_codigo_cf":"em-ciep-00","fecha":191120,"hora":15826,"reg_val":25, "time_s":"1:58:26"}
db_row_dflt   = ("em-ciep-00",191120,15826,25)

#CSV
path_csv = "data/fecha_hora/"

def json2db(d):
  row_list = []
  for key in d:
    row_list.append(str(d[key]))      
  row = ",".join(row_list)
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
  try:
    data = json.loads(msg.payload)  #convert payload to a dic
  except:
    data = recv_dic_dflt

  if data['type'] == "data":  #check if the type of msg is "data"
    t  = datetime.now()
    t  = t - timedelta(0,160)

    for i in range(0,20):
      t = t + timedelta(0,8)

      fecha = t.strftime("%d%m%y")
      hora  = t.strftime("%H%M%S")

      recv_dic = {}
      recv_dic["s_codigo_cf"] = data["id"]
      recv_dic["fecha"]       = int(fecha)
      recv_dic["hora"]        = int(hora)
      recv_dic["reg_val"]     = data["data"][i]
      recv_dic["time_s"]        = t.strftime("%H:%M:%S")

      try:
        writer.writerow(recv_dic)
      except:
        writer.writerow(dic_dflt)

      db_row = json2db(recv_dic)
      try:
        my_cursor.execute(post_db_data,db_row)
        mydb.commit()
      except:
        my_cursor.execute(post_db_data,db_row_dflt)
        mydb.commit()
      

if __name__ == '__main__':
  print("starting test")
  
  print("connecting to mySQL database")
  mydb = mysql.connector.connect(user = USER, passwd = PASSWD,host = HOST,database = DATABASE)
  my_cursor = mydb.cursor()
  post_db_data  = "INSERT INTO fecha_hora_test (s_codigo_cf,fecha,hora,reg_val, time_s) VALUES (%s,%s,%s,%s,%s)"

  print("open csv file")

  #mosquitto_pub -h "190.121.23.217" -t "CIEP/TX" -m "{\"id\":\"node_nn\", \"type\":\"data\",\"time\": 0,"count":0,\"data\": [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],\"sht_t\":0.0,\"sht_t\":0.0,\"sht_h\":0.0,\"dht_t\":0.0,\"dht_h\":0.0,\"pms_t\":0.0,\"pms_h\":0.0}" 
  header      = dic_dflt.keys()
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