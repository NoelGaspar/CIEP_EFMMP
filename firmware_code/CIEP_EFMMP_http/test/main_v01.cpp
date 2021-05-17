/*
VERSION V 0.1.0b
author: WAC@IOWLABS

CIEP:Estaciones de monitoreo de material particulado.
description: This code read a PMsensor and publish the data to a server.
the protocols used to publish data are:
- Json to strcuture data
- MQTT to send data
- GSM/GPRS to transmit data (SIM800L module)
Aslo the node count with a SHT sensor to read external temp and humidity, a RTC module to get actual time,  and pre-chamber to acconditionate the samples. This prechamber 
count with a heater and 2 sensors: a termo sensor PT100  to read the temperature of the pre chamber, and a sensor to read the sample temp and humidity.
This code run over a esp32.

dependences:
- On board SIM800. Uart on pins 26 and 27
- On board LED. on pin 13
- STH31 sensor: externals temperature and humidity. I2C (addr: 0x78)
- RTC sensor: Real time clock. I2C (addr: 0x )
- 
- Heater: Drive the power to the heater. DAC on pin 25
- Pt100 sensor: Read the temperature of the pre-chamber. OneWire On pin 18
- Status LED: General propouse LED. On pin 15
- PMS: PM sensor. UART on pins 12 and 14.
- DHT sensor: Read the internal temp and humedity of the node. on pin 19




The json strutures are:

RX-MSG: Json structure of msg to recive
{
  "sensor": "em-ciep-05",
  "cmd": "LED",
  "arg1": 48.7560,
  "arg2": 48.75608
}


TX-MSG: Json structure of msg to send
{
  "sensor": "em-ciep-05",
  "time": 1351824120,
  "data": data[],
  "temp": 482.678,
  "humidity": 45.2432,
  "state": "ok"
}

TODOLIST:

-Read sensor PMS                         check
-Implement Json format to the output     check
-send data by gsm                        check
-send mqtt data                          check
-PROBE I2C bus to 400kHz                 check
-Read sensor SHT31                       check
-Probe DACs of the esp32                 check
-Probe DHT sensor.
-Probe pt100 sensor. 
-ADD automatic temperature control feature
-Probe OTA.

*/ 


/*-----------------
    Includes
-----------------*/
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_SHT31.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*----------------
    Defines
-----------------*/
//PIN DEFINITIONS
#define LED_PIN           13 //ON BOARD LED PIN
#define LED_STATUS_PIN    15 //STATUS LED
#define HEATER_PIN        25 //DAC OUTPUT PIN
#define PT100_SENSOR_PIN  18 //PRE CHAMBER TEMPERATURE SENSOR
#define DHT_SENSOR_PIN    19 //I2C SCL

#define SCL_PIN   34  // SPI SCL
#define MOSI_PIN  32  // SPI MOSI
#define MISO_PIN  35  // SPI MOSI
#define CS_PIN    33  // SPI CS

//SENSOR PMS UART
#define PMS_TX    14  //UART PMS TX
#define PMS_RX    12  //UART PMS RX

//MODEM PINS
#define MODEM_RST       5
#define MODEM_PWKEY     4
#define MODEM_POWER_ON  23
#define MODEM_TX        27
#define MODEM_RX        26
#define MODEM_SDA       21
#define MODEM_SCL       22

//SERIAL DEFINITIONS
#define SerialDBG       Serial
#define SerialSIM       Serial1
#define SerialPMS       Serial2
#define BAUDRATE_DEBUG  9600
#define BAUDRATE_SIM    9600
#define BAUDRATE_PMS    9600
#define PMS5003ST
//#define PMS5003 //FOR D AND C sensors

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800       // Modem is SIM800
#define TINY_GSM_RX_BUFFER    1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <PubSubClient.h>

//NODE DEFINITIONS
#define SENSOR_ID         "em-ciep-03"// NODE ID
#define VESRION           "V0.0.1"    // ACTUAL CODE VERSION
#define RESPONSE_OK       "ok"        // RESPONSE TO INCOMMING CMD
#define RESPONSE_FAIL     "error"     // RESPONSE TO INCOMMING CMD
#define RESPONSE_ACQ      "reading"   // RESPONSE TO INCOMMING CMD
#define RESPONSE_START    "start_ok"  // RESPONSE TO INCOMMING CMD
#define RESPONSE_STOP     "stop_ok"   // RESPONSE TO INCOMMING CMD
#define TYPE_DATA         "data"      // TYPE OF RESPONSE. DATA: DATA FROM SENSOR
#define TYPE_RESP         "resp"      // TYPE OF RESPONSE. RESP: RESPONSE TO INCOMMING CMD

#define TIMER_F_SCLK      10000       // TIMER SOURCE CLOCK. IN kHz
#define TIMER_DEFAULT_TS  8           // TIMER DEFAULT PERIOD. IN SECONDS 
#define MAX_PMS_BUFF_SIZE 20          // SIZE OF THE ARRAY OF DATA TO SEND FORM THE PMS
#define HEATER_LIMIT      80          // SOFTWARE LIMIT FOR THE HEATER. OUTPUT RANGE : 0 .. 255

//SHT31
#define SHT31_ADDR        0x44
#define DHTTYPE           DHT22

//GPRS sim credentials
const char apn[]      = "internet.emt.ee"; //"freeeway"; // APN from PRIMCARDS
const char gprsUser[] = "";         // GPRS User
const char gprsPass[] = "";         // GPRS Password

//MQTT definitions
const char* mqtt_server = "190.121.23.217"; //mqtt IP broker (string)
#define mqtt_port     1883                  //mqtt port (int)
#define MQTT_USER     ""                    //mqtt user
#define MQTT_PASSWORD ""                    //mqtt passwd
#define MQTT_PUBLISH_CH   "pm/test1"        // CHANNEL TO PUBLISH
#define MQTT_RECEIVER_CH  "pm/test2"        // CHANNEL TO SUBSCRIBE (RECIVE COMMANDS)

//POWER TTGO-TCALL MODULE  DEFINITION 
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00


/*-----------------
    Instances
-----------------*/

char json_tx[1024];

//JSON to RECIVE
const size_t capacity_rx = JSON_OBJECT_SIZE(4) + 40;
DynamicJsonDocument doc_rx(capacity_rx);
const char* json_rx =  "{\"id\":\"em-ciep-02\",\"cmd\":\"LED\",\"arg1\":48.756,\"arg2\":48.75608}";
DeserializationError error_rx;

//I2C BUSES 
TwoWire I2C_BUS = TwoWire(0);

//Client and mqtt
TinyGsm       modem(SerialSIM);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

//timers 
hw_timer_t * timer = NULL;    //timer to adquisition of sensors
hw_timer_t * timerhm = NULL;  //timer to control the heater

//SHT31 sensor
Adafruit_SHT31 sht31 = Adafruit_SHT31();

//DHT sensor
DHT dht(DHT_SENSOR_PIN,DHTTYPE);

//oneWire sensor
OneWire oneWire(PT100_SENSOR_PIN);
DallasTemperature pt100(&oneWire);

/*----------------------------
    VARIABLES AND STRUCTURES
-----------------------------*/
//Structs to save sensor data
#ifdef PMS5003ST
  struct pms5003data
  {
    uint16_t framelen;
    uint16_t pm10_standard, pm25_standard, pm100_standard;
    uint16_t pm10_env, pm25_env, pm100_env;
    uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;
    uint16_t pm_fc;
    uint16_t pm_t;
    uint16_t pm_h;
    uint16_t unused;
    uint16_t error;
    uint16_t checksum;
  };
#endif
#ifdef PMS5003
  struct pms5003data
  {
    uint16_t framelen;
    uint16_t pm10_standard, pm25_standard, pm100_standard;
    uint16_t pm10_env, pm25_env, pm100_env;
    uint16_t particles_03um, particles_05um, particles_10um, particles_25um, particles_50um, particles_100um;    
    uint16_t unused;
    uint16_t checksum;
  };
#endif
struct pms5003data data;

bool pms_data_ready = false ; // FLAG FOR PMS CORRECT ADQUISITION
long pms_count      = 0;      // COUNTER FOR CORRECT ADQUIRED DATA

//TIMER vars
int timer_adqt    = TIMER_DEFAULT_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME
int timer_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool timer_flag   = false;

//TIMER2 heater monitoring
int timerhm_adqt    = TIMER_DEFAULT_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME
int timerhm_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool timerhm_flag   = false;

//data buffer 
uint8_t pms_idx   = 0;                // INDEX OF THE PMS BUFFER ARRAY
bool    pms_flag  = false;            // SAMPLES ARE READY
uint16_t PMS_buff[MAX_PMS_BUFF_SIZE]; // PMS BUFFER ARRAY

//sensors vars
float SHT_temp_buff[MAX_PMS_BUFF_SIZE];
float SHT_rh_buff[MAX_PMS_BUFF_SIZE];
float sht_temp_avg  = 0.0;
float sht_rh_avg    = 0.0;
float sht_temp      = 0.0;
float sht_rh        = 0.0;

float DHT_temp_buff[MAX_PMS_BUFF_SIZE];
float DHT_rh_buff[MAX_PMS_BUFF_SIZE];
float dht_temp_avg  = 0.0;
float dht_rh_avg    = 0.0;
float dht_temp      = 0.0;
float dht_rh        = 0.0;

float PMS_temp_buff[MAX_PMS_BUFF_SIZE];
float PMS_rh_buff[MAX_PMS_BUFF_SIZE];
float pms_temp_avg  = 0.0;
float pms_rh_avg    = 0.0;
float pms_temp      = 0.0;
float pms_rh        = 0.0;

//LED
bool led_state    = false; // for test LED
bool led_s_state  = false; // for status LED

//HEATER 
uint8_t heater_value = 0;     // VARIABLE TO SAVE THE HEATER OUTPUT VALUE. RANGE : 0 TO 255
bool    heater_state = false; // VARIABLE TO TURN ON OFF THE HEATER.

//INCOMMING COMMAND VARIABLES
const char* rcv_id ;                    //RECIVED ID
const char* cmd;                        //RECIVED COMMAND
int param1 = 0;                         //RECIVED ARG 1
int param2 = 0;                         //RECIVED ARG 2
const char* response = RESPONSE_OK;     //RESPONSE TO INCOMMING CMD

/*-------------------
  Functions headers
-------------------*/
bool setPowerBoostKeepOn(int en);
void setup_gsm();
void reconnect();
void mqttCallback(char* topic, byte *payload, unsigned int length);
void processCmd(byte* payload, unsigned int length);
void publishSerialData(char *serialData);
void publishStatus(const char *resp);
void publishSensor(const char *resp);
void readSensors();
bool readPMSdata(Stream *s);
void IRAM_ATTR timerISR();
void IRAM_ATTR timer_HM_ISR();


void setup()
{
  //Serial
  SerialDBG.begin(BAUDRATE_DEBUG);
  SerialPMS.begin(BAUDRATE_PMS, SERIAL_8N1,PMS_RX,PMS_TX);
  delay(15);

  //I2C
  I2C_BUS.begin(MODEM_SDA,MODEM_SCL,400000);
  
   // Keep power when running from battery
  bool isOk = setPowerBoostKeepOn(1);
  SerialDBG.println(String("IP5306 KeepOn ") + (isOk ? "OK" : "FAIL"));

  // Set modem reset, enable, power pins
  pinMode(MODEM_PWKEY, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(MODEM_POWER_ON, OUTPUT);
  digitalWrite(MODEM_PWKEY, LOW);
  digitalWrite(MODEM_RST, HIGH);
  digitalWrite(MODEM_POWER_ON, HIGH);

  //onboard LED
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_STATUS_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  digitalWrite(LED_STATUS_PIN, LOW);

  //dac start on 0
  dacWrite(HEATER_PIN,0);

  //default dumy data sensors
  response = RESPONSE_OK;

  //Check digital dependenses
  if (!sht31.begin(SHT31_ADDR))
  {
    Serial.println("Couldn't find SHT31");
  }

  dht.begin();
  delay(150);
  pt100.begin();
  delay(150);
    
  //Set GSM module baud rate and UART pins
  SerialSIM.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

  //setup Timer
  timer = timerBegin(3, 8000, true);    //timer 3, prescaler 8000, counting up
  timerAttachInterrupt(timer,&timerISR,true); //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
  timerAlarmWrite(timer, timer_counter,true);   // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
  timerAlarmEnable(timer);              // enable CTC mode
  timerStop(timer);                     //stop timer

//  //setup Timer
//  timerhm = timerBegin(2, 8000, true);              //timer 2, prescaler 8000, counting up
//  timerAttachInterrupt(timerhm,&timer_HM_ISR,true); //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
//  timerAlarmWrite(timerhm, timerhm_counter,true);   // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
//  timerAlarmEnable(timerhm);              // enable CTC mode
//  timerStop(timerhm);                     //stop timer

  //GSM
  setup_gsm();

  //MQTT init
  mqtt.setServer(mqtt_server, mqtt_port);
  mqtt.setCallback(mqttCallback); 

  reconnect();
}


void loop()
{
  mqtt.loop();
  if(timer_flag)
  {
    SerialDBG.println("get data");
    readSensors();
    timer_flag = false;
  }
  if(pms_flag)
  {
    publishSensor(RESPONSE_ACQ);    //publish data
    pms_flag = false; 
  }
}

/*-------------------
  Functions
-------------------*/

void setup_gsm()
{
  delay(10);

  SerialDBG.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialDBG.print("Modem Info: ");
  SerialDBG.println(modemInfo);

  SerialDBG.print("Waiting for network...");
  if (!modem.waitForNetwork())
  {
    SerialDBG.println(" fail");
    delay(10000);
    return;
  }
  SerialDBG.println(" success");

  SerialDBG.print("Connecting to the network...");
  if (!modem.isNetworkConnected()) 
  {
    SerialDBG.println(" fail");  
  }
  SerialDBG.println("Network connected");

  SerialDBG.print("Connecting to ");
  SerialDBG.print(apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) 
  {
    SerialDBG.println(" fail");
    delay(10000);
    return;
  }
  SerialDBG.println(" success");

  SerialDBG.print("Checking connection...");
  if (!modem.isGprsConnected()) 
  {
    SerialDBG.println(" fail");
  }
  SerialDBG.println("GPRS connected");
}

void reconnect()
{ 
  while(!mqtt.connected())
  {
    SerialDBG.println("MQTT not connected");
    if (mqtt.connect(SENSOR_ID))
    //if (mqtt.connect(SENSOR_ID,MQTT_USER,MQTT_PASSWORD))  // if you are using psw and usser
    {
      SerialDBG.println("connected");
      mqtt.publish(MQTT_PUBLISH_CH, "hello world");
      mqtt.subscribe(MQTT_RECEIVER_CH);
    }
    else
    {
      SerialDBG.print("failed, rc=");
      SerialDBG.print(mqtt.state());
      SerialDBG.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte *payload, unsigned int length)
{
  SerialDBG.println("-------new message from broker-----");
  SerialDBG.print("channel:");
  SerialDBG.println(topic);
  SerialDBG.print("data:");
  SerialDBG.write(payload, length);
  SerialDBG.println();

  processCmd(payload,length);
}

void processCmd(byte* payload, unsigned int length)
{
  error_rx = deserializeJson(doc_rx, payload,length);
  if(error_rx)
  {
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error_rx.c_str());
    return;
  }
  rcv_id = doc_rx["id"];

  if(strcmp(rcv_id,SENSOR_ID)==0) // check if the message is for us
  {
    SerialDBG.println("ID OK");
    cmd     = doc_rx["cmd"];
    param1  = doc_rx["arg1"];
    param2  = doc_rx["arg2"];

    if(strcmp(cmd,"led") == 0)     // SET LED CMD: arg1 = led state, 
    {
      if(param1 == 1) 
      {
        if(param2 == 1)
        {
          digitalWrite(LED_PIN,param2);
          publishStatus(RESPONSE_OK);
        }
        else{digitalWrite(LED_PIN,0);}
      }
      if(param1 == 2) 
      {
        if(param2 ==1)
        {
          digitalWrite(LED_STATUS_PIN,param2);
          publishStatus(RESPONSE_OK);
        }
        else{digitalWrite(LED_STATUS_PIN,0);}  
      }
      
    }
    else if(strcmp(cmd,"heater_set") == 0) // SET HEATER value: arg1 = heater value
    {
      if( param1 > HEATER_LIMIT){ heater_value = HEATER_LIMIT; }
      else { heater_value = param1; }

      publishStatus(RESPONSE_OK);
    }
    else if(strcmp(cmd,"heater_on") == 0) // HEATER on off: arg1 = heater state
    {
      if( param1 == 1 )
      { 
        dacWrite(HEATER_PIN, heater_value); 
        heater_state = true;
      }
      if( param1 == 0 )
      { 
        dacWrite(HEATER_PIN, 0); 
        heater_state = false;
      }
    }
    else if(strcmp(cmd,"start") == 0)
    {
      SerialDBG.println("starting sampling");
      timerRestart(timer);
      publishStatus(RESPONSE_START); 
    }
    else if(strcmp(cmd,"stop") == 0)
    {
      timerStop(timer); 
      pms_idx = 0;
      publishStatus(RESPONSE_OK);
    }
    else if(strcmp(cmd,"set_freq") == 0)
    {
      timerStop(timer);
      timerAlarmDisable(timer);

      timer_adqt    = param1;
      timer_counter = int(timer_adqt*TIMER_F_SCLK);  
      
      timerAlarmWrite(timer,timer_counter,true);
      timerAlarmEnable(timer);              
      timerRestart(timer);
      publishStatus(RESPONSE_OK);
    }
    else if(strcmp(cmd,"Reset") == 0)
    {
      publishStatus(RESPONSE_OK);
      ESP.restart();
    }
    else
    {
      publishStatus(RESPONSE_FAIL);
    }
  }
  else
  {
    SerialDBG.println("ID fail");
  }
  
}

void publishSensor(const char * resp)
{ 
  #ifdef PMS5003ST
    const size_t capacity_tx = JSON_ARRAY_SIZE(20) + JSON_OBJECT_SIZE(11);
  #else
    const size_t capacity_tx = JSON_ARRAY_SIZE(20) + JSON_OBJECT_SIZE(9);
  #endif
  DynamicJsonDocument doc_tx(capacity_tx);
  
  doc_tx["id"]      = SENSOR_ID;
  doc_tx["type"]    = TYPE_DATA;
  doc_tx["time"]    = millis();
  doc_tx["count"]   = pms_count;

  JsonArray data_tx = doc_tx.createNestedArray("data");
  
  for(uint8_t i = 0; i< MAX_PMS_BUFF_SIZE ; i++)
  {
    data_tx.add(PMS_buff[i]);
  }
  
  sht_temp  = sht_temp_avg / 20.0;
  sht_rh    = sht_rh_avg   / 20.0;

  dht_temp  = dht_temp_avg / 20.0;
  dht_rh    = dht_rh_avg   / 20.0;

  doc_tx["sht_t"]  = sht_temp;
  doc_tx["sht_h"]  = sht_rh;
  doc_tx["dht_t"]  = dht_temp;
  doc_tx["dht_h"]  = dht_rh;
  #ifdef PMS5003ST
    pms_temp  = pms_temp_avg / 20.0;
    pms_rh    = pms_rh_avg   / 20.0;
    doc_tx["psm_t"]  = pms_temp;
    doc_tx["psm_h"]  = pms_rh;
  #endif
  
  serializeJson(doc_tx, json_tx);
  SerialDBG.println("msg to publish: ");
  SerialDBG.println(json_tx);
  publishSerialData(json_tx);
}

void publishStatus(const char * resp)
{ 
  const size_t capacity_tx = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc_tx(capacity_tx);
  
  doc_tx["id"]      = SENSOR_ID;
  doc_tx["time"]    = millis();
  doc_tx["type"]    = TYPE_RESP;
  doc_tx["state"]   = resp;

  serializeJson(doc_tx, json_tx);
  SerialDBG.println("msg to publish: ");
  SerialDBG.println(json_tx);
  publishSerialData(json_tx);
}

void publishSerialData(char *serialData)
{
  if(!mqtt.connected())
  {
    reconnect();
  }

  mqtt.publish(MQTT_PUBLISH_CH, serialData);
}

void IRAM_ATTR timerISR()  //Timer ISR
{
  pms_idx++;
  timer_flag = true;
  if(pms_idx >= MAX_PMS_BUFF_SIZE)
  {
    pms_flag  = true;
    pms_idx   = 0;
  }
}

void IRAM_ATTR timer_HM_ISR()  //Timer ISR
{
  //update sensor temperature
  //if temp > limit then stop

}

void readSensors()
{
  while(!pms_data_ready)
  {
    pms_data_ready = readPMSdata(&SerialPMS);
  }

  if(pms_data_ready)
  {
    pms_count+=1;
    pms_data_ready = false;
  }

  PMS_buff[pms_idx] = data.pm25_standard;

  sht_rh_avg    -= SHT_rh_buff[pms_idx];
  sht_temp_avg  -= SHT_temp_buff[pms_idx];
  dht_rh_avg    -= DHT_rh_buff[pms_idx];
  dht_temp_avg  -= DHT_temp_buff[pms_idx];
  

  SHT_rh_buff[pms_idx]    = sht31.readHumidity();
  SHT_temp_buff[pms_idx]  = sht31.readTemperature();
  DHT_rh_buff[pms_idx]    = dht.readHumidity();
  DHT_temp_buff[pms_idx]  = dht.readTemperature();

  sht_rh_avg    += SHT_rh_buff[pms_idx];
  sht_temp_avg  += SHT_temp_buff[pms_idx];
  dht_rh_avg    += DHT_rh_buff[pms_idx];
  dht_temp_avg  += DHT_temp_buff[pms_idx];
 
  #ifdef PMS5003ST
    pms_rh_avg    -= PMS_rh_buff[pms_idx];
    pms_temp_avg  -= PMS_temp_buff[pms_idx];
    PMS_rh_buff[pms_idx]    = data.pm_h;
    PMS_temp_buff[pms_idx]  = data.pm_t;
    pms_rh_avg    += PMS_rh_buff[pms_idx];
    pms_temp_avg  += PMS_temp_buff[pms_idx];
  #endif
  
  SerialDBG.print("muestra: ");SerialDBG.print(pms_idx);
  SerialDBG.print(" valor: ");SerialDBG.println(PMS_buff[pms_idx]);
}

bool readPMSdata(Stream *s) 
{
#ifdef PMS5003
  if (!s->available()) 
  {
    return false;
  }

  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) 
  {
    s->read();
    return false;
  }

  // Now read all 32 bytes
  if (s->available() < 32) 
  {
    return false;
  }

  uint8_t buffer[32];
  uint16_t sum = 0;
  s->readBytes(buffer, 32);

  // get checksum ready
  for(uint8_t i=0; i<30; i++) 
  {
    sum += buffer[i];
  }

  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[15];
  for(uint8_t i=0; i<15; i++)
  {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 30);

  if(sum != data.checksum)
  {
    // Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
#endif
#ifdef PMS5003ST
  if (!s->available()) 
  {
    return false;
  }
  // Read a byte at a time until we get to the special '0x42' start-byte
  if (s->peek() != 0x42) 
  {
    s->read();
    return false;
  }
  // Now read all 32 bytes
  if (s->available() < 40) 
  {
    return false;
  }

  uint8_t buffer[40];
  uint16_t sum = 0;
  s->readBytes(buffer, 40);
  // get checksum ready
  for(uint8_t i=0; i<38; i++) 
  {
    sum += buffer[i];
  }
  // The data comes in endian'd, this solves it so it works on all platforms
  uint16_t buffer_u16[19];
  for(uint8_t i=0; i<19; i++)
  {
    buffer_u16[i] = buffer[2 + i*2 + 1];
    buffer_u16[i] += (buffer[2 + i*2] << 8);
  }

  // put it into a nice struct :)
  memcpy((void *)&data, (void *)buffer_u16, 38);

  if(sum != data.checksum)
  {
    // Serial.println("Checksum failure");
    return false;
  }
  // success!
  return true;
#endif
}

bool setPowerBoostKeepOn(int en)
{
  I2C_BUS.beginTransmission(IP5306_ADDR);
  I2C_BUS.write(IP5306_REG_SYS_CTL0);
  if (en)
  {
    I2C_BUS.write(0x37); // Set bit1: 1 enable 0 disable boost keep on
  } 
  else 
  {
    I2C_BUS.write(0x35); // 0x37 is default reg value
  }
  return I2C_BUS.endTransmission() == 0;
}