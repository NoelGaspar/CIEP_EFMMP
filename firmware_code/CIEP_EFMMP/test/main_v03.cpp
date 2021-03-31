/*


VESRION : V 0.2.1 b
AUTOR   : WAC@IOWLABS

CIEP        : Estaciones de monitoreo de material particulado.
DESCRIPTION : This code read a PMsensor and publish the data to a server.
  the protocols used to publish data are:
    - Json to strcuture data
    - MQTT to publish data
    - GSM/GPRS to transmit data (SIM800L module)
  Aslo the node count with a 
    - DHT sensor to read external temp and humidity, 
    - RTC module to get actual time
    - SD  card to save data localy
    - RGB PM25 status LED
    - LED to show node status
  and pre-chamber to acconditionate the samples. This prechamber count with
    - heater  
    - a Temperatura sensor PT100  to read the temperature of the pre chamber, and  
    - Sensor to read the sample temp and humidity.
  
  This code run over a esp32.

dependences:
- On board SIM800. Uart on pins 26 and 27
- On board LED. on pin 13
- RTC sensor: Real time clock. I2C (addr: 0x )
- SD card: Save data localy. SPI (custom pins)
- Heater: Drive the power to the heater. DAC on pin 25
- Pt100 sensor: Read the temperature of the pre-chamber. OneWire On pin 18
- RGB pm25 LED: RGB to show actual pm25 status.
- PMS: PM sensor. UART on pins 12 and 14.
- DHT sensor: Read the external temp and humedity of the node. on pin 19


The json strutures are:

RX-MSG: Json structure of msg to recive
{
  "id": "CIEP-05",
  "cmd": "LED",
  "arg1": 48.7560,
  "arg2": 48.75608
}


TX-MSG: Json structure of msg to send
{
  "id": "CIEP-05",
  "type": "data",
  "time": 1511118471,
  "pm25": 34,
  "tn": 25.5,
  "hn": 55.5,
  "ts": 45.2432,
  "hs":  45.5
}

TX-MSG: Json structure of msg to respond status
{
  "id"      = "CIEP-05";
  "time"    = 1511118471;
  "type"    = "resp";
  "state"   = "error";
}

TODOLIST:

- Probe pt100 sensor.           CHECK
- add temperature control.      
- make n_samples programmable.  CHECK
- probe RTC                     CHECK
- probe datalogger SD           CHECK
- add RGB LED feature           CHECK

*/


/*-----------------
    Includes
-----------------*/
#include <Arduino.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include <Wire.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <RTClib.h>
#include <FastLED.h>


/*----------------
    Defines
-----------------*/
//PIN DEFINITIONS
#define OB_LED_PIN        13 //ON BOARD LED PIN
#define RGB_LED_PIN       15 //STATUS LED
#define HEATER_PIN        25 //DAC OUTPUT PIN
#define PT100_SENSOR_PIN  18 //PRE CHAMBER TEMPERATURE SENSOR
#define DHT_SENSOR_PIN    19 //DHT

#define SCL_PIN   2 //34  // SPI SCL
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
#define BAUDRATE_DEBUG  115200
#define BAUDRATE_SIM    9600
#define BAUDRATE_PMS    9600
#define PMS5003ST   //Define the type of sensor  use #define PMS5003 FOR D AND C sensors
#define NODE_INT    //define the type of node if ext use #define NODE_EXT   

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800       // Modem is SIM800
#define TINY_GSM_RX_BUFFER    1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <PubSubClient.h>

//NODE DEFINITIONS
#define SENSOR_ID         "CIEP-19"// NODE ID
#define VESRION           "V0.0.2"    // ACTUAL CODE VERSION
#define RESPONSE_OK       "ok"        // RESPONSE TO INCOMMING CMD
#define RESPONSE_FAIL     "error"     // RESPONSE TO INCOMMING CMD
#define RESPONSE_READY    "Ready"     // RESPONSE TO INCOMMING CMD
#define RESPONSE_ACQ      "reading"   // RESPONSE TO INCOMMING CMD
#define RESPONSE_START    "start_ok"  // RESPONSE TO INCOMMING CMD
#define RESPONSE_STOP     "stop_ok"   // RESPONSE TO INCOMMING CMD
#define TYPE_DATA         "data"      // TYPE OF RESPONSE. DATA: DATA FROM SENSOR
#define TYPE_RESP         "resp"      // TYPE OF RESPONSE. RESP: RESPONSE TO INCOMMING CMD

#define TIMER_F_SCLK      10000       // TIMER SOURCE CLOCK. IN kHz
#define TIMER_DEFAULT_TS  10          // TIMER DEFAULT PERIOD. IN SECONDS 
#define N_SAMPLES         10          // NUMBER OF SAMPLES TO CONSIDERAR IN THE AVERAGE OF SAMPLES
#define HEATER_TRESHOLD   80          // SOFTWARE LIMIT FOR THE HEATER. OUTPUT RANGE : 0 .. 255
#define T_TRESHOLD        4.0         // 
#define H_TRESHOLD        80.0        // 
#define NUM_LEDS          1

#define DHTTYPE           DHT22

//GPRS sim credentials
const char apn[]      = "freeeway"; //"internet.emt.ee"; // APN from PRIMCARDS
const char gprsUser[] = "";         // GPRS User
const char gprsPass[] = "";         // GPRS Password

//MQTT definitions
const char* mqtt_server = "190.121.23.217"; //mqtt IP broker (string)
#define mqtt_port     1883                  //mqtt port (int)
#define MQTT_USER     ""                    //mqtt user
#define MQTT_PASSWORD ""                    //mqtt passwd
/*
#define MQTT_PUBLISH_CH   "pm/test1"        // CHANNEL TO PUBLISH
#define MQTT_RECEIVER_CH  "pm/test2"        // CHANNEL TO SUBSCRIBE (RECIVE COMMANDS)
*/
#define MQTT_PUBLISH_CH   "CIEP/tx"        // CHANNELS FOR UDD
#define MQTT_RECEIVER_CH  "CIEP/rx"        // CHANNELS FOR UDD
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

//SPI BUSES 
SPIClass spi;

//I2C BUSES 
TwoWire I2C_BUS = TwoWire(0);

//Client and mqtt
TinyGsm       modem(SerialSIM);
TinyGsmClient client(modem);
PubSubClient  mqtt(client);

//timers 
hw_timer_t * timer = NULL;    //timer to adquisition of sensors
hw_timer_t * timerhm = NULL;  //timer to control the heater

//DHT sensor
DHT dht(DHT_SENSOR_PIN,DHTTYPE);

//oneWire sensor
OneWire oneWire(PT100_SENSOR_PIN);
DallasTemperature pt100(&oneWire);

//RTC
RTC_DS3231 rtc;
DateTime now;

//RGB
CRGB rgb_led[NUM_LEDS];

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
int   timer_adqt    = TIMER_DEFAULT_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME. in seconds
int   timer_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timer_flag    = false;

//TIMER2 heater monitoring
int   timerhm_adqt    = TIMER_DEFAULT_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME
int   timerhm_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timerhm_flag    = false;

//data buffer 
uint8_t scount        = 0;            // COUNTER FOR THE SAMPLES
bool publish_flag   = false;        // flag for publish data
bool reading_state  = false;        // state of the readings

uint16_t pm25     = 0;
uint16_t pm25_sum = 0;
float    pm25_avg = 0;

//sensors vars
float t_node_sum = 0.0;
float h_node_sum = 0.0;
float t_node_avg = 0.0;
float h_node_avg = 0.0;
float t_node     = 0.0;
float h_node     = 0.0;

float t_pms_sum = 0.0;
float h_pms_sum = 0.0;
float t_pms_avg = 0.0;
float h_pms_avg = 0.0;
float t_pms     = 0.0;
float h_pms     = 0.0;

float t_heater = 0.0;

//LED
bool led_state    = false; // for test LED
bool led_s_state  = false; // for status LED

//RGB
int rgb_pos;

//HEATER 
uint8_t heater_value = 0;     // VARIABLE TO SAVE THE HEATER OUTPUT VALUE. RANGE : 0 TO 255
bool    heater_state = false; // VARIABLE TO TURN ON OFF THE HEATER.

//INCOMMING COMMAND VARIABLES
const char* rcv_id ;                    //RECIVED ID
const char* cmd;                        //RECIVED COMMAND
int param1 = 0;                         //RECIVED ARG 1
int param2 = 0;                         //RECIVED ARG 2
const char* response = RESPONSE_OK;     //RESPONSE TO INCOMMING CMD

int pm25_th_1 = 50;   // Threshold for the first pm25 alert interval
int pm25_th_2 = 80;   // Threshold for the second pm25 alert interval
int pm25_th_3 = 110;  // Threshold for the third pm25 alert interval
int pm25_th_4 = 170;  // Threshold for the fourth pm25 alert interval

int n_samples   = N_SAMPLES;              // store the actual number of samples to be average
const char* id  = SENSOR_ID;             // VAR TO store ID


//RTC
uint32_t lastTimeUnix = 0;

//SD
String dataMsg;

/*-------------------
  Functions headers
-------------------*/
bool setPowerBoostKeepOn(int en);
void setup_gsm();
void reconnect();
void mqttCallback(char* topic, byte *payload, unsigned int length);
void processCmd(byte* payload, unsigned int length);
void publishMqtt(char *serialData);
void publishStatus(const char *resp);
void publishSensor();
void writeDatalogger();
void readSensors();
bool readPMSdata(Stream *s);void rgbSet(uint16_t pm25_value, bool rgb_onoff);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void IRAM_ATTR timerISR();
void IRAM_ATTR timer_HM_ISR();


void setup()
{
  //Serial
  SerialDBG.begin(BAUDRATE_DEBUG);
  SerialPMS.begin(BAUDRATE_PMS, SERIAL_8N1, PMS_RX,PMS_TX);
  delay(15);

  //Set GSM module baud rate and UART pins
  SerialSIM.begin(9600, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);

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

  // set SD pinout
  pinMode(SCL_PIN,OUTPUT);
  pinMode(MOSI_PIN,OUTPUT);
  pinMode(MISO_PIN,INPUT);
  pinMode(CS_PIN,OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  //onboard LED
  pinMode(OB_LED_PIN, OUTPUT);
  digitalWrite(OB_LED_PIN, LOW);
  
  //RGB
  FastLED.addLeds<NEOPIXEL,RGB_LED_PIN>(rgb_led,NUM_LEDS);
  pinMode(RGB_LED_PIN, OUTPUT);

  //dac start on 0
  dacWrite(HEATER_PIN,0);

  //default dumy data sensors
  response = RESPONSE_OK;

  dht.begin();
  delay(250);

  pt100.begin();
  delay(250);

  if (! rtc.begin()) {
    SerialDBG.println("Couldn't find RTC");
    
  }

  
  spi = SPIClass(HSPI);
  spi.begin(SCL_PIN,MISO_PIN,MOSI_PIN,CS_PIN);
  
  if(!SD.begin(CS_PIN,spi,10000000))
  {
    SerialDBG.println("Card Mount Failed");
  
  }
  
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE)
  {
    SerialDBG.println("No SD card attached");
    
  }

  SerialDBG.println("Initializing SD card...");
  if (!SD.begin(CS_PIN))
  {
    SerialDBG.println("ERROR - SD card initialization failed!");
  }

  File file = SD.open("/data.txt");
  if(!file) 
  {
    SerialDBG.println("File doens't exist");
    SerialDBG.println("Creating file...");
    writeFile(SD, "/data.txt", "Reading ID, Date, Hour, Temperature \r\n");
  }
  else 
  {
    SerialDBG.println("File already exists");  
  }
  file.close();
  
  
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
  
  //setup Timer
  timer = timerBegin(3, 8000, true);    //timer 3, prescaler 8000, counting up
  timerAttachInterrupt(timer,&timerISR,true); //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
  timerAlarmWrite(timer, timer_counter,true);   // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
  timerAlarmEnable(timer);              // enable CTC mode
  //timerStop(timer);                     //stop timer

}


void loop()
{
  DateTime now = rtc.now();
  lastTimeUnix = now.unixtime();

  mqtt.loop();
  if(timer_flag)
  {
    SerialDBG.println("adquiring data");
    readSensors();     // read all sensors
    writeDatalogger(); // write raw data in the SD
    timer_flag = false;
  }
  if(publish_flag)
  {
    publishSensor();    //publish data
    publish_flag = false; 
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

  const size_t capacity_tx = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc_tx(capacity_tx);
  
  doc_tx["id"]      = id;
  doc_tx["time"]    = lastTimeUnix;
  doc_tx["type"]    = TYPE_RESP;
  doc_tx["state"]   = RESPONSE_READY;

  serializeJson(doc_tx, json_tx);
   
  while(!mqtt.connected())
  {
    SerialDBG.println("MQTT not connected");
    if (mqtt.connect(id))
    {
      SerialDBG.println("connected");
      mqtt.publish(MQTT_PUBLISH_CH, json_tx);
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

  if(strcmp(rcv_id,id)==0) // check if the message is for us
  {
    SerialDBG.println("ID OK");
    cmd     = doc_rx["cmd"];
    param1  = doc_rx["arg1"];
    param2  = doc_rx["arg2"];

    if(strcmp(cmd,"led") == 0)     // SET LED CMD: arg1 = led state, 
    {
      if(param1 == 1) 
      {
       
        digitalWrite(OB_LED_PIN,1);
        publishStatus(RESPONSE_OK);
      }
      else
      {
        digitalWrite(OB_LED_PIN,0);
        publishStatus(RESPONSE_OK);
      }
    }
    
    else if(strcmp(cmd,"heater_set") == 0) // SET HEATER value: arg1 = heater value
    {
      if( param1 > HEATER_TRESHOLD){ heater_value = HEATER_TRESHOLD; }
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
      scount = 0;
      reading_state = true;
      timerRestart(timer);
      publishStatus(RESPONSE_START); 
    }
    else if(strcmp(cmd,"stop") == 0)
    {
      timerStop(timer); 
      reading_state = false;
      rgbSet(0,0);
      publishStatus(RESPONSE_OK);
    }
    else if(strcmp(cmd,"samples") == 0)
    {
      timerStop(timer);
      timerAlarmDisable(timer);

      n_samples    = param1;
      
      timerAlarmEnable(timer);              
      timerRestart(timer);
      
      t_pms_sum   = 0.0;
      h_pms_sum   = 0.0;
      h_node_sum  = 0.0;  
      t_node_sum  = 0.0;  
      pm25_sum    = 0;
      scount      = 0;
            
      publishStatus(RESPONSE_OK);
    }
    else if(strcmp(cmd,"ID") == 0)
    {
      /* TODO*/
      publishStatus(RESPONSE_OK); 
    }
    else if(strcmp(cmd,"pmTH") == 0)
    {
      if(param1 == 1){pm25_th_1 = param2;}
      if(param1 == 2){pm25_th_2 = param2;}
      if(param1 == 3){pm25_th_3 = param2;}
      if(param1 == 4){pm25_th_2 = param2;}
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

void publishSensor()
{ 
  #ifdef PMS5003ST
    const size_t capacity_tx = JSON_OBJECT_SIZE(8);
  #else
    const size_t capacity_tx = JSON_OBJECT_SIZE(6);
  #endif
  DynamicJsonDocument doc_tx(capacity_tx);
  
  doc_tx["id"]      = id;
  doc_tx["type"]    = TYPE_DATA;
  doc_tx["time"]    = lastTimeUnix;
  
  pm25_avg = (float)pm25_sum / (float)n_samples;

  t_node_avg = t_node_sum / n_samples;
  h_node_avg = h_node_sum / n_samples;

  doc_tx["PM25"]  = pm25_avg;
  doc_tx["tn"]    = t_node_avg;
  doc_tx["hn"]    = h_node_avg;

  #ifdef PMS5003ST
    t_pms_avg = t_pms_sum / n_samples;
    h_pms_avg = h_pms_sum / n_samples;
    doc_tx["ts"] = t_pms_avg;
    doc_tx["hs"] = h_pms_avg;
  #endif
  
  serializeJson(doc_tx, json_tx);
  SerialDBG.println("msg to publish: ");
  SerialDBG.println(json_tx);
  publishMqtt(json_tx);

  t_pms_sum   = 0.0;
  h_pms_sum   = 0.0;
  h_node_sum  = 0.0;  
  t_node_sum  = 0.0;  
  pm25_sum    = 0;
}

void publishStatus(const char * resp)
{ 
  const size_t capacity_tx = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc_tx(capacity_tx);
  
  doc_tx["id"]      = id;
  doc_tx["time"]    = lastTimeUnix;
  doc_tx["type"]    = TYPE_RESP;
  doc_tx["state"]   = resp;

  serializeJson(doc_tx, json_tx);
  SerialDBG.println("msg to publish: ");
  SerialDBG.println(json_tx);
  publishMqtt(json_tx);
}

void publishMqtt(char *serialData)
{
  if(!mqtt.connected())
  {
    reconnect();
  }

  mqtt.publish(MQTT_PUBLISH_CH, serialData);
}

void writeDatalogger()
{
#ifdef PMS5003ST
  dataMsg = String(lastTimeUnix)+","+String(data.pm25_standard)+","+String(t_node)+","+String(h_node)+","+String(data.pm_t)+","+String(data.pm_h)+"\r\n";
#else
  dataMsg = String(lastTimeUnix)+","+String(data.pm25_standard)+","+String(t_node)+","+String(h_node)+","+String(0.00)+","+String(0.00)+"\r\n";
#endif
  SerialDBG.println(dataMsg);
    //appendFile(SD, "/data.txt", dataMsg.c_str());
}


void readSensors()
{

  while(!pms_data_ready)
  {
    pms_data_ready = readPMSdata(&SerialPMS);
  }

  if(pms_data_ready)
  {
    pms_count     += 1;
    pms_data_ready = false;
  }

  pm25      = data.pm25_standard;
  pm25_sum += pm25;

  h_node = dht.readHumidity();
  t_node = dht.readTemperature();

  if(isnan(h_node)){ h_node = 0.0;}
  if(isnan(t_node)){ t_node = 0.0;}

  h_node_sum  += h_node; 
  t_node_sum  += t_node; 

  #ifdef PMS5003ST
    h_pms_sum  += data.pm_h;
    t_pms_sum  += data.pm_t;  
  #endif  

  rgbSet(pm25,reading_state);
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


void rgbSet(uint16_t pm25_value, bool rgb_onoff)
{
  if(rgb_onoff)
  {
    if(pm25_value <= pm25_th_1)
    {
      rgb_led[0] = CRGB::Green;
    }
    else if(pm25_value > pm25_th_1 && pm25_value <= pm25_th_2 )
    {
      rgb_led[0] = CRGB::Yellow;
    }
    else if(pm25_value > pm25_th_2 && pm25_value <= pm25_th_3 )
    {
      rgb_led[0] = CRGB::OrangeRed;
    }
    else if(pm25_value > pm25_th_3 && pm25_value <= pm25_th_4 )
    {
      rgb_led[0] = CRGB::Red;
    }
    else if(pm25_value > pm25_th_4)
    {
      rgb_led[0] = CRGB::MediumPurple;
    }
  }
  else 
  {
      rgb_led[0] = CRGB::Black;
  }
  FastLED.show();
}

// Write to the SD card (DON'T MODIFY THIS FUNCTION)
void writeFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    Serial.println("File written");
  } else {
    Serial.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  Serial.printf("Appending to file: %s\n", path);

  File file = fs.open(path, FILE_APPEND);
  if(!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  if(file.print(message)) {
    Serial.println("Message appended");
  } else {
    Serial.println("Append failed");
  }
  file.close();
}


void IRAM_ATTR timerISR()  //Timer ISR
{
  scount++;
  timer_flag = true;
  if(scount >= n_samples)
  {
    publish_flag  = true;
    scount        = 0;
  }
}

void IRAM_ATTR timer_HM_ISR()  //Timer ISR
{
  //update sensor temperature
  //if temp > limit then stop

}
