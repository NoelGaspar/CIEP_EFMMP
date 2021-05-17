/*


VESRION : V 0.2.1 b
AUTOR   : WAC@IOWLABS

http post by GPRS using json  on ESP32 board


LIST OF LIBRARIES USED :
-http client:
 ArduinoHttpClient
-GPRS:
 TinyGSM
- JSON
  ArduinoJson
- DHT sensor 
  DHT
- RTC module
  RTClib
- LEDs RGB
  FastLED
- Sensor 


*/


/*-----------------
    Includes
-----------------*/
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
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
#define OW_SENSOR_PIN     18 //PRE CHAMBER TEMPERATURE SENSOR
#define DHT_SENSOR_PIN    19 //DHT

//SPI PORT
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

// Configure TinyGSM library
#define TINY_GSM_MODEM_SIM800       // Modem is SIM800
#define TINY_GSM_RX_BUFFER    1024  // Set RX buffer to 1Kb

#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>

//NODE DEFINITIONS
#define SENSOR_ID           "CIEP-19"// NODE ID
#define VESRION             "V0.2.1"    // ACTUAL CODE VERSION
#define RESPONSE_OK         "ok"        // RESPONSE TO INCOMMING CMD
#define RESPONSE_FAIL       "error"     // RESPONSE TO INCOMMING CMD
#define RESPONSE_READY      "Ready"     // RESPONSE TO INCOMMING CMD
#define RESPONSE_ACQ        "reading"   // RESPONSE TO INCOMMING CMD
#define RESPONSE_START      "start_ok"  // RESPONSE TO INCOMMING CMD
#define RESPONSE_STOP       "stop_ok"   // RESPONSE TO INCOMMING CMD
#define TYPE_DATA           "data"      // TYPE OF RESPONSE. DATA: DATA FROM SENSOR
#define TYPE_RESP           "resp"      // TYPE OF RESPONSE. RESP: RESPONSE TO INCOMMING CMD
#define HEATER_MANUAL_MODE  0         // MANUAL MODE FOR HEATER
#define HEATER_AUTO_MODE    1         // AUTO MODE FOR HEATER
#define DHTTYPE             DHT22       // DEFINE DHT TYPE SENSOR
#define PMS5003ST                     // Define the type of sensor  use #define PMS5003 FOR D AND C sensors
#define WARMING_UP          1
#define CHILLING            0

#define TIMER_F_SCLK        10000     // TIMER SOURCE CLOCK. IN kHz
#define TIMER_DEFAULT_TS    60        // TIMER DEFAULT PERIOD. IN SECONDS
#define HEATER_UPDATE_TS    1         // TIMER DEFAULT PERIOD. IN SECONDS
#define N_SAMPLES           10        // NUMBER OF SAMPLES TO CONSIDERAR IN THE AVERAGE OF SAMPLES
#define NUM_LEDS            1         // NUMBER OF LEDS PER STRIP
#define HEATER_TRESHOLD     150       // SOFTWARE LIMIT FOR THE HEATER. OUTPUT RANGE : 0 .. 255
#define HUMIDITY_LIMIT      75.0      // HUMIDITY LIMIT FOR AUTO MODE
#define TEMP_LIMIT          27.0      // TEMPERATURE LIMIT FOR THE AUTO MODE
#define TEMP_DELTA          3         // DELTA VALUE FOR HYSTERESIS

//GPRS sim credentials
const char apn[]      = "freeeway"; //"internet.emt.ee"; // APN from PRIMCARDS
const char gprsUser[] = "";         // GPRS User
const char gprsPass[] = "";         // GPRS Password

//HTTP POST

//const char* servername  = "35.223.234.244";
//const int   port  = 1880;
const char* servername  = "190.121.23.217";
const int   port  = 80;

//POWER TTGO-TCALL MODULE  DEFINITION
#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

/*-----------------
    Instances
-----------------*/
//I2C BUSES
TwoWire I2C_BUS = TwoWire(0);

//SPI BUSES
SPIClass spi;

//Client and mqtt
TinyGsm       modem(SerialSIM);
TinyGsmClient client(modem);
HttpClient    http(client,servername,port);

//timers
hw_timer_t * timer = NULL;    //timer to adquisition of sensors
hw_timer_t * heater_timer = NULL;  //timer to control the heater

//DHT sensor
DHT dht(DHT_SENSOR_PIN,DHTTYPE);

//oneWire sensor
OneWire oneWire(OW_SENSOR_PIN);
DallasTemperature ow_sensor(&oneWire);

//RTC
RTC_DS3231 rtc;
DateTime now;

//RGB
CRGB rgb_led[NUM_LEDS];

/*----------------------------
    VARIABLES AND STRUCTURES
-----------------------------*/

//Structs to save sensor data
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
struct pms5003data data;

bool pms_data_ready = false ; // FLAG FOR PMS CORRECT ADQUISITION
long pms_count      = 0;      // COUNTER FOR CORRECT ADQUIRED DATA

//TIMER vars
int   timer_adqt    = TIMER_DEFAULT_TS;               //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME. in seconds
int   timer_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timer_flag    = false;

//TIMER2 heater monitoring
int   heater_timer_adqt    = HEATER_UPDATE_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME
int   heater_timer_counter = int(heater_timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  heater_timer_flag    = false;   // flag to update heater temperature reading


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
uint8_t heater_value = 0;     // VARIABLE TO store the heater output value. must be between 0 and 255
bool    heater_state = false; // VARIABLE TO TURN ON OFF THE HEATER.
bool    heater_mode  = HEATER_MANUAL_MODE; // 1  auto mode; 0 manual mode.
float   heater_temp  = 0.0;

//data buffer
uint8_t scount      = 0;            // COUNTER FOR THE SAMPLES
bool publish_flag   = false;        // flag for publish data
bool reading_state  = false;        // state of the readings

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
void setupGSM();
void httpPublish();
void writeDatalogger();
void readSensors();
bool readPMSdata(Stream *s);void rgbSet(uint16_t pm25_value, bool rgb_onoff);
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);
void IRAM_ATTR timerISR();
void IRAM_ATTR timerHeaterISR();


void setup()
{
  //Serial
  SerialDBG.begin(BAUDRATE_DEBUG);
  //Serial sensor PMS
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

  dht.begin();
  delay(250);

  ow_sensor.begin();
  delay(250);

  if (! rtc.begin())
  {
    SerialDBG.println("Couldn't find RTC");
  }
  else
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  spi = SPIClass(HSPI);
  spi.begin(SCL_PIN,MISO_PIN,MOSI_PIN,CS_PIN);

  if(!SD.begin(CS_PIN,spi,10000000))
  { SerialDBG.println("Card Mount Failed");}

  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE)
  { SerialDBG.println("No SD card attached");}

  SerialDBG.println("Initializing SD card...");
  if (!SD.begin(CS_PIN))
  { SerialDBG.println("ERROR - SD card initialization failed!"); }

  File file = SD.open("/data.txt");
  if(!file)
  {
    SerialDBG.println("File doens't exist");
    SerialDBG.println("Creating file...");
    writeFile(SD, "/data.txt", "ID, Date, Hour, Temperature \r\n");
  }
  else
  {
    SerialDBG.println("File already exists");
  }
  file.close();


  //GSM
  setupGSM();

  //setup Timer
  timer = timerBegin(3, 8000, true);          //timer 3, prescaler 8000, counting up
  timerAttachInterrupt(timer,&timerISR,true); //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
  timerAlarmWrite(timer, timer_counter,true); // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
  timerAlarmEnable(timer);                    // enable CTC mode

  //setup heater Timer
  heater_timer = timerBegin(2, 8000, true);                 //timer 2, prescaler 8000, counting up
  timerAttachInterrupt(heater_timer,&timerHeaterISR,true);  //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
  timerAlarmWrite(heater_timer, heater_timer_counter,true); // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
  timerAlarmEnable(heater_timer);                           // enable CTC mode
  timerStop(heater_timer);                                  //stop timer

}


void loop()
{
  DateTime now = rtc.now();
  lastTimeUnix = now.unixtime();

  if(timer_flag)
  {
    SerialDBG.println("Adquiring data");
    readSensors();     // read all sensors
    writeDatalogger(); // write raw data in the SD
    timer_flag = false;
  }
  if(publish_flag)
  {
    httpPublish();
    publish_flag = false;
  }
  if(heater_timer_flag)
  {
    ow_sensor.requestTemperatures();
    heater_temp = ow_sensor.getTempCByIndex(0);
    heater_timer_flag = false;
  }
  //automatic control for heater
  if(heater_mode == HEATER_AUTO_MODE) // automatic control of temperature
  {
    if(heater_temp >= TEMP_LIMIT)
    {
      dacWrite(HEATER_PIN,0);
      heater_state = CHILLING ;
    }
    else if((heater_temp < TEMP_LIMIT) && (heater_temp >= (TEMP_LIMIT - TEMP_DELTA)))
    {
      if(heater_state == WARMING_UP){ dacWrite(HEATER_PIN, heater_value); }
      else{ dacWrite(HEATER_PIN,0); }
    }
    else if( heater_temp < (TEMP_LIMIT - TEMP_DELTA) )
    {
      dacWrite(HEATER_PIN, heater_value);
      heater_state = WARMING_UP;
    }
  }
  else
  {
    dacWrite(HEATER_PIN, 0);
  }

}

/*-------------------
  Functions
-------------------*/
void setupGSM()
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

void httpPublish()
{
  const size_t capacity = JSON_OBJECT_SIZE(8);
  // Prepare JSON document
  DynamicJsonDocument doc(capacity);
  doc["id"]    = id;
  doc["type"]  = TYPE_DATA;
  doc["time"]  = lastTimeUnix;


  pm25_avg    = (float)pm25_sum / (float)n_samples;
  t_node_avg  = t_node_sum  / n_samples;
  h_node_avg  = h_node_sum  / n_samples;
  t_pms_avg   = t_pms_sum   / n_samples;
  h_pms_avg   = h_pms_sum   / n_samples;

  doc["PM25"]  = pm25_avg;
  doc["tn"]    = t_node_avg;
  doc["hn"]    = h_node_avg;
  doc["ts"]    = t_pms_avg;
  doc["hs"]    = h_pms_avg;

  // Serialize JSON document
  String json;
  serializeJson(doc, json);
  SerialDBG.println(json);
  //post by http
  http.post("/mqqt/http/post.php","text/plain", "data=hola mundo");
  //http protocol response
  int httpResponseCode = http.responseStatusCode();
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);
  // Free resources
  http.stop();
}

void writeDatalogger()
{
  dataMsg = String(SENSOR_ID) + String(lastTimeUnix)+","+String(data.pm25_standard)+","+String(t_node)+","+String(h_node)+","+String(data.pm_t)+","+String(data.pm_h)+"\r\n";

  SerialDBG.println(dataMsg);
  appendFile(SD, "/data.txt", dataMsg.c_str());
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

 
  h_pms_sum  += data.pm_h;
  t_pms_sum  += data.pm_t;

  rgbSet(pm25,reading_state);
}

bool readPMSdata(Stream *s)
{
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

void IRAM_ATTR timerHeaterISR()  //Timer ISR
{
  heater_timer_flag = 1;
}
