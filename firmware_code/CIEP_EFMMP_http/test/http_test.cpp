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





*/


/*-----------------
    Includes
-----------------*/
#include <Arduino.h>
#include <Wire.h>
//#include <HTTpClient.h>
#include <ArduinoJson.h>

/*----------------
    Defines
-----------------*/
//PIN DEFINITIONS
#define OB_LED_PIN        13 //ON BOARD LED PIN
#define RGB_LED_PIN       15 //STATUS LED
#define HEATER_PIN        25 //DAC OUTPUT PIN
#define PT100_SENSOR_PIN  18 //PRE CHAMBER TEMPERATURE SENSOR
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
#define TIMER_DEFAULT_TS    120        // TIMER DEFAULT PERIOD. IN SECONDS
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

//Client and mqtt
TinyGsm       modem(SerialSIM);
TinyGsmClient client(modem);
HttpClient    http(client,servername,port);


//timers
hw_timer_t * timer = NULL;    //timer to adquisition of sensors
hw_timer_t * heater_timer = NULL;  //timer to control the heater

/*----------------------------
    VARIABLES AND STRUCTURES
-----------------------------*/

//TIMER vars
int   timer_adqt    = TIMER_DEFAULT_TS;               //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME. in seconds
int   timer_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timer_flag    = false;

//TIMER2 heater monitoring
int   heater_timer_adqt    = HEATER_UPDATE_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME
int   heater_timer_counter = int(heater_timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  heater_timer_flag    = false;   // flag to update heater temperature reading

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
void IRAM_ATTR timerISR();
void IRAM_ATTR timerHeaterISR();


void setup()
{
  //Serial
  SerialDBG.begin(BAUDRATE_DEBUG);

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


  //default dumy data sensors
  response = RESPONSE_OK;

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

  if(timer_flag)
  {
    SerialDBG.println("posting info");
    httpPublish();
    timer_flag = false;
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
  doc["time"]  = millis();
  doc["PM25"]  = 31;
  doc["tn"]    = 25.5;
  doc["hn"]    = 55.5;
  doc["ts"]    = 26.6;
  doc["hs"]    = 56.5;

  // Serialize JSON document
  String json;

  serializeJson(doc, json);
  SerialDBG.println(json);

  //http.post("/update_sensor","application/json", "{\"json\":\"test\", \"numero\":1}");
  //http.post("/mqqt/http/post.php","application/json", "data="+json);
  http.post("/mqqt/http/post.php","text/plain", "data=hola mundo");
  //http.post("/trama","application/json", json);

  //http.beginRequest();
  //http.post("/update_sensor");
  //http.sendHeader("Content-Type", "application/json");
  //http.sendHeader("Content-Length", json.length());
  //http.sendHeader("X-Custom-Header", "custom-header-value");
  //http.beginBody();
  //http.print(json);
  //http.endRequest();


  int httpResponseCode = http.responseStatusCode();
  Serial.print("HTTP Response code: ");
  Serial.println(httpResponseCode);

  // Free resources
  http.stop();
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
