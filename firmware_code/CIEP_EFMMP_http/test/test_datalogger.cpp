/*

List of libs to install

  - DHT sensor library 1.4.1          Adafruit
  - RTClib             1.12.4         Adafruit
  - Adafruit Unified Sensors 1.1.4    Adafruit
  - ArduinoJson 6.15.1                Benito Blanchon

*/



// TESTING DATALOGGER
#include <Arduino.h>

#include <Wire.h>
#include <RTClib.h>
#include <DHT.h>



/*----------------
    Defines
-----------------*/
//PIN DEFINITIONS
#define OB_LED_PIN       13 //ON BOARD LED PIN
#define RGB_LED_PIN      15 //STATUS LED
#define HEATER_PIN       25 //DAC OUTPUT PIN
#define PT100_SENSOR_PIN 18 //PRE CHAMBER TEMPERATURE SENSOR
#define DHT_SENSOR_PIN   19 //DHT


#define SCL_PIN   2   //34  // SPI SCL
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
#define NODE_INT    //define the type of node if ext use #define NODE_EXT   //POWER TTGO-TCALL MODULE  DEFINITION 


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00

#define SENSOR_ID "CIEP-01"

#define TIMER_F_SCLK      10000       // TIMER SOURCE CLOCK. IN kHz
#define TIMER_DEFAULT_TS  10          // TIMER DEFAULT PERIOD. IN SECONDS 
#define N_SAMPLES         10          // NUMBER OF SAMPLES TO CONSIDERAR IN THE AVERAGE OF SAMPLES
#define HEATER_TRESHOLD   80          // SOFTWARE LIMIT FOR THE HEATER. OUTPUT RANGE : 0 .. 255
#define T_TRESHOLD        4.0         // 
#define H_TRESHOLD        80.0        // 

#define NUM_LEDS          1

#define DHTTYPE           DHT22

SPIClass spi;

//I2C BUSES 
TwoWire I2C_BUS = TwoWire(0);

DHT dht(DHT_SENSOR_PIN,DHTTYPE);

//RTC
RTC_DS3231 rtc;

//TIMER
hw_timer_t * timer = NULL;    //timer to adquisition of sensors

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

uint16_t pm25     = 0;
bool pms_data_ready = false ; // FLAG FOR PMS CORRECT ADQUISITION
long pms_count      = 0;      // COUNTER FOR CORRECT ADQUIRED DATA

//TIMER vars
int   timer_adqt    = TIMER_DEFAULT_TS;             //VARIABLE TO SET THE TIMER PERIOD. PERIOD = ADQUISITION TIME. in seconds
int   timer_counter = int(timer_adqt * TIMER_F_SCLK); //VARIABLE TO SET THE MAX COUNT OF THE TIMER COUNTER
bool  timer_flag    = false;


String dataMsg;

//RTC
uint32_t lastTimeUnix = 0;
int  count = 0;

uint8_t SD_trys = 0;

float t_node = 0.0;
float h_node = 0.0;


void writeDatalogger();
void readSensors();
bool readPMSdata(Stream *s);
void writeFile(fs::FS &fs, const char * path, const char * message) ;
void appendFile(fs::FS &fs, const char * path, const char * message);
void IRAM_ATTR timerISR();

void setup()
{
  //UART
  SerialDBG.begin(BAUDRATE_DEBUG);
  SerialPMS.begin(BAUDRATE_PMS, SERIAL_8N1, PMS_RX,PMS_TX);
  delay(15);

  //I2C
  I2C_BUS.begin(MODEM_SDA,MODEM_SCL,400000);
  
  //SPI
  pinMode(SCL_PIN,OUTPUT);
  pinMode(MOSI_PIN,OUTPUT);
  pinMode(MISO_PIN,INPUT);
  pinMode(CS_PIN,OUTPUT);
  digitalWrite(CS_PIN, HIGH);


  if (! rtc.begin())
  {
    SerialDBG.println("Couldn't find RTC");
  }
  else
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  dht.begin();
  delay(250);

  spi = SPIClass(VSPI);
  spi.begin(SCL_PIN,MISO_PIN,MOSI_PIN,CS_PIN);
  
  if(!SD.begin(CS_PIN,spi,10000000))
  {
      SerialDBG.println("Card mount failed");
      return;
  }
  
  uint8_t cardType = SD.cardType();
  if(cardType == CARD_NONE) {
    SerialDBG.println("No SD card attached");
    return;
  }

  SerialDBG.println("Initializing SD card...");
  if (!SD.begin(CS_PIN)) {
    SerialDBG.println("ERROR - SD card initialization failed!");
    return;    // init failed
  }

  File file = SD.open("/data.txt");
  if(!file)
  {
    SerialDBG.println("File doens't exist");
    SerialDBG.println("Creating file...");
    writeFile(SD, "/data.txt", "id, time, pm25, ts, hs, tn, hn \r\n");
  }
  else 
  {
    SerialDBG.println("File already exists");  
  }
  
  file.close();


  //TIMER
  timer = timerBegin(3, 8000, true);    //timer 3, prescaler 8000, counting up
  timerAttachInterrupt(timer,&timerISR,true); //params: timer object, pointer to ISR function anddress, mode edge (if false: level mode)
  timerAlarmWrite(timer, timer_counter,true);   // params: timer object, counter_limit, restart counter on top.// to get T= 1s, n=T*f_{timer source clock}
  timerAlarmEnable(timer);              // enable CTC mode
  timerStop(timer);                     //stop timer
  timerRestart(timer);  //start the timer

}

void loop()
{
  DateTime now = rtc.now();
  lastTimeUnix = now.unixtime();
  
   if(timer_flag)
  {
    SerialDBG.println("adquiring data");
    readSensors();     // read all sensors
    writeDatalogger(); // write raw data in the SD
    timer_flag = false;
  }
}


void readSensors()
{
  
  while(!pms_data_ready)
  {
    pms_data_ready = readPMSdata(&SerialPMS);
  }

  if(pms_data_ready)
  {
    pms_data_ready = false;
  }

  pm25      = data.pm25_standard;
  
  h_node = dht.readHumidity();
  t_node = dht.readTemperature();

  if(isnan(h_node)){ h_node = 0.0;}
  if(isnan(t_node)){ t_node = 0.0;}

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

void writeDatalogger()
{
    dataMsg = String(SENSOR_ID)+","+String(lastTimeUnix)+","+String(data.pm25_standard)+","+String(data.pm_t)+","+String(data.pm_h)+","+String(t_node)+","+String(h_node)+"\r\n";
    //SerialDBG.println(dataMsg);
    appendFile(SD, "/data.txt", dataMsg.c_str());
    dataMsg = "";
}

void writeFile(fs::FS &fs, const char * path, const char * message) {
  SerialDBG.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if(!file) {
    SerialDBG.println("Failed to open file for writing");
    return;
  }
  if(file.print(message)) {
    SerialDBG.println("File written");
  } else {
    SerialDBG.println("Write failed");
  }
  file.close();
}

// Append data to the SD card (DON'T MODIFY THIS FUNCTION)
void appendFile(fs::FS &fs, const char * path, const char * message) {
  
  
  
  File datafile = fs.open(path, FILE_APPEND);
  if(!datafile) {
    SerialDBG.println("Failed to open file for appending");
    return;
  }
  
  if(datafile.print(message)) {
  
  } else {
  
  }
  
  datafile.close();
}

void IRAM_ATTR timerISR()  //Timer ISR
{
  timer_flag = true;
}