// TESTING DATALOGGER
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <RTClib.h>
#include "FS.h"
#include "SD.h"

/*----------------
    Defines
-----------------*/
//PIN DEFINITIONS
#define OB_LED_PIN       13 //ON BOARD LED PIN
#define RGB_LED_PIN      15 //STATUS LED
#define HEATER_PIN          25 //DAC OUTPUT PIN
#define PT100_SENSOR_PIN    18 //PRE CHAMBER TEMPERATURE SENSOR
#define DHT_EXT_SENSOR_PIN  19 //DHT
#define DHT_NODE_SENSOR_PIN 2  //DHT

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
#define PMS5003ST   //Define the type of sensor  use #define PMS5003 FOR D AND C sensors
#define NODE_INT    //define the type of node if ext use #define NODE_EXT   //POWER TTGO-TCALL MODULE  DEFINITION 


#define IP5306_ADDR          0x75
#define IP5306_REG_SYS_CTL0  0x00


//I2C BUSES 
TwoWire I2C_BUS = TwoWire(0);

//RTC
RTC_DS3231 rtc;

//SD
String dataMsg;

//RTC
uint32_t lastTimeUnix = 0;
int  count = 0;

bool setPowerBoostKeepOn(int en);
void writeDatalogger();
void writeFile(fs::FS &fs, const char * path, const char * message);
void appendFile(fs::FS &fs, const char * path, const char * message);



void setup()
{
  //Serial
  SerialDBG.begin(BAUDRATE_DEBUG);
  SerialPMS.begin(BAUDRATE_PMS, SERIAL_8N1, PMS_RX,PMS_TX);
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

  // set SD pinout
  pinMode(SCL_PIN,OUTPUT);
  pinMode(MOSI_PIN,OUTPUT);
  pinMode(MISO_PIN,INPUT);
  pinMode(CS_PIN,OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  SPIClass spi = SPIClass(HSPI);
  spi.begin(SCL_PIN,MISO_PIN,MOSI_PIN,CS_PIN);
  //SD setUp
  //SD.begin(CS_PIN,spi,80000000);
  if(!SD.begin(CS_PIN,spi,80000000)) {
    SerialDBG.println("Card Mount Failed");
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
  if(!file) {
    SerialDGB.println("File doens't exist");
    SerialDGB.println("Creating file...");
    writeFile(SD, "/data.txt", "time, count \r\n");
  }
  else {
    SerialDBG.println("File already exists");  
  }
  file.close();
  
}

void loop()
{

  delay(60000);
  DataTime now = rtc.now();
  lastTimeUnix =  now.unixTime();
  count+=1;
  dataMsg = String(lastTimeUnix) + "," + String(count)+ "\r\n";
  appendFile(SD, "/data.txt", dataMsg.c_str());
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