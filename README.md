# CIEP_EFMMP
Code for project "Estaciones fijas de monitoreo de material particulado"
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

VERSION FEATURE
- fixing rtc time setup
- fixing on SD card per sampler


TODOLIST:
- add temperature control. CHECK
- add humidity to temperature control. 

LIST OF LIBRARIES USED :

bblanchon/ArduinoJson @ ^6.17.3
knolleary/PubSubClient @ ^2.8
vshymanskyy/TinyGSM @ ^0.10.9
fastled/FastLED @ 3.4.0    ;FastLED for controlling RGB leds.
adafruit/RTClib @ 1.12.4   ; RTClib for RTC module.
arduino-libraries/SD @ ^1.2.4 ; SD lib
adafruit/Adafruit Unified Sensor @ ^1.1.4
adafruit/DHT sensor library @ ^1.4.0
milesburton/DallasTemperature @ ^3.9.1
paulstoffregen/OneWire @ ^2.3.5