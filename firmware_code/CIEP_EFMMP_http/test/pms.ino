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

bool readPMSdata(Stream *s);

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  while(!pms_data_ready)
  {
    pms_data_ready = readPMSdata(&Serial);
  }
  delay(500);  
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
  // Now read all 40 bytes
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