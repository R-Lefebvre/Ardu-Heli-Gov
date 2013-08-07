/* Arduino GPS-Gyro I2c Slave */

// Using Arduino Mini as I2c Salve to make GPS into I2c device and free up the serial port.
// All we have to do is read the serial port, decode the GPS strings, and deliver the data
// to the masterCPU  when asked.
// Version 9.6

#include <math.h>
#include <Wire.h>
#include <GPS.h>

#define NMEA_BAUD_RATE_4800    "$PSRF100,1,4800,8,1,0*0E\r\n"
#define NMEA_BAUD_RATE_9600    "$PSRF100,1,9600,8,1,0*0D\r\n"
#define NMEA_BAUD_RATE_19200   "$PSRF100,1,19200,8,1,0*38\r\n"

#define SIRF_BAUD_RATE_4800    "$PSRF100,0,4800,8,1,0*0F\r\n"
#define SIRF_BAUD_RATE_9600    "$PSRF100,0,9600,8,1,0*0C\r\n"
#define SIRF_BAUD_RATE_19200   "$PSRF100,0,19200,8,1,0*39\r\n"
#define SIRF_BAUD_RATE_57600   "$PSRF100,0,57600,8,1,0*37\r\n"

#define NMEA_WAAS_ON    "$PSRF151,1*3F\r\n"       // enable WAAS
#define NMEA_WAAS_OFF   "$PSRF151,0*3E\r\n"       // disable WAAS

// -----------------------------------------------------------

#define LED_pin 13

unsigned long my20ms_timer = 0;     // timers
unsigned long OneSecTimer = 0;
unsigned long GPSTimeStamp = 0;
unsigned long loopCounter = 0;

int adcCounter;
int gpsNoData = 0;

int analog0, analog1, gyroBias;

char RxBuffer[32];
#define MAX_BUF_INDEX 31

struct I2cBlock {
  int gyroRate;
  int GPSFlag;
  float gyroHeading;
  unsigned long TimeStamp;
  float Lat;
  float Lon;
  float Altitude;
  float CMG;
  float gyroCycle;
};

struct xI2cBlock {
  unsigned long TimeStamp;
  float Lat;
  float Lon;
  int gpsHeading;
  int gpsAltitude;  // 16 bytes

  int gpsSpeed;
  int gpsFlags;
  int gyroHeading;  
  int gyroCycle;    // 8 bytes

  int gyroRate;
  int imuRoll;
  int imuPitch;
  int imuRate;
};

union data_block_union {
  struct I2cBlock I2cData;
  uint8_t byteData[32];
};

data_block_union dataBlock, dataBlock2;

float fGyroCycle;
float fGyroHeading;

float prevLat, prevLon;
char DataIsLocked;

// -----------------------------------------------------------

GPS myGPS = GPS();    // Initialize a GPS structure

// -----------------------------------------------------------
// turn off extra messages about satilite usage and DOP info...

void turn_off_GSV_messages(void)
{
  Serial.print("$PSRF103,03,00,00,01*27\r\n");    // turn off GP GSV messages
  Serial.print("$PSRF103,02,00,00,01*26\r\n");    // turn off GP GSA messages
}

// -----------------------------------------------------------

void setup_GPS_for_NMEA_4800(void)
{
  delay(200);
  Serial.print(NMEA_BAUD_RATE_4800);
  delay(500);
  Serial.begin(4800);
}
// -----------------------------------------------------------

void setup_GPS_for_NMEA_9600(void)
{
  delay(200);
  Serial.print(NMEA_BAUD_RATE_9600);
  delay(500);
  Serial.begin(9600);
}
// -----------------------------------------------------------

void turn_GPS_WAAS_on(void)
{
  Serial.print(NMEA_WAAS_ON);
}

// -----------------------------------------------------------
// Switch from Binary SIRF back to ASCII based NMEA

void switch_to_NMEA(void)    // switch to NMEA text at 9600 baud.
{
  const byte start_seq[]={
    0xA0,0xA2,0x00,0x18};        // Start-Sequence and Payload length

  const byte gps_payload[]={
    0x81,0x02,0x01,0x01,0x00,0x01,0x01,0x01,0x05,0x01,0x01,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x00,0x01,0x25,0x80};

  const byte gps_checksum[]={  // Check-sum and end bytes.
    0x01,0x3A,0xB0,0xB3};

  int y;

   for(y=0; y<4; y++)
      Serial.print(byte(start_seq[y]));  
 
   for(y=0; y<24; y++)
      Serial.print(byte(gps_payload[y]));

   for(y=0; y<4; y++)
      Serial.print(byte(gps_checksum[y]));

}

/*************************************************************************
 * //Function to calculate the bearing between two locations
 *************************************************************************/
 
float calc_bearing(float lat1, float lon1, float lat2, float lon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (lat2 - lat1); 
  float y = 69.1 * (lon2 - lon1) * cos(lat1 / 57.3);
 
  calc = atan2(y,x);
  bear_calc = degrees(calc);

  if (bear_calc <= 0)
    bear_calc = 360.0 + bear_calc; 
  
  return bear_calc;
}

/*************************************************************************
 * //Function to calculate the distance between two locations
 *************************************************************************/

int calc_dist(float lat1, float lon1, float lat2, float lon2)
{
 float x = 69.1 * (lat2 - lat1); 
 float y = 69.1 * (lon2 - lon1) * cos(lat1 / 57.3);
 float theDist;
 
 theDist = sqrt((x*x)+(y*y)) * 1609.344; 

 if (theDist > 32000.0) return 32000; 
 return int(theDist); 
}

// -----------------------------------------------------------------------------------------------------------------------------
// We're about to change things within the dataBlock.
// First make a copy of it, then lock it out so the interrupt routines. 
// don't access the data while we're changing it in the fore-ground.

void lock_out_data()
{
  for (int n=0; n<32; n++)
    dataBlock2.byteData[n] = dataBlock.byteData[n];
  DataIsLocked = true;
}

// -----------------------------------------------------------------------------------------------------------------------------
// rate of gyro at full swing = 150 degrees per second
// this should be called every 20ms = 50 times per second
// dt is in milli-seconds

void update_gyro_heading(int dt)
{
  int gData;
  float gyroDelta;
  
  gData = (analog1 - analog0) - gyroBias;        // range is now -512 ... 0 ... 512
  gData = gData * -1;
  gyroDelta = (float)(gData) / 512.0;            // range is now -1.0 ... 0 ... 1.0
  gyroDelta = gyroDelta * 150.0 * (float)(dt / 1000.0);     
  
  fGyroHeading += gyroDelta;
  fGyroCycle += gyroDelta;
  
  if (fGyroHeading > 360.0) fGyroHeading -= 360.0;
  if (fGyroHeading < 0.0) fGyroHeading += 360.0;

  lock_out_data();
  dataBlock.I2cData.gyroHeading = fGyroHeading;
  dataBlock.I2cData.gyroCycle = fGyroCycle; 
  dataBlock.I2cData.gyroRate = gData;
  DataIsLocked = false;
}


// -----------------------------------------------------------------------------------------------------------------------------

void reset_xbee_module()
{
  pinMode( 6, OUTPUT );    // this is connected to XBee reset line.
  digitalWrite( 6, LOW );  // reset XBee by taking line low.
  delay(200);
  pinMode( 6, INPUT );    // go into input mode to allow line to go high
}

// -----------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------


void setup()
{
  pinMode(LED_pin, OUTPUT); 
  reset_xbee_module();

  Wire.begin(4);                // join i2c bus with address #4
  Wire.onReceive(master_has_sent_us_data);   // register event handlers
  Wire.onRequest(master_wants_data); 

  Serial.begin(9600);    // This is for talking to the GPS module.
    
  Serial.println("\r\n");    
  Serial.println("- ArduPilot I2c Slave GPS v9.6 - 9600 Baud - 32 Byte I2c Buffer");    

//  delay(800);
  
  analog0 = 512;
  analog1 = 486;
  gyroBias = -23;
  dataBlock.I2cData.GPSFlag = 1;  // No GPS-Fix yet
  dataBlock.I2cData.gyroHeading = 0.0;
  prevLat = 0;
  prevLon = 0;
  DataIsLocked = false;
  fGyroHeading = 0.0;
  fGyroCycle = 0.0;
  
//  calc_bias();
  
  OneSecTimer = millis();
}


// -----------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------------
// Not much happens here - sample analog pins - wait for I2c Interrupts,
// Check incomming Serial line for GPS data.

void loop()
{
  int theAltitude;
  unsigned long newTimeStamp, dt;
  byte dataFlags;
  float lat, lon, ourCMG;
  
  loopCounter++;

  analog0 = (analog0 + analogRead(0)) >> 1;    // sample Analog inputs every time thru the loop
  analog1 = (analog1 + analogRead(1)) >> 1;    // Rate is Analog(1)  vRef is Analog(0) 

  // ----- Handle GPS Data ------
  
  while (Serial.available() > 0)
    myGPS.process_GPS_data(Serial.read());
 
  if (dataFlags = myGPS.New_Data()) {    
      digitalWrite(LED_pin, HIGH);
      gpsNoData = 0;    // triggers if no GPS for 2 seconds

      lock_out_data();
      dataBlock.I2cData.GPSFlag = (dataFlags << 8) + 0;  // 0 = Good_Fix
      dataBlock.I2cData.TimeStamp = myGPS.TimeStamp();
      dataBlock.I2cData.Lat = myGPS.Lat();
      dataBlock.I2cData.Lon = myGPS.Lon();
      dataBlock.I2cData.Altitude = myGPS.Altitude();
      dataBlock.I2cData.CMG = myGPS.CMG();
      dataBlock.I2cData.gyroHeading = fGyroHeading;
	  
//      if (dataFlags & 0x01) 
//        fGyroHeading = myGPS.CMG();    // turn this off in v93, back on in v94
      
      newTimeStamp = myGPS.TimeStamp();
      if (newTimeStamp != GPSTimeStamp)
        {
          GPSTimeStamp = newTimeStamp;
          /*
          Serial.println("New GPS Data");
          Serial.println(myGPS.Lat());
          Serial.println(myGPS.Lon());
          Serial.println(myGPS.Altitude());
          Serial.println(myGPS.CMG());
          Serial.println();
          */    
      // Serial.print("Gyro: ");    
      // Serial.println((int)(analog1-analog0) - gyroBias);
        }

    lat = myGPS.Lat();
    lon = myGPS.Lon();
    if ( calc_dist(prevLat, prevLon, lat, lon) > 1)   // this will only happen once per sec when new data arrives.
		{
//		ourCMG = calc_bearing(prevLat, prevLon, lat, lon) + heading_roll_compensation();
		ourCMG = calc_bearing(prevLat, prevLon, lat, lon);
		if (ourCMG > 360.0) ourCMG -= 360.0;
		if (ourCMG < 0) ourCMG += 360.0;
		prevLat = lat;
		prevLon = lon;
		dataBlock.I2cData.gyroHeading = ourCMG;  // turn off in v94
		fGyroHeading = ourCMG;
		}

      DataIsLocked = false;  
  }
  
 // -------------------------------------------------
// Do all tasks that require 20ms timing.

 dt = (millis() - my20ms_timer);
 if (dt >= 20)
    {
      my20ms_timer = millis();     // sample Analog inputs every 20ms
      update_gyro_heading(dt);
    }
   
// -------------------------------------------------
// Do tasks that require 1 second timing.

 if ((millis() - OneSecTimer) >= 1000)
    { 
     OneSecTimer = millis();
      
      if (gpsNoData < 32) gpsNoData++;    // if no GPS data for 2 seconds, turn out the LED
      if (gpsNoData > 2) {
        digitalWrite(LED_pin, LOW);
        dataBlock.I2cData.GPSFlag = 1;  // 1 = No_Fix
      }
      
      adcCounter = 0;
      
//      Serial.print("Loop Counter: ");    // at 16 mHz, we're clocking 27-30K loops per second
//      Serial.println(loopCounter);

      loopCounter = 0;
    }
   
}

// -----------------------------------------------------------------------------------------------------------------------------

void calc_bias()
{
 int n,j;
 long accum=0;
 
  for (n=0; n<100; n++)
   {
    for (j=0; j<100; j++)
     {
      analog0 = (analog0 + analogRead(0)) >> 1;    // sample Analog inputs every time thru the loop
      analog1 = (analog1 + analogRead(1)) >> 1;    // Rate is Analog(1)  vRef is Analog(0) 
     } 
    accum += (analog1-analog0);
   } 
  gyroBias = accum / 100;
  
  Serial.print("Gyro bias: ");
  Serial.print(gyroBias);
  Serial.print("  Accum: ");
  Serial.println(accum);
  Serial.print("  Analog-1: ");
  Serial.println(analog1);
  Serial.print("  Analog-0: ");
  Serial.println(analog0);
}

// -----------------------------------------------------------------------------------------------------------------------------
// This routine executes whenever data is received from the master.

void master_has_sent_us_data(int howMany)
{
  byte slen=0;
  byte theData;
  
  //Serial.println((int)howMany);
  
  if (Wire.available() > 0)   
  {
    theData = Wire.receive();
    
    if (theData == 1) calc_bias();
    
    if (theData == 2) fGyroCycle = 0.0;  // Zero the Gyro Counter.  Measure total rotation.
  }

}

// -----------------------------------------------------------------------------------------------------------------------------
// This function executes when-ever data is requested by the master (every 20ms)
// Send back GPS and gyro data.

void master_wants_data()
{
  if (DataIsLocked)
     Wire.send((uint8_t*)(dataBlock2.byteData), 32);
  else
     Wire.send((uint8_t*)(dataBlock.byteData), 32);
}

// -----------------------------------------------------------------------------------------------------------------------------
// End of File
