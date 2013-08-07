// --------------------------------------------------------------------------------------------------
// Structure for moving data between GPS I2c slave and master CPU

struct I2cBlock {
  int gyroRate;             // current rate: -512..0..512
  int GPSFlag;              // has two bytes of flags
  float gyroHeading;        // Our CMG updated by Gyro
  unsigned long TimeStamp;  // GPS time stamp
  float Lat;
  float Lon;
  float Altitude;
  float CMG;
  float gyroCycle;
};

// --------------------------------------------------------------------------------------------------
// Get GPS data from the I2c Slave

void decode_gps(void)
{
  unsigned long newTimeStamp;
  float prevAltitude;

  union data_block_union {
    struct I2cBlock I2cData;
    uint8_t byteData[32];
  } dataBlock;

  if ((millis() - GPS_timer) < 200) return;    // only do this 5 times a second
  GPS_timer = millis();  
  gpsFix = NO_FIX;
  prevAltitude = alt_MSL;
  
  Wire.requestFrom(4, 32);    // request 32 bytes from slave device #4
  if (Wire.available() == 32)
    {
      for (int n=0; n<32; n++) 
          dataBlock.byteData[n] = Wire.receive();
      
      gyroRate = dataBlock.I2cData.gyroRate;     
      gyroHeading = dataBlock.I2cData.gyroHeading;
      
      lat = dataBlock.I2cData.Lat;
      lon = dataBlock.I2cData.Lon;
      
      ground_course = dataBlock.I2cData.CMG;
      //ground_speed = dataBlock.I2cData.Speed;
      ground_speed = 0;
      
      newTimeStamp = dataBlock.I2cData.TimeStamp;      
      alt_MSL = dataBlock.I2cData.Altitude;
      theGyroCycle = int(dataBlock.I2cData.gyroCycle);
      
      if (highByte(dataBlock.I2cData.GPSFlag) & 0x02)
         climb_rate = alt_MSL - prevAltitude;

      if (lowByte(dataBlock.I2cData.GPSFlag) == 0) 
        gpsFix = GOOD_FIX;
      else
        gpsFix = NO_FIX;
		
      data_update_event = 0x03;	   // calc new set points 5 times/sec
	  
      if (newTimeStamp > GPSTimeStamp)
        {
         GPSTimeStamp = newTimeStamp;  // once per second
         //data_update_event = 0x03;	   // calc new set points
         //data_update_event |= 0x01;  // Lat/lon Data
         //data_update_event |= 0x02;  // altitude data
      // IMU_Heading = Avg_Headings( IMU_Heading, ground_course );  
        }
    }

  if (gpsFix == GOOD_FIX) 
      digitalWrite(BLUE_LED, HIGH);
  else
      digitalWrite(BLUE_LED, LOW);
}

// --------------------------------------------------------------------------------------------------

void Wait_GPS_Fix(void) // Wait for good GPS fix...
{
  do
  {
    catch_analogs();
    decode_gps();
    
    digitalWrite(BLUE_LED,HIGH);
    delay(25);
    digitalWrite(BLUE_LED,LOW);
    delay(25);
  }
  while (gpsFix == NO_FIX);
}


/****************************************************************
 ****************************************************************/
long Get_Relative_Altitude()
{
  long relative = alt_MSL - launch_alt;
  if (relative < 0) relative=0;  
  return relative;
}

/****************************************************************
Function that will read and store the current altitude when you switch to autopilot mode.
 ****************************************************************/
 
int Hold_Current_Altitude(void) 
{
  if ((AP_Mode == MANUAL_MODE) || (AP_Mode == FLY_BY_WIRE))  // Keep updating current alt while in manual mode
     {
      hold_Alt = Get_Relative_Altitude();
     }  // when not in Manual mode, return the last stored value
     
   return hold_Alt;
}

// -------------------------------------------------------------------------

void update_distance(void)
{
 wp_distance = calc_dist(lat, lon, wp_current_lat, wp_current_lon); 
 //wp_distance = calc_dist(lat,lon,33.954308,-117.504032);
 if (wp_distance < closest_to_wp) closest_to_wp = wp_distance;
}

/*************************************************************************
 * //Function to calculate the course between two waypoints
 *************************************************************************/
int calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
  calc=atan2(y,x);
  bear_calc= degrees(calc);

  if (bear_calc <= 0)
    bear_calc = 360.0 + bear_calc; 
  
  return int(bear_calc);
}
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 *************************************************************************/

int calc_dist(float flat1, float flon1, float flat2, float flon2)
{
 float x = 69.1 * (flat2 - flat1); 
 float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 float theDist;
 
 theDist = sqrt((x*x)+(y*y)) * 1609.344; 

 if (theDist > 32000.0) return 32000; 
 return int(theDist); 
}

// --------------------------------------------------------------------------------------------------
// Tell the Slave GPS-Gyro module to zero the gyroCycle.
// We can then measure how far we've rotated since we zero'd the counter.

void zero_gyro_cycle()
{
  byte theData = 2;
  Wire.beginTransmission(4);     // transmit to I2c device #4
  Wire.send(theData);
  Wire.endTransmission();
  theGyroCycle = 0;
}

// --------------------------------------------------------------------------------------------------
// Tell the Slave GPS-Gyro module to calc the gyro bias.
// We can then measure how far we've rotated since we zero'd the counter.

void reset_gyro_bias()
{
  byte theData = 1;
  Wire.beginTransmission(4);     // transmit to I2c device #4
  Wire.send(theData);
  Wire.endTransmission();
}

// --------------------------------------------------------------------------------------------------
// End of File
