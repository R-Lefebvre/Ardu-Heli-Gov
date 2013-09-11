// ****************************************************************
// FrSky telemetry
// Changes: April 2013 by QuadBow - works with 2.2
//                                - compatible with display FLD-02
//                                - improved speed (to be sent in knots)
//                                - optimized scheduler
//                                - counts arming time
//                                - displays numbers of satellites as fuel bar
//                                - instead of altitude the distance to home can be displayed
//                                - integration of VBAT and POWERMETER
// Date 20/09/2012
// Changes: V0.2.1: - make it work with 2.1 (shared dev)
// Date: 14/08/2012
// Changes: V0.2: - Byte stuffing added
//                - vBat will be send, if "#define FAS_100" is comment out
//                V0.1: - First release
// ****************************************************************

#if defined(TELEMETRY_FRSKY)
  // Frame protocol
  #define Protocol_Header    0x5E
  #define Protocol_Tail      0x5E

  // Data Ids  (bp = before point; af = after point)
  // Official data IDs
  #define ID_GPS_Altitude_bp    0x01
  #define ID_GPS_Altitude_ap    0x09
  #define ID_Temperature1       0x02
  #define ID_RPM                0x03
  #define ID_Fuel_level         0x04
  #define ID_Temperature2       0x05
  #define ID_Volt               0x06
  #define ID_Altitude           0x10
  #define ID_GPS_speed_bp       0x11
  #define ID_GPS_speed_ap       0x19
  #define ID_Longitude_bp       0x12
  #define ID_Longitude_ap       0x1A
  #define ID_E_W                0x22
  #define ID_Latitude_bp        0x13
  #define ID_Latitude_ap        0x1B
  #define ID_N_S                0x23
  #define ID_Course_bp          0x14
  #define ID_Course_ap          0x1C
  #define ID_Date_Month         0x15
  #define ID_Year               0x16
  #define ID_Hour_Minute        0x17
  #define ID_Second             0x18
  #define ID_Acc_X              0x24
  #define ID_Acc_Y              0x25
  #define ID_Acc_Z              0x26
  #define ID_Voltage_Amp_bp     0x3A
  #define ID_Voltage_Amp_ap     0x3B
  #define ID_Current            0x28
  // User defined data IDs
  #define ID_Gyro_X             0x40
  #define ID_Gyro_Y             0x41
  #define ID_Gyro_Z             0x42

   // Main function FrSky telemetry
    void inline telemetry_frsky()
   {         
      static uint32_t lastTime;
      static uint8_t tele_loop;
      if ((millis() - lastTime) > 250) {
         // Data sent every 250ms
         lastTime = millis();
         tele_loop++;         
         send_Voltage_ampere();
         send_Accel();
         // Data sent every 1s
         switch (tele_loop) {
            case 1:
               send_Num_Sat();
               send_GPS_longitude();
            break;
            case 2:
               send_Altitude();
               send_RPM();
               send_GPS_speed();
            break;
            case 3:
               send_Num_Sat();
               send_GPS_latitude();
            break;
            case 4:
               send_Temperature();
               send_RPM();
               send_Time();
               tele_loop = 0;
            break;
            default:
            break;
            }
        sendDataTail();         
        }
   }

   void inline write_FrSky8(uint8_t Data)
   {
      SerialWrite(TELEMETRY_FRSKY_SERIAL, Data);
   }

   void inline write_FrSky16(uint16_t Data)
   {
      uint8_t Data_send;
      Data_send = Data;      
      check_FrSky_stuffing(Data_send);
      Data_send = Data >> 8 & 0xff;
      check_FrSky_stuffing(Data_send);
   }
   
   void inline check_FrSky_stuffing(uint8_t Data) //byte stuffing
   {
      if (Data == 0x5E)   
      {
         write_FrSky8(0x5D);
         write_FrSky8(0x3E);
      }
      else if (Data == 0x5D)   
      {
         write_FrSky8(0x5D);
         write_FrSky8(0x3D);
      }
      else
      {
         write_FrSky8(Data);         
      }
   }

   static void inline sendDataHead(uint8_t Data_id)
   {
      write_FrSky8(Protocol_Header);
      write_FrSky8(Data_id);
   }

   static void inline sendDataTail(void)
   {
      write_FrSky8(Protocol_Tail);      
   }

   //*********************************************************************************
   //-----------------   Telemetrie Data   ------------------------------------------   
   //*********************************************************************************

   // GPS altitude
   void inline send_GPS_altitude(void)
   {         
      if (f.GPS_FIX && GPS_numSat >= 4)
      {
         int16_t Data_GPS_altitude_bp;
         uint16_t Data_GPS_altitude_ap;

         Data_GPS_altitude_bp = GPS_altitude;
         Data_GPS_altitude_ap = 0;

         sendDataHead(ID_GPS_Altitude_bp);
         write_FrSky16(Data_GPS_altitude_bp);
         sendDataHead(ID_GPS_Altitude_ap);
         write_FrSky16(Data_GPS_altitude_ap);
      }
   }
   
   // Temperature
   void inline send_Temperature(void)
   {
      int16_t Data_Temperature1;
      int16_t Data_Temperature2;

      Data_Temperature1 = baroTemperature / 100;
      Data_Temperature2 = 0;
      sendDataHead(ID_Temperature1);
      write_FrSky16(Data_Temperature1);
      sendDataHead(ID_Temperature2);
      write_FrSky16(Data_Temperature2);
   }

   // RPM
   void inline send_RPM(void)
   {
      uint16_t Data_RPM = 0;
     
      Data_RPM = rpm;

      sendDataHead(ID_RPM);
      write_FrSky16(Data_RPM);
   }

   // Fuel level
   void inline send_Num_Sat(void)
   {
      uint16_t Data_Num_Sat;

         Data_Num_Sat = (GPS_numSat / 2) * 25;

      sendDataHead(ID_Fuel_level);
      write_FrSky16(Data_Num_Sat);
   }

   // Temperature 2
   void inline send_Distance(void)
   {
      if (f.GPS_FIX_HOME)
      {
         int16_t Data_Distance;

         Data_Distance = GPS_distanceToHome; // Distance to home alias Temp2

         sendDataHead(ID_Altitude);
         write_FrSky16(Data_Distance); 
      }     
   }

   // Cell voltage  todo !!!!!!!!!!!!!!!!!!
   void inline send_Cell_volt(void) // Data FrSky FLVS-01 voltage sensor
   {
      uint16_t Data_Volt;
      uint8_t number_of_cells = 0;   // LiPo 3S = 3; LiPo 4S = 4 ...
      static uint8_t cell = 0;

      if (cell >= number_of_cells)
         cell = 0;
      Data_Volt = 0; // 0.01v / 0 ~ 4.2v

      sendDataHead(ID_Volt);
      write_FrSky16(Data_Volt);
   }

   // Altitude
   void inline send_Altitude(void)
   {
      int16_t Data_altitude;

      #if defined BARO
        Data_altitude = EstAlt / 100; // - Start_altitude;
      #endif
      
      sendDataHead(ID_Altitude);
      write_FrSky16(Data_altitude);
   }

   // GPS speed
   void inline send_GPS_speed(void)
   {
      uint16_t Data_GPS_speed_bp;
      uint16_t Data_GPS_speed_ap;
      uint16_t temp;

      if (f.GPS_FIX && GPS_numSat >= 4)
      {           
         temp = (GPS_speed * 40) / 203;
         Data_GPS_speed_bp = temp / 10;
         Data_GPS_speed_ap = temp - Data_GPS_speed_bp * 10;
     
         sendDataHead(ID_GPS_speed_bp);
         write_FrSky16(Data_GPS_speed_bp);
         sendDataHead(ID_GPS_speed_ap);
         write_FrSky16(Data_GPS_speed_ap);
      }
   }

   // GPS position
 void inline send_GPS_longitude(void)
      {
      uint16_t Data_Longitude_bp;
      uint16_t Data_Longitude_ap;
      uint16_t Data_E_W;
      uint32_t temp, rest, grad;
         
      if (f.GPS_FIX && GPS_numSat >= 4)
         {           
         temp = abs(GPS_coord[LON]);
         grad = temp / 10000000;
         temp -= grad * 10000000;
         temp *= 6;
         rest = temp;
         temp /= 1000000;
         rest -= temp * 1000000;
         Data_Longitude_bp = grad * 100 + temp;
         Data_Longitude_ap = rest / 100;
         Data_E_W = GPS_coord[LON] < 0 ? 'W' : 'E';

         sendDataHead(ID_Longitude_bp);
         write_FrSky16(Data_Longitude_bp);
         sendDataHead(ID_Longitude_ap);
         write_FrSky16(Data_Longitude_ap);
         sendDataHead(ID_E_W);
         write_FrSky16(Data_E_W);
         }
      }

 void inline send_GPS_latitude(void)
      {
      uint16_t Data_Latitude_bp;
      uint16_t Data_Latitude_ap;
      uint16_t Data_N_S;
      uint32_t temp, rest, grad;

      if (f.GPS_FIX && GPS_numSat >= 4)
         {           
         temp = abs(GPS_coord[LAT]);
         grad = temp / 10000000;
         temp -= grad * 10000000;
         temp *= 6;
         rest = temp;
         temp /= 1000000;
         rest -= temp * 1000000;
         Data_Latitude_bp = grad * 100 + temp;
         Data_Latitude_ap = rest / 100;
         Data_N_S = GPS_coord[LAT] < 0 ? 'S' : 'N';

         sendDataHead(ID_Latitude_bp);
         write_FrSky16(Data_Latitude_bp);
         sendDataHead(ID_Latitude_ap);
         write_FrSky16(Data_Latitude_ap);
         sendDataHead(ID_N_S);
         write_FrSky16(Data_N_S);     
         }
   }

   // Course
   void inline send_Course(void)
   {
      uint16_t Data_Course_bp;
      uint16_t Data_Course_ap;

      Data_Course_bp = heading;
      Data_Course_ap = 0;

      sendDataHead(ID_Course_bp);
      write_FrSky16(Data_Course_bp);
      sendDataHead(ID_Course_ap);
      write_FrSky16(Data_Course_ap);
   }

   // Time
   void inline send_Time(void)
   {
      uint16_t seconds_since_start;
      uint16_t Data_Minutes_hours;
      uint16_t Data_seconds;

      if (showTime.TimerStart) {
         seconds_since_start = (millis() - showTime.armingTime) / 1000;
         Data_Minutes_hours = seconds_since_start / 60;
         Data_seconds = seconds_since_start - 60 * Data_Minutes_hours;     
         sendDataHead(ID_Hour_Minute);
         write_FrSky16(Data_Minutes_hours * 256);
         sendDataHead(ID_Second);
         write_FrSky16(Data_seconds);
         }
   }

   // ACC
   void inline send_Accel(void)
   {
      int16_t Data_Acc_X;
      int16_t Data_Acc_Y;
      int16_t Data_Acc_Z;

      Data_Acc_X = ((float)accSmooth[0] / acc_1G) * 1000;
      Data_Acc_Y = ((float)accSmooth[1] / acc_1G) * 1000;
      Data_Acc_Z = ((float)accSmooth[2] / acc_1G) * 1000;

      sendDataHead(ID_Acc_X);
      write_FrSky16(Data_Acc_X);
      sendDataHead(ID_Acc_Y);
      write_FrSky16(Data_Acc_Y);
      sendDataHead(ID_Acc_Z);
      write_FrSky16(Data_Acc_Z);     
   }

   // Voltage (Ampere Sensor) 
   void inline send_Voltage_ampere(void)
   {
      uint16_t Data_Voltage_vBat_bp;
      uint16_t Data_Voltage_vBat_ap;   
      uint16_t Data_Voltage_I_Motor;

      Data_Voltage_vBat_bp = vbat / 10;         
      Data_Voltage_vBat_ap = vbat - Data_Voltage_vBat_bp * 10;         
      #if defined(POWERMETER)
        Data_Voltage_I_Motor = pCurrent / 10;
      #else
        Data_Voltage_I_Motor = 0;
      #endif
      sendDataHead(ID_Voltage_Amp_bp);
      write_FrSky16(Data_Voltage_vBat_bp);
      sendDataHead(ID_Voltage_Amp_ap);
      write_FrSky16(Data_Voltage_vBat_ap);   
      sendDataHead(ID_Current);
      write_FrSky16(Data_Voltage_I_Motor);   
   }

#endif 

