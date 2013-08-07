/*
  GPS.h - Library for decoding GPS NMEA data.
  Created by Petey the Programmer 23-Sept-2009
*/

#ifndef GPS_h
#define GPS_h

#include "WProgram.h"

#define GPS_STR_SIZE 80

class GPS
{
  public:
    GPS(void);
	byte process_GPS_data(char theData);	
	char GPS_Valid();
	char New_Data();	
	float Altitude();
	float CMG();
	float Speed();
	float Lat();
	float Lon();
	unsigned long TimeStamp();
	
  private:
	uint8_t Valid_GPS_Data, New_GPS_Data;
	unsigned long gTimeStamp;
	float gLat, gLon;
	float gSpeed, gCMG, gAltitude;

	int glat1, glat2, glat3;
	int glon1, glon2, glon3;
	
	byte GPSStrLen;
	char GPSData[GPS_STR_SIZE];

	byte build_GPS_string(char c);
	int Validate_GPS_CheckSum(void);
	
	void decode_GPS_string();
	void Load_GPS_Data();

	void process_GPRMC_data();
	void process_GPGGA_data();

	uint8_t ScanField( uint8_t fNum, uint8_t *fStart, uint8_t *dPlace, uint8_t *fEnd );
	short GetWordValue(uint8_t cStart, uint8_t nChars);
	float GetFloatValue(uint8_t cStart, uint8_t dPlace, uint8_t cEnd);
};

#endif

