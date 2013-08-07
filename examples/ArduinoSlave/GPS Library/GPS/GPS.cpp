// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// GPS.cpp - Utility routines for scanning GPS data strings
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

#include "WProgram.h"
#include "GPS.h"

#define TRUE	1
#define FALSE	0

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

float Load_Lat_Lon(char NegValue, short Degs, short Minutes, short mFract);
unsigned long strTolong(char *s);

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

GPS::GPS(void)
{
	Valid_GPS_Data = 0;
	New_GPS_Data = 0;
	GPSStrLen = 0;
	gAltitude = 0.0;
	gLat = 0.0;
	gLon = 0.0;
	gSpeed = 0.0;
	gCMG = 0.0;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Called from Main Loop - 
// Process chars - build up GPS strings.
// Host sends us the char from the serial port...

byte GPS::process_GPS_data(char theData)
{
	if (build_GPS_string(theData)) {	// When we have a complete NMEA string,
		decode_GPS_string();	        // process the GPS data.
		return 1;
		}
	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// process incoming chars - NMEA commands start with '$' and end with '\r'
// return 1 if GPS string is complete - else return zero

byte GPS::build_GPS_string(char c)
{
	if (c == '$') {			// this will catch re-starts and stalls as well as valid commands.
		GPSStrLen = 0;
		GPSData[GPSStrLen++] = c;
		return 0;
		}
	
	if (GPSStrLen != 0)		// string has already started
		{
		if (GPSStrLen < GPS_STR_SIZE) 
			GPSData[GPSStrLen++] = c;
		return (c == 0x0D);
		}
		
	return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// we have a complete GPS string - extract the current Lat/Lon data.

// $GPGGA,230106.000,3657.4733,N,12201.7087,W,1,04,2.2,31.6,M,-26.7,M,,0000*5A
// $GPGGA,230449.000,3657.4767,N,12201.7079,W,1,06,1.7,13.1,M,-26.7,M,,0000*57


void GPS::decode_GPS_string(void)
{
	uint8_t Unknown_Msg = 1;
	
	// scan the string - compare to check-sum on end
	Valid_GPS_Data = Validate_GPS_CheckSum();
	if (Valid_GPS_Data == 0) {
		 // Serial.println("Invalid CheckSum");
		return;
		}
	
	if (GPSData[3] == 'R') {		// $GPRMC,032849.000,A,
		//Serial.println("$GP-RMC Msg\r\n");
			Unknown_Msg = 0;
			process_GPRMC_data();
			if (Valid_GPS_Data) {
				New_GPS_Data |= 0x01;	// Lat, Lon, GndSpd, CMG
				Load_GPS_Data();
				}
			}
		else
	// -----------------------
	
	if (GPSData[3] == 'G') {
			if (GPSData[4] == 'G') {		// $GPGGA,032850.000,3657.4778,N,
			//Serial.println("$GP-GGA Msg\r\n");
				Unknown_Msg = 0;
				process_GPGGA_data();
				if (Valid_GPS_Data) {
					New_GPS_Data |= 0x02;	// Lat, Lon, Altitude
					Load_GPS_Data();
					}
				}
			else if ((GPSData[4] == 'S') && (GPSData[5] == 'A')) {	// $GPGSA,A,3,28,17,02,15,09,27,26
			   // Serial.println("$GP-GSA Extra Msg\r\n");
				Unknown_Msg = 0;
				}
			else if ((GPSData[4] == 'S') && (GPSData[5] == 'V')) {	// $GPGSV,3,1,11,27
				//Serial.println("$GP-GSV Extra Msg\r\n");	
				Unknown_Msg = 0;
				}
		}
	
	if (Unknown_Msg)
		{
		//Serial.println("Unknown GPS Msg\r\n");
		GPSData[8] = 0;
		//Serial.println((char*)GPSData);
		}
		
	// -----------------------
	
	GPSStrLen = 0;	// clear len, start building next string
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Get GPS transit and Fix data from NMEA GP-RMC string.
// Contains Lat, Lon, Ground Track, Speed, and Date/time
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598,,*10 

void GPS::process_GPRMC_data(void)
{	
	uint8_t i, dPlace, fStart, fEnd;
	char timeStamp[8];
	
	ScanField( 2, &fStart, &dPlace, &fEnd );
	Valid_GPS_Data = (GPSData[fStart] == 'A');
    
	ScanField( 3, &fStart, &dPlace, &fEnd );
	glat1 = GetWordValue( fStart, 2 );
	glat2 = GetWordValue( fStart+2, 2 );
	glat3 = GetWordValue( dPlace+1, 4 );
	
	ScanField( 5, &fStart, &dPlace, &fEnd );
	glon1 = GetWordValue( fStart, 3 );
	glon2 = GetWordValue( dPlace-2, 2 );
	glon3 = GetWordValue( dPlace+1, 4 );
	
	ScanField( 7, &fStart, &dPlace, &fEnd );
	gSpeed = GetFloatValue( fStart, dPlace, fEnd );		// Ground Speed
	
	ScanField( 8, &fStart, &dPlace, &fEnd );	// Course Made Good over ground
	gCMG = GetFloatValue( fStart, dPlace, fEnd );
	
	for (i = 7; i<14; i++) 
		timeStamp[i-7] = GPSData[i];
	timeStamp[6] = 0;
	
//	Serial.println("\r\n- TimeStamp: ");
//	Serial.println(timeStamp);
	
	gTimeStamp = strTolong(timeStamp);
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Get GPS Fix data from GP-GGA NMEA string.
// Contains Lat, Lon, ALTITUDE, and timestamp
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// $GPGGA,230449.000,3657.4767,N,12201.7079,W,1,06,1.7,13.1,M,-26.7,M,,0000*57

void GPS::process_GPGGA_data(void)
	{	
	uint8_t i, dPlace, fStart, fEnd;
	char timeStamp[8];
		
	ScanField( 2, &fStart, &dPlace, &fEnd );
	glat1 = GetWordValue( fStart, 2 );
	glat2 = GetWordValue( fStart+2, 2 );
	glat3 = GetWordValue( dPlace+1, 4 );
	
	ScanField( 4, &fStart, &dPlace, &fEnd );
	glon1 = GetWordValue( fStart, 3 );
	glon2 = GetWordValue( dPlace-2, 2 );
	glon3 = GetWordValue( dPlace+1, 4 );

	Valid_GPS_Data = 1;
	ScanField( 6, &fStart, &dPlace, &fEnd );
	if (GPSData[fStart] == '0') Valid_GPS_Data = 0;
	
	ScanField( 9, &fStart, &dPlace, &fEnd );
	if (GPSData[fStart] == '-') gAltitude = 0.0;
	else gAltitude = GetFloatValue( fStart, dPlace, fEnd );
	
	for (i = 7; i<14; i++) timeStamp[i-7] = GPSData[i];
	timeStamp[6] = 0;

	gTimeStamp = strTolong(timeStamp);
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Scan the GPS data to find a given field
// Enter with fNum.
// Exits with fStart, dPlace and fEnd
// $GPRMC,xxxx,xx

uint8_t GPS::ScanField( uint8_t fNum, uint8_t *fStart, uint8_t *dPlace, uint8_t *fEnd )
	{
    uint8_t i, f = 1;
	*dPlace = 0;
	*fStart = 0;
	
    for (i = 7; i <= GPS_STR_SIZE; i++)
        {
		if (GPSData[i] == ',') f = f + 1;
       
		if (f == fNum)
			{
			*fStart = i + 1;
			while (i < GPS_STR_SIZE)
				{
				i = i + 1;
				if (GPSData[i] == '.') *dPlace = i;
				if ((GPSData[i] == ',') || (GPSData[i] == 0x0D) || (i == GPS_STR_SIZE))
					{
					*fEnd = i;
					return 1;
					}
				}

			*fEnd = i;
			return 1;
			}
		}
	
	return 0;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Get a WORD value from a given field
// Enter with cStart pointing to first chr, nChrs = number of chars

short GPS::GetWordValue(uint8_t cStart, uint8_t nChars)
	{
    short i, fVal = 0;
    for (i = cStart; i < (cStart + nChars); i++)
        fVal = (fVal * 10) + (GPSData[i] - 48);
	return fVal;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Get a Float value from a given field
// Enter with cStart, dPlace, cEnd 

float GPS::GetFloatValue(uint8_t cStart, uint8_t dPlace, uint8_t cEnd)
	{
    short i, iVal = 0;
	float divisor;
	float floatValue;
	
    for (i = cStart; i < dPlace; i++)
        iVal = (iVal * 10) + (GPSData[i] - 48);
	
	floatValue = iVal;
	
	divisor = 1.0;
	iVal = 0;
    for (i = (dPlace+1); i <= cEnd; i++)
        {
		iVal = (iVal * 10) + (GPSData[i] - 48);
		divisor = divisor * 10.0;
		}
	floatValue = floatValue + (float)(iVal / divisor);
	
	return floatValue;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Convert Integer Degrees, Minutes/Fractions into floating point Degrees
// Enter with FPU_Reg = register to load (Lon1,Lat1, etc)
// Deg, Minutes, mFract = lat/lon from GPS
// Result = (((mFract / 10000) + Minutes) / 60) + Degs
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

float Load_Lat_Lon(char NegValue, short Degs, short Minutes, short mFract)
	{
	float tmp;
	
		tmp = (mFract / 10000.0) + Minutes;
		tmp = (tmp / 60.0) + Degs;
		
		if (NegValue)
			tmp = tmp * -1.0;
		
		return tmp;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

void GPS::Load_GPS_Data(void)
	{
	gLat = Load_Lat_Lon( FALSE, glat1, glat2, glat3 );
	gLon = Load_Lat_Lon( TRUE, glon1, glon2, glon3 );
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

char GPS::New_Data()
	{
	char rtnValue = New_GPS_Data;
	New_GPS_Data = 0;
	return rtnValue;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

char GPS::GPS_Valid() {
	return Valid_GPS_Data;
	}

float GPS::Altitude() {
	return gAltitude;
	}

float GPS::CMG() {
	return gCMG;
	}

float GPS::Speed() {
	return gSpeed;
	}

float GPS::Lat() {
	return gLat;
	}

float GPS::Lon() {
	return gLon;
	}

unsigned long GPS::TimeStamp() {
	return gTimeStamp;
	}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

unsigned long strTolong(char *s)
{
    short i;
	unsigned long fVal = 0;
    for (i = 0; i < 7; i++)
        fVal = (fVal * 10) + (s[i] - 48);
	return fVal;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
/*
byte xatoh(char c)
{
 char HexChrs[] = {"0123456789ABCDEF"};
 int x,v;
 for (x=0; x<16; x++)
   if (c == HexChrs[x]) return x;
 return 0;
}
*/
// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------

byte atoh(char c)
{
if ( (c >= '0') && (c <= '9') )
	return (c - '0');
else
if ( (c >= 'A') && (c <= 'F') )
	return (c - 'A' + 10);
return 0;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// $GPGGA,230106.000,3657.4733,N,12201.7087,W,1,04,2.2,31.6,M,-26.7,M,,0000*5A

int GPS::Validate_GPS_CheckSum(void)
{
	int x=1;
	byte tmp, ckSum = 0;
		
	while ((GPSData[x] != '*') && (GPSData[x] != 0x0D))
		{
		ckSum = ckSum ^ GPSData[x];
		x++;
		}
	
	if (GPSData[x] == '*')		// good string - has check sum bytes on end.
		{
		tmp = (16 * atoh(GPSData[x+1])) + atoh(GPSData[x+2]);
        if (tmp == ckSum) return 1;	// check sums match
		}
	
	return 0;	// no check sum on end - invalid string.
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// End of File
