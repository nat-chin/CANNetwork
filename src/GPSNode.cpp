#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int TXPin = 4, RXPin = 5;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/* CAN init */
#include <SPI.h>
#include <mcp2515.h> 
struct can_frame canMsg1;

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  canMsg1.can_id  = 0x0F8; // 11 bit identifier(standard CAN)
  canMsg1.can_dlc = 8; // dlc = data length code -> indicates 8 byte of data that will be sent to the bus (can be less than 8 but not more)
}

static void smartDelay(unsigned long ms);
static void printFloat(float val, bool valid, int len, int prec);
static void printInt(unsigned long val, bool valid, int len);
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
static void printStr(const char *str, int len);


void loop() {

  // what I only want is printing the Latitude , and Longitude data
  // Then encode (This one I may need to understand NMEA sentence a bit)
  /* All of this ties to NMEA Sequence field (not by order) */ 

  printFloat(gps.location.lat(), gps.location.isValid(), 12, 7); // Latitude 1e7 precision floating point
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 7); // Longitude 1e7 precision floating point

  Serial.println();

  // Encode integer and floating point of both lat long , 4 byte each , precision point of 1e7
  canMsg1.data[0]; 
  canMsg1.data[1]; 
  canMsg1.data[2];
  canMsg1.data[3];
  canMsg1.data[4];
  canMsg1.data[5];
  canMsg1.data[6];
  canMsg1.data[7];
  
  // printInt(gps.satellites.value(), gps.satellites.isValid(), 5); // Number of GPS satellies (USA) interacting with out module
  // printInt(gps.location.age(), gps.location.isValid(), 5); // Difference between the Sattelies timestamp and timestamp of the correction data used to update it. 
  // printDateTime(gps.date, gps.time); // Data time stamp when signal reached .
  // printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2); // Altitude from what? a fucking sky?


  /*Later Advance NMEA stuff */
  // printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  // printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  // printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1); // HDOP Geological Accuracy factor??
  // printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);
  // printInt(gps.charsProcessed(), true, 6); // RX Return encodeded Char from NMEA
  // printInt(gps.sentencesWithFix(), true, 10); // RX Return sentencewithfix count from NMEA , (practically it's )
  // printInt(gps.failedChecksum(), true, 9); // Return Sum of Failed check (if 0 means all character is checked)

  
  
  smartDelay(100);
  
  // Failure GPS will print the following
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));

    // Error Handing of CAN frame

  }
}


/* Custom BlinkWithout Delay Code to ensures that the gps object is fed by UART reading. */
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();

  // Execute command in do{} while milis-start < input ms. The condition is deemed to be false after milis and start difference grow past ms
  do {
    // if available, read NMEA and encode to TinyGPS+ instance for fine format.
    while (ss.available()) 
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
}

/* Formatting Function */
static void printFloat(float val, bool valid, int len, int prec){
  if (!valid) {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len) {
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
  if (!d.isValid()) {
    Serial.print(F("********** "));
  }
  else {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid()) {
    Serial.print(F("******** "));
  }
  else {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len) {
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}





//Serial.write() vs .print => the difference

// Let's see Nmea Senstence
// 22:59:41.512 -> $GPRMC,155942.00,A,1342.52690,N,10031.85941,E,0.058,,110624,,,A*7D
// 22:59:41.577 -> $GPVTG,,T,,M,0.058,N,0.107,K,A*28
// 22:59:41.611 -> $GPGGA,155942.00,1342.52690,N,10031.85941,E,1,07,1.21,13.4,M,-27.8,M,,*4F
// 22:59:41.677 -> $GPGSA,A,3,16,03,28,10,32,26,31,,,,,,2.35,1.21,2.01*0F
// 22:59:41.744 -> $GPGSV,4,1,14,01,80,122,22,02,08,269,,03,11,323,15,10,39,157,30*78
// 22:59:41.807 -> $GPGSV,4,2,14,16,39,201,24,21,12,255,,23,06,151,,25,02,036,*72
// 22:59:41.876 -> $GPGSV,4,3,14,26,81,187,26,27,07,189,,28,35,015,21,29,05,078,*74
// 22:59:41.941 -> $GPGSV,4,4,14,31,47,334,25,32,41,068,28*7E
// 22:59:41.973 -> $GPGLL,1342.52690,N,10031.85941,E,155942.00,A,A*69



/*

17:55:52.524 -> Sats HDOP  Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum
17:55:52.555 ->            (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail
17:55:52.587 -> ----------------------------------------------------------------------------------------------------------------------------------------
17:55:52.587 -> **** ***** ********** *********** **** ********** ******** **** ****** ****** ***** ***   ******** ****** ***   63    0         0        
17:55:53.591 -> 7    1.0   13.708792  100.530960  361  06/13/2024 10:55:53 477  24.60  0.00   1.24  N     9540     322.17 NW    551   2         0        
17:55:54.602 -> 7    1.0   13.708794  100.530960  373  06/13/2024 10:55:54 488  24.70  0.00   0.39  N     9540     322.17 NW    1039  4         0        
17:55:55.606 -> 8    1.0   13.708796  100.530960  375  06/13/2024 10:55:55 491  24.70  0.00   0.52  N     9540     322.17 NW    1531  6         0        
17:55:56.610 -> 8    1.0   13.708800  100.530967  383  06/13/2024 10:55:56 498  24.00  0.00   0.94  N     9540     322.17 NW    2023  8         0     


Which data do I need to send in the CAN frame

*/


/* Encode NMEA*/
// import struct

// def encode_location(lat, lon):
//     # Convert to fixed-point representation
//     lat_fixed = int(lat * 1e7)
//     lon_fixed = int(lon * 1e7)
    
//     # Pack into 8-byte CAN frame
//     can_frame = struct.pack('<ii', lat_fixed, lon_fixed)
//     return can_frame

// # Example latitude and longitude
// latitude = 48.1173
// longitude = 11.5167

// # Encode into CAN frame
// can_frame = encode_location(latitude, longitude)
// print(can_frame)

/* Decode (put this in head unit) */
// def decode_location(can_frame):
//     # Unpack from 8-byte CAN frame
//     lat_fixed, lon_fixed = struct.unpack('<ii', can_frame)
    
//     # Convert back to floating point representation
//     lat = lat_fixed / 1e7
//     lon = lon_fixed / 1e7
//     return lat, lon

// # Decode from CAN frame
// decoded_latitude, decoded_longitude = decode_location(can_frame)
// print(decoded_latitude, decoded_longitude)