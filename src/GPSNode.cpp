#include <ArduinoSTL.h> // C standard Libs

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
// static void printFloat(float val, bool valid, int len, int prec);
// static void printInt(unsigned long val, bool valid, int len);
// static void printDateTime(TinyGPSDate &d, TinyGPSTime &t);
// static void printStr(const char *str, int len);
unsigned char *EncodeFloat(float f);
float DecodeFloat(unsigned char* c);


void loop() {
 
  // what I only want is printing the Latitude , and Longitude data
  // Then encode (This one I may need to understand NMEA sentence a bit)
  /* All of this ties to NMEA Sequence field (not by order) */ 

  // printFloat(gps.location.lat(), gps.location.isValid(), 12, 7); // Latitude 1e7 precision floating point
  // printFloat(gps.location.lng(), gps.location.isValid(), 12, 7); // Longitude 1e7 precision floating point

  // Serial.println(gps.location.lat(),7);

  if(gps.location.isValid()){
     // Encode integer and floating point of both lat long , 4 byte each , precision point of 1e7
    // gps
    // canMsg1.data[0]; 
    // canMsg1.data[1]; 
    // canMsg1.data[2];
    // canMsg1.data[3];
    // canMsg1.data[4];
    // canMsg1.data[5];
    // canMsg1.data[6];
    // canMsg1.data[7];
  }

  smartDelay(100);
  
  // Failure GPS will print the following
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));

    // Error Handing of CAN frame

  }
   float f = 13.1234567; // 4 bytes of data (4 memory slots)

    unsigned char *ch;
    float rev;
    ch = EncodeFloat(f);
    rev = DecodeFloat(ch);
    for (int i = 0; i < sizeof(f); ++i) {
      Serial.print(ch[i],HEX);
    }
    Serial.println();
    Serial.println(rev,8);
    

}


unsigned char *EncodeFloat(float f) {
    // Use memcpy to copy the bytes of the float into the array
    static unsigned char c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    // Copy to address of array , Copy from address of float , size of float
    // Now, c[0] to c[3] contain the bytes of the float
    return c; 
}
float DecodeFloat(unsigned char* c) {
    float f;
    // Use memcpy to copy the bytes from the array back into the float
    memcpy(&f, c, sizeof(f));
    return f;
}


/* Custom BlinkWithout Delay Code that ensures the gps object is being fed by UART reading on time. and contain read function itself */
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
// static void printFloat(float val, bool valid, int len, int prec){
//   if (!valid) {
//     while (len-- > 1)
//       Serial.print('*');
//     Serial.print(' ');
//   }
//   else {
//     Serial.print(val, prec);
//     int vi = abs((int)val);
//     int flen = prec + (val < 0.0 ? 2 : 1); // . and -
//     flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
//     for (int i=flen; i<len; ++i)
//       Serial.print(' ');
//   }
//   smartDelay(0);
// }

// static void printInt(unsigned long val, bool valid, int len) {
//   char sz[32] = "*****************";
//   if (valid)
//     sprintf(sz, "%ld", val);
//   sz[len] = 0;
//   for (int i=strlen(sz); i<len; ++i)
//     sz[i] = ' ';
//   if (len > 0) 
//     sz[len-1] = ' ';
//   Serial.print(sz);
//   smartDelay(0);
// }

// static void printDateTime(TinyGPSDate &d, TinyGPSTime &t) {
//   if (!d.isValid()) {
//     Serial.print(F("********** "));
//   }
//   else {
//     char sz[32];
//     sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
//     Serial.print(sz);
//   }
  
//   if (!t.isValid()) {
//     Serial.print(F("******** "));
//   }
//   else {
//     char sz[32];
//     sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
//     Serial.print(sz);
//   }

//   printInt(d.age(), d.isValid(), 5);
//   smartDelay(0);
// }

// static void printStr(const char *str, int len) {
//   int slen = strlen(str);
//   for (int i=0; i<len; ++i)
//     Serial.print(i<slen ? str[i] : ' ');
//   smartDelay(0);
// }





//Serial.write() vs .print => the difference


/* Encode NMEA*/
