#include <ArduinoSTL.h> // C standard Libs

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int TXPin = 4, RXPin = 5;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/* mcp2515 init */
#include <SPI.h>
#include <mcp2515.h> 
#define standard_bitrate CAN_500KBPS
#define standard_delay 100
MCP2515 mcp2515(10);
struct can_frame canMsg1; // 1st CAN frame
struct can_frame canMsg2;  // 2nd CAN frame

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();
  /* Set CAN Frame struct.*/
  canMsg1.can_id  = 0x0F1; // 11 bit identifier(standard CAN)
  canMsg1.can_dlc = 8; // dlc = data length code -> max 8 byte of data

}

static void smartDelay(unsigned long ms);
unsigned char *Encode_bytearray(float f);
float Decode_bytearray(unsigned char* c);


void loop() {
  smartDelay(standard_delay); // 1ms delay for feeding gps object
  if(gps.location.isValid()){
    // Return type of lat() lng() is 8 byte double each , but the goal here is to cram both message into one CAN frame
    // Encode integer and floating point of both lat long , 4 byte each , precision point of 1e7
    
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    unsigned char *sendByteLat = Encode_bytearray(lat);
    unsigned char *sendByteLng = Encode_bytearray(lng);
    // float receiveFloat = Decode_bytearray(Packet)
    byte aa = 0;
    for(int i = 0;i < sizeof(lat); i++) {
        aa = sendByteLat[i];
        Serial.println(aa);
    }
    Serial.println()
    // Serial.println(lat,7);
    // Serial.println(lng,7);
 
    // Latitude
    canMsg1.data[0] = sendByteLat[0]; 
    canMsg1.data[1] = sendByteLat[1]; 
    canMsg1.data[2] = sendByteLat[2];
    canMsg1.data[3] = sendByteLat[3];
    // Longitude
    canMsg1.data[4] = sendByteLng[4];
    canMsg1.data[5] = sendByteLng[5];
    canMsg1.data[6] = sendByteLng[6];
    canMsg1.data[7] = sendByteLng[7];

    // Transmit CAN frame out of mcp2515 FIFO Buffer then transmit into CAN Bus
    mcp2515.sendMessage(&canMsg1);  

    // May add some Acknowledgement functionality
    /*
    
    */
    
  }

  // Failure GPS will print the following
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    // Error Handing of CAN frame

  }
}

unsigned char *Encode_bytearray(float f) {
    // Use memcpy to copy the bytes of the float into the array
    static unsigned char c[sizeof(f)]; 
    memcpy(c, &f, sizeof(f));
    // Copy to address of array , Copy from address of float , size of float
    // Now, c[0] to c[3] contain the bytes of the float
    return c; 
}
float Decode_bytearray(unsigned char* c) {
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