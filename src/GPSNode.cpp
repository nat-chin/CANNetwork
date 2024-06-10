// Define SoftwareSerial Connection   
#define swsTX 4 // Transmit FROM GPS
#define swsRX 5 // Receive TO GPS

//GPS Baud rate
#define GPSBaud 9600 

//Serial Monitor Baud Rate
#define Serial_Monitor_Baud 115200   

// Include and set up the SoftwareSerial Library
#include <SoftwareSerial.h> 
SoftwareSerial GPSserial(swsRX, swsTX);  
#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsg1;

void setup()
{
 //Start Serial Monitor
 Serial.begin(Serial_Monitor_Baud); 
 
 // Start SoftwareSerial  
  GPSserial.begin(GPSBaud);
  canMsg1.can_id  = 0x0F8; // 11 bit identifier(standard CAN)
  canMsg1.can_dlc = 8; // dlc = data length code -> indicates 8 byte of data that will be sent to the bus (can be less than 8 but not more)
  canMsg1.data[0] = 0x8E; // Accel X
  canMsg1.data[1] = 0x87; // Accel Y
  canMsg1.data[2] = 0x32; // Accel Z
  canMsg1.data[3] = 0xFA; // GyRo X
  canMsg1.data[4] = 0x26; // GyRo Y
  canMsg1.data[5] = 0x8E; // GyRo Z
  canMsg1.data[6] = 0xBE; // Encode DT
  canMsg1.data[7] = 0x86; // Encode CLK

}
    
void loop()
{
  // Write SoftwareSerial data to Serial Monitor
  while (GPSserial.available() > 0){
    Serial.write(GPSserial.read());
    // Serial.println(GPSserial.read());
  }
  
}

//Serial.write() vs .print => the difference
