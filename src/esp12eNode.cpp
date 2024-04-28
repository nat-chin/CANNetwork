#include "Wire.h"
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
MCP2515 mcp2515(10);
  
#include <SoftwareSerial.h>
#define gpsTx 5
#define gpsRx 4
SoftwareSerial gps(gpsTx,gpsRx);

void setup() {

  /*  CAN frame prep  */
    canMsg1.can_id  = 0x0F6; // 11 bit ID (standard CAN)
    canMsg1.can_dlc = 8; // 8 byte DLC
    

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  gps.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
  // while(gps.available()() >0);
  // Serial.write(gps.read());
  if(gps.available() > 0){
    int gpsdata = gps.read();
    // How to chop gps data by sentences into CAN frame, do I need extended CAN for this
    canMsg1.data[0] = 0x8E; 
    canMsg1.data[1] = 0x87; 
    canMsg1.data[2] = 0x32; 
    canMsg1.data[3] = 0xFA; 
    canMsg1.data[4] = 0x26; 
    canMsg1.data[5] = 0x8E; 
    canMsg1.data[6] = 0xBE; 
    canMsg1.data[7] = 0x86; 
    Serial.write(gpsdata);
    // Serial.println(gps.read());
    // Serial.print(gps.read());
  }
}


//Serial.write() vs .print => the difference
