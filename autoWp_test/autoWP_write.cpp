#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg1;
struct can_frame canMsg2;
// init struct of this lib

MCP2515 mcp2515(10);
//Create instance variable from mcp2515 class

void setup() {
  //Preparing CAN frames that contains data packet

  //1st Frame
  canMsg1.can_id  = 0x0F6; // 11 bit identifier (this is standard CAN) , Lower value ID will get Higher priority in holding the bus
  canMsg1.can_dlc = 8; // dlc = data length code -> indicates 8 byte of data that will be sent to the bus (can be less than 8 but not more)
  canMsg1.data[0] = 0x8E; // data packet transmit in both CANH CANL , the 
  canMsg1.data[1] = 0x87;
  canMsg1.data[2] = 0x32;
  canMsg1.data[3] = 0xFA;
  canMsg1.data[4] = 0x26;
  canMsg1.data[5] = 0x8E;
  canMsg1.data[6] = 0xBE;
  canMsg1.data[7] = 0x86;
  //2nd Frame
  canMsg2.can_id  = 0x036;
  canMsg2.can_dlc = 8;
  canMsg2.data[0] = 0x0E;
  canMsg2.data[1] = 0x00;
  canMsg2.data[2] = 0x00;
  canMsg2.data[3] = 0x08;
  canMsg2.data[4] = 0x01;
  canMsg2.data[5] = 0x00;
  canMsg2.data[6] = 0x00;
  canMsg2.data[7] = 0xA0;
  
  while (!Serial); // halt communication if Uart Serial port isn't available (has no byte comming in)
  // while(Serial.available <= 0); is also fine
  Serial.begin(9600);
  
  mcp2515.reset();
   //Set Bit rate to 125KBPS (Need to match with target device [Other nodes that don't need this node data, isn't necessary to match bit rate])
  mcp2515.setBitrate(CAN_125KBPS);
 
  mcp2515.setNormalMode();
  /*
        setConfigMode(); // this one I don't know
        setListenOnlyMode();
        setSleepMode(); // this one I don't know
        setLoopbackMode();
        setNormalMode();
    */


  Serial.println("Example: Write to CAN");
}

void loop() {
  mcp2515.sendMessage(&canMsg1);
  // Pass the address to this method as a way to pass array argument 
  mcp2515.sendMessage(&canMsg2);

// I think the send data is acutally in CAN H and CAN L
  Serial.println("Messages sent");
  
  delay(100);
}
