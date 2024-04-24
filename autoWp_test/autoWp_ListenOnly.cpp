#include <Arduino.h>
// #include <Wire.h>
#include <SPI.h>
#include <mcp2515.h>

struct can_frame canMsg;
MCP2515 mcp2515(10); // Slave select 10 , in arduino , but esp it's gpio5
// MCP2515 mcp2515(5);

void setup() {
  Serial.begin(115200);
  
  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setListenOnlyMode();
  
  Serial.println("------- CAN Read ----------");
  Serial.println("ID  DLC   DATA");
}


// Polling read based approach
void loop() {
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) { // error handling ,, Read the whole 8 byte data at this iteration 

  // The ID and DLC has 1 data each
    Serial.print(canMsg.can_id, HEX); // print ID in hex format
    Serial.print(" "); 
    Serial.print(canMsg.can_dlc, HEX); // print DLC in hex format
    Serial.print(" ");
    
    for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data in hex format also
      Serial.print(canMsg.data[i],HEX); // Printout the data packet that has been kept as array
      Serial.print(" ");
    }

    Serial.println();      
  }
}

/* Okay seems that the data is flooding in, 
F6 8 8E 87 32 FA 26 8E BE 86 -> Seems to be the first data packet from canMsg1
36 8 E 0 0 8 1 0 0 A0 -> Seems to be the first data packet from canMsg2
Containing ID , DLC ,DATA , the data comes in 8 bit package again , not sure if it is to fit into 8 bit memory register (1 slot per 8 bit)
*/ 
