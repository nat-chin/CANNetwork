//==================================================================================//
// #include "Wire.h"
#include <Arduino.h>
#include <CAN.h>
// #include <esp32-hal.h>
extern "C"{
  #include <driver/twai.h>
  // #include <driver/gpio.h>
  // #include "soc/clk_tree_defs.h"
}

#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
#define standard_bitrate 125E3
#define standard_delay 90
#define standard_dlc 4

//==================================================================================//
volatile bool interrupt = false;


void canReceiver();
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);
void canNodeTrigger(uint16_t ID);

void setup() {
  Serial.begin (115200);
  // Serial1.begin (115200);
  // Serial2.begin (115200);

  while (!Serial); // halt if serial port is not available
  Serial.println ("CAN Receiver/Receiver");
  
  
  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps , E3 indicates 10^3 or kilos
  if (!CAN.begin (standard_bitrate)) {
    Serial.println ("Starting CAN failed!");
    while (true);
  }
  else { Serial.println ("CAN Initialized"); }

  // TWAI Dual Filter (How to activate???)
  // Okay let's use single filter for now  (32 bit Acceptance)
  CAN.filter(0x10,0x10); // 0x10-0x1F

  // delay(10); // Proper Delay should be used , (Not Exceeding standard delay on Sender Side)
  Serial.print ("Sending Trigger to start the System ... ");
  CAN.beginPacket (0x01, 1, false); CAN.write(123); CAN.endPacket();
  Serial.println ("done");
}

void assignDataToArray(int* destArray, uint8_t* srcData, uint8_t size) {
    for (int i = 0; i < size; i++) {
        destArray[i] = srcData[i];
    }
}


const short canMsgNum = 10;  // Number of arrays buffer correspond to Blynk Virtual Pin
const short candlc = 4;  // Assuming canMsg.can_dlc is 8
int blynkData[canMsgNum][candlc];  // Array of arrays to hold data sets
uint8_t canMsgCount = 0;
unsigned long lasttime = 0;
unsigned char message[4];
volatile bool doneflag = false;
void loop() {

  if(millis()-lasttime >= standard_delay) { // 90 millis delay for 10 messages
    // Polling from RX Buffer [Better to use Receive Interrupt, but this will suffice , since bus load isn't high]
    if (CAN.parsePacket()) {
      
      uint8_t canMsgID = CAN.packetId();
      uint8_t canMsgDlc = CAN.packetDlc();
      uint8_t canMsgbuf[canMsgDlc];
      // Display Message Content
      Serial.print ("Packets of id 0x"); Serial.print (canMsgID, HEX);
      Serial.print (" of length "); Serial.println (canMsgDlc);

      /* Only Sample to data that isn't a done flag (Sample data to Buffer Before Blynk) : 0x1F is TX Done Flag 
      Transfer Current canMsg Buffer to 2D array that Each element is Data Buffer correspond to Blynk Virtual Pin */
      if(canMsgID != 0x1F){
        // Read CAN data field Byte by Byte after parsing the message from RX buffer
        int i = 0; while (CAN.available()){ 
          canMsgbuf[i] = CAN.read(); // Read , and Clear Rx buffer
          i++; }

        /*Decode Message back to 4 byte float */
        float receiveFloat = Decode_bytearray(canMsgbuf); Serial.println(receiveFloat,7);
        assignDataToArray(blynkData[canMsgCount], canMsgbuf,canMsgDlc);
       

        Serial.print("Data for Each Blynk Virtual Pin: ");
        for (int i = 0; i< canMsgDlc ; i++){
          Serial.print(blynkData[canMsgCount][i]); // Let's peek at 1st msg 1st byte , should output 1 
        } Serial.println();
        canMsgCount = (canMsgCount + 1) % canMsgNum; // Increment counter fot next Receive interval, reset upon meeting maximum canMsgnum

      } else if(canMsgID == 0x1F){
        /* Send packet of ID 0x01 to signal all Non-Head unit Node to Starts streaming Sensor Data */

        // 1. Read from RX buffer when ID = 0x1F
          int i = 0; while (CAN.available()){ 
          canMsgbuf[i] = CAN.read(); 
          Serial.print((char)canMsgbuf[i]);
          i++; } Serial.println();

        // Message 'A' mean NodeA finished Transmission, Trigger NodeB
        if(canMsgbuf[0] == 'A'){ 
          CAN.beginPacket(0x02,1,false); CAN.write(1); CAN.endPacket(); // TrgMsg ID 0x02 : Node B
          Serial.println("Done Flag Detected A");
        }
        // Message B mean NodeB finished Transmission , Trigger NodeA 
        else if(canMsgbuf[0] == 'B' ){ 
          CAN.beginPacket(0x01,1,false);CAN.write(1); CAN.endPacket(); // TrgMsg ID 0x01 : Node A
          Serial.println("Done Flag Detected B");
        }
      }
    }
    lasttime = millis();
  }
}
  
/* Sample of Received Bit */
// Blynk API code


/* Function to Record all data to local SD card in CSV format (Optional) */

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

//==================================================================================//

void canReceiver() {
  int packetSize = CAN.parsePacket(); // Parse packet

  // received a packet
  if (packetSize > 0) {
    Serial.print ("Received ");
    // Extended CAN ID check
    if (CAN.packetExtended()) { Serial.print ("extended ");}
    
    /* Check Standard CAN ID */
    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);
    
    // RTR check (if RTR , then RTR bit is 1)
    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
      Serial.print (" with requested length ");
      Serial.println (CAN.packetDlc());
    } else {

      /* Print out NON-RTR packet  (read Each byte)*/
      Serial.print (" Of length: ");
      Serial.println (packetSize);

      int i = 0; // set iteration at zero
      while (CAN.available()) {
        // This system BUS is only 4 byte data , so 4 iteration
        // Serial.print(CAN.read()); 
        message[i] = CAN.read();  
        i++;
        } 
      // Serial.println();
    }
  } 
  // else{ Serial.println("NO Packet");}
}


  // if(millis()-lasttime >= standard_delay){ // 90 millis delay for 10 messages
  //   canReceiver(); // Read data from RX buffer Per Iteration (Per message transmission delay)

  //   /*Decode Message back to 4 byte float */
  //   float receiveFloat = Decode_bytearray(message);
  //   Serial.println(receiveFloat,7);

  //   /* Confirm 4 bytes message each CAN frame*/
  //   for (int i = 0; i < 4; i++) {
  //     Serial.print(message[3-i],DEC);
  //     Serial.print(',');
  //   } Serial.println();

  //   for (int i = 0; i < 4; i++) {
  //     Serial.print(message[3-i], HEX);
  //     Serial.print(',');
  //   } Serial.println();

  //   lasttime = millis();
  // }


//==================================================================================//





