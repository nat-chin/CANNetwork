//==================================================================================//
// #include "Wire.h"
#include <Arduino.h>
#include <CAN.h>
// #include <esp32-hal.h>
// extern "C"{
//   #include <driver/twai.h>
// }


#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
#define standard_bitrate 125E3
#define standard_delay 10
#define standard_dlc 4
unsigned long lasttime =0;

//==================================================================================//
void canSender(); void canReceiver();
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);
float readCurrent(); float readVoltage();

/* Voltage and Current Sense from Battery */
  #define AmpsPin A11
  #define VoltsPin A0

void setup() {

  Serial.begin (115200);
  // Serial1.begin (115200);
  // Serial2.begin (115200);
  while (!Serial); // halt if serial port is not available
  Serial.println ("CAN Receiver/Receiver");
  delay(500);
  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps , E3 indicates 10^3 or kilos
  if (!CAN.begin (standard_bitrate)) {
    Serial.println ("Starting CAN failed!");
    while (true);
  }
  else {
    Serial.println ("CAN Initialized");
  }
}

// Now only problem is how to read get full data out of this CAN library??? Huh??? , I don't want to do Serial.parseInt()
unsigned char message[4];

void loop() {
  // CAN message Event Scheduling Rate (Expect 9 message in total) (10 ms each)

  // Read Message by Polling
  if(millis()-lasttime >= standard_delay){
    canReceiver();
    /* Function to Record all data to local SD card in CSV format (Optional) */

    lasttime = millis();
  }
 
}
/* Sample of Received Bit */


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

      // int i = 0; // set iteration at zero
      while (CAN.available()) {
        // This system BUS is only 4 byte data , so 4 iteration
        Serial.print(CAN.read(),HEX); 
        // message[i] = CAN.read();  
        // i++;
        } 
      Serial.println();
    }
  } 
}

// Sending Acknowledgement Bit (??)
void canSender() {
  Serial.print ("Sending packet ... ");
  // CAN.beginPacket (0x12);  //sets the ID and clears the transmit buffer
  // // CAN.beginExtendedPacket(0xabcdef);
  // CAN.write ('1'); //write data to buffer. data is not sent until endPacket() is called.
  // CAN.endPacket();

  //RTR packet with a requested data length
  // RTR sends empty packet and request some data length back
  CAN.beginPacket (0x12, 0, true);
  CAN.endPacket();

  Serial.println ("done");

  // delay(1000);
}

//==================================================================================//
