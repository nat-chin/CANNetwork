//==================================================================================//
#include "Wire.h"
#include <Arduino.h>
#include <CAN.h>

#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
unsigned long lasttime =0;

//==================================================================================//
void canSender(); void canReceiver();
unsigned char *Encode_bytearray(float f);
float Decode_bytearray(unsigned char* c);

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
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (true);
  }
  else {
    Serial.println ("CAN Initialized");
  }
}

// Receiving & Record Data to Local SD Card

// #include <ArduinoSTL.h>
// #include <typeinfo>

// Now only problem is how to read get full data out of this CAN library??? Huh??? , I don't want to do Serial.parseInt()
unsigned char message[4];


void loop() {
  
  if(millis()-lasttime >= 100){
  // canSender();
  canReceiver();
  // Only one frame
  // unsigned char *msgptr = &message[0];

  /*Decode Latitude and Longtitude Data*/
  float receiveFloat = Decode_bytearray(message);
  Serial.println(receiveFloat,7);
  Serial.println();
  /* Confirm 4 bytes message in CAN frame*/
  for (int i = 0; i < 4; i++) {
        Serial.print(message[3-i]);
        Serial.print(',');
  } Serial.println();
  for (int i = 0; i < 4; i++) {
        Serial.print(message[3-i], HEX);
        Serial.print(',');
  } Serial.println();
  
  

  /* But arbritation table
  Lat : 0xF1
  Lng : 0xF2 

  So the order of received message per clock (with delay) is 
  Lat Lng RPM Accelx , Accely , Accelz , Gyro x , Gyro y , Gyro z
  */

  

  /* Function to Record all data to local SD card in CSV format */

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
  // try to parse packet
  int packetSize = CAN.parsePacket();

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
      Serial.print (" Of length: ");
      Serial.println (packetSize);

      /* Print out NON-RTR packet  (read Each byte)*/
      int i = 0; // set iteration at zero
      while (CAN.available()) {
        //  4 iteration
        // Serial.print ((char) CAN.read()); // some how unable to read the pakcet , given the  right ID and right DLC? , RTR = 0 it is a data!
        // Serial.print(CAN.read()); 
        message[i] = CAN.read();  
        i++;
        } 
      // Serial.println();
    }
  } 
}

// Sending Acknowledgement Bit (??)
void canSender() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print ("Sending packet ... ");

  CAN.beginPacket (0x12);  //sets the ID and clears the transmit buffer
  // CAN.beginExtendedPacket(0xabcdef);
  CAN.write ('1'); //write data to buffer. data is not sent until endPacket() is called.
  CAN.write ('2');
  CAN.write ('3');
  CAN.write ('4');
  CAN.write ('5');
  CAN.write ('6');
  CAN.write ('7');
  CAN.write ('8');
  // 8 byte data frame
  CAN.endPacket();

  //RTR packet with a requested data length
  // RTR sends empty packet and request some data length back
  CAN.beginPacket (0x12, 3, true);
  CAN.endPacket();

  Serial.println ("done");

  delay(1000);
}

//==================================================================================//
