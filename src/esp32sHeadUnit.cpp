//==================================================================================//
#include "Wire.h"
#include <Arduino.h>
#include <CAN.h>

#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
unsigned long lasttime =0;

//==================================================================================//
void canSender();
void canReceiver();

void setup() {
  Serial.begin (115200);
  while (!Serial); // halt if serial port is not available
  // delay (1000);

  Serial.println ("CAN Receiver/Receiver");

  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps , E3 indicates 10^3 or kilos
  if (!CAN.begin (500E3)) {
    Serial.println ("Starting CAN failed!");
    while (1);
  }
  else {
    Serial.println ("CAN Initialized");
  }
}

//==================================================================================//

void loop() {
  if(millis()-lasttime >= 100){
  // canSender();
  canReceiver();
  lasttime = millis();
  }
}

//==================================================================================//

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
  CAN.endPacket();

  //RTR packet with a requested data length
  CAN.beginPacket (0x12, 3, true);
  CAN.endPacket();

  Serial.println ("done");

  delay(1000);
}

//==================================================================================//

void canReceiver() {
  // try to parse packet
  int packetSize = CAN.parsePacket();
  Serial.println(packetSize);

  if (packetSize) {
    // received a packet
    Serial.print ("Received ");

    if (CAN.packetExtended()) {
      Serial.print ("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data (if RTR is 1)
      Serial.print ("RTR ");
    }

    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print (" and requested length ");
      Serial.println (CAN.packetDlc());
    } else {
      Serial.print (" and length ");
      Serial.println (packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        // Serial.print ((char) CAN.read()); // some how unable to read the pakcet , given the  right ID and right DLC? , RTR = 0 it is a data!
        Serial.print (CAN.read()); 
      }
      Serial.println();
    }

    Serial.println();
  } 
}

//==================================================================================//
