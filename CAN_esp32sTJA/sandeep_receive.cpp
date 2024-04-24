// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
#include <CAN.h>

void setup() {
  Serial.begin(115200);
  while (!Serial);
  // Uart COM failure will halt communication

  Serial.println("CAN Receiver");

  // start the CAN bus at 500 kbps , failure will halt communication
  // 500E3 means bps unit , E3 is 3 exponentiation or 10^3 = kilo , so overall 500kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  // so now no packet has been parsed into (to what , from 8 bit to)
  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print((char)CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  } 
  // else {Serial.println("AAAA");}
}

