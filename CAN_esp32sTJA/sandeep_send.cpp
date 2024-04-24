// Copyright (c) Sandeep Mistry. All rights reserved.
// Licensed under the MIT license. See LICENSE file in the project root for full license information.
#include <Arduino.h>
#include <CAN.h>

void setup() {
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps , (This is bit rate not the same as baud rate)

  // baud rate is per second unit, but unlike Hertz , it translate to 
  // bit rate is direct : it is bit data read/write per second => 500kbps is 500,000 bit / 1 second , which is quite fast

  //  if CAN bus can't be join emit error message and halt communication
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");

  CAN.beginPacket(0x12); // specified as a size of 
  CAN.write('h'); // 1 character is one byte -> Similar to how Wire Library works
  CAN.write('e');
  CAN.write('l');
  CAN.write('l');
  CAN.write('o');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);

  // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending extended packet ... ");

  CAN.beginExtendedPacket(0xabcdef);
  CAN.write('w');
  CAN.write('o');
  CAN.write('r');
  CAN.write('l');
  CAN.write('d');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);
}

