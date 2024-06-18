1st mistakes when using library is that, not knowing that CAN library provide a way to access each CAN frame data byte separately
So the following is the attempt of me trying to match the sent message and receivepacket with the same memcpy() and using IEEE-754
converter app to verify the answer , in which it's correct , but I still can't find a way to get CAN data into variable not just Serial monitor , so I just found out that the loop CAN.available means available byte by byte, and it should run 4 times always
that when I stop worrying about the my algorithm being wrong

// unsigned char *packet_array = Encode_bytearray(message);
  // for (int i = 0; i < 4; i++) {
  //       Serial.print(packet_array[3-i], HEX);
  //       Serial.print(',');
  // } Serial.println();
  // Serial.println(receivepacket);