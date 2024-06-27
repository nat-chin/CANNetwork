// This node maybe change to PI pico instead of UNO R3
#include <ArduinoSTL.h> // C standard Libs , already include Arduino.h

/* mcp2515 init */
  #include <SPI.h>
  #include <mcp2515.h> 
  #define standard_bitrate CAN_125KBPS
  #define standard_delay 80
  #define standard_dlc 4
  MCP2515 mcp2515(10);
  struct can_frame canMsg1, canMsg2, canMsg3 ,canMsg4, canMsg5, canMsg6, doneMsg; ; // CAN frame Transmit to Headunit 

  #define AcceptedTriggerID 0x02
  struct can_frame trgMsg; // CAN frame Received From Headunit

//  1-7 CAN frame

/*  IMU  */
  #include "Wire.h"
  #include <MPU6050_light.h>
  MPU6050 mpu(Wire); unsigned long lasttime = 0;
// --------------------------------------------------------//

// volatile bool interrupt = false;
// void IRQ_HANDLER() {
//   interrupt = true;
// }

float* readMPU(); void mcp2515Error(); 
void readEncoder(); void resetEncoder(); 
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);

 
void setup() { 
  Serial.begin (115200);
  Wire.begin(); // join I2C bus (IMu is I2C COM)

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(standard_bitrate,MCP_16MHZ); //Set Bit rate to 125KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /*  IMU init  */
    mpu.begin(); // initialize mpu instance
    Serial.println(F("Calculating gyro offset, so do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(); // Calibrate both gyro and acc offset 
    Serial.println("Done: \n");

    // attachInterrupt(digitalPinToInterrupt(2), IRQ_HANDLER ,FALLING); // interrupt pin driven low after triggered

  /*  Set CAN Frame struct. */
  canMsg1.can_id  = 0x14; 
  canMsg2.can_id  = 0x15; 
  canMsg3.can_id  = 0x16; 
  canMsg4.can_id  = 0x17; 
  canMsg5.can_id  = 0x18; 
  canMsg6.can_id  = 0x19; 
  
  canMsg1.can_dlc = canMsg2.can_dlc = canMsg3.can_dlc = standard_dlc;
  canMsg4.can_dlc = canMsg5.can_dlc = canMsg6.can_dlc = standard_dlc; 


  //Set Mask and Filter Receiving of ID to receive only from Head unit , using only RXB0 
  // mcp2515.setFilter(MCP2515::RXF0, false, 0x02); // Filter for RXB0 , accept ID 0x01 (Head Unit ID)

} 

// CAN message Scheduling
// volatile bool trigger = false;   
volatile uint8_t msg_counter = 0; // will be set to one after the 1st trigger 
unsigned long last_msg_time = 0; // get this out the loop
volatile bool doneFlag = false;

void loop() { 

  mcp2515Error();
  // Polling for Trigger from ESP32 Head unit (Better to use Interrupt , but INT pin is damaged rn.)
  if(millis()-last_msg_time >= 10){
    if (mcp2515.readMessage(&trgMsg) == MCP2515::ERROR_OK) { 
      if(trgMsg.can_id == AcceptedTriggerID){
        msg_counter = 1; }
      Serial.println("Node B Triggered");
      last_msg_time = millis(); // Assign last_msg_time for the 1st time for next comparison  
    } 
  }  
  float* mpuData = readMPU();  
    
    // Accel X 2nd Frame
    unsigned char* sendByte_Accelx = Encode_bytearray(mpuData[0]);  
    for(int i=0 ; i < standard_dlc ; i++){
      canMsg1.data[i] = sendByte_Accelx[i];
    }

    // Accel Y 3rd Frame
    unsigned char* sendByte_Accely = Encode_bytearray(mpuData[1]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg2.data[i] = sendByte_Accely[i];
    }

    // Accel Z 4th Frame
    unsigned char* sendByte_Accelz = Encode_bytearray(mpuData[2]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg3.data[i] = sendByte_Accelz[i];
    }
  
    // Gyro X 5th Frame
    unsigned char* sendByte_Gyrox = Encode_bytearray(mpuData[3]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg4.data[i] = sendByte_Gyrox[i];
    }

    // Gyro Y 6th Frame
    unsigned char* sendByte_Gyroy = Encode_bytearray(mpuData[4]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg5.data[i] = sendByte_Gyroy[i];
    }

    // Gyro Z 7th Frame
    unsigned char* sendByte_Gyroz = Encode_bytearray(mpuData[5]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg6.data[i] = sendByte_Gyroz[i];
    }
    
  /* Write Message to TX buffer then let protocol Engine transmit into CAN Bus with Proper Bit timing */
    
   
  
  if ((msg_counter == 1) && (millis() - last_msg_time >= standard_delay)) {

  mcp2515.sendMessage(&canMsg1);
  Serial.println("Sent 1");
  msg_counter = 2; // Step to Next Msg.
  last_msg_time = millis();
  }

  // sending 2nd message 
  if ((msg_counter == 2) && (millis() - last_msg_time >= standard_delay)) {

    mcp2515.sendMessage(&canMsg2);
    Serial.println("Sent 2");
    msg_counter = 3; // Step to Next Msg.
    last_msg_time = millis();
  }

  // sending 3rd message
  if ((msg_counter == 3) && (millis() - last_msg_time >= standard_delay)) {

    mcp2515.sendMessage(&canMsg3);
    Serial.println("Sent 3");
    msg_counter = 4; // Step to Next Msg.
    last_msg_time = millis();
  }

  // sending 4th message
  if ((msg_counter == 4) && (millis() - last_msg_time >= standard_delay)) {

    mcp2515.sendMessage(&canMsg4);
    Serial.println("Sent 4");
    msg_counter = 5; // Reset Back to Bus Off state waiting for Head unit to allow Next Msg.
    last_msg_time = millis();
  }

  // sending 5th message
  if ((msg_counter == 5) && (millis() - last_msg_time >= standard_delay)) {
    mcp2515.sendMessage(&canMsg5);
    Serial.println("Sent 5");
    msg_counter = 6; // Reset Back to Bus Off state waiting for Head unit to allow Next Msg.
    last_msg_time = millis();
  }

  // sending 5th message
  if ((msg_counter == 6) && (millis() - last_msg_time >= standard_delay)) {
    mcp2515.sendMessage(&canMsg6);
    Serial.println("Sent 6");
    msg_counter = 0; // Reset Back to Bus Off state waiting for Head unit to allow Next Msg.
    doneFlag = true;
    last_msg_time = millis();
  }

  /* sending Done Flag to signal Head unit to sent The trigger again */
  if ((doneFlag == true) && (millis() - last_msg_time >= standard_delay)) {
    doneMsg.can_id  = 0x1F; doneMsg.can_dlc = 1; doneMsg.data[0] = 'B';
    mcp2515.sendMessage(&doneMsg);
    Serial.println("Done Flag B Sent");
    doneFlag = false;
    last_msg_time = millis();
  }
    
// Error Handling
  
}

// CAN Error Flag Detection , (and Error Frame Detection + TEC REC counter)
void mcp2515Error(){
  // Error Detection , by getting Error Flag. 
  uint8_t errorFlags = mcp2515.getErrorFlags();

  if (errorFlags & MCP2515::EFLG_TXBO)
      Serial.println("TX Bus off");
  if (errorFlags & MCP2515::EFLG_TXEP)
      Serial.println("TX Error-Passive");
  if (errorFlags & MCP2515::EFLG_RXEP)
      Serial.println("RX Error-Passive");
  if (errorFlags & MCP2515::EFLG_TXWAR)
      Serial.println("TX Error Warning");
  if (errorFlags & MCP2515::EFLG_RXWAR)
      Serial.println("RX Error Warning");
  if (errorFlags & MCP2515::EFLG_EWARN)
    Serial.println("Overall Error Warning");

  // Buffer Overflow Error
  if (errorFlags & MCP2515::EFLG_RX0OVR)
      Serial.println("RXB0 overflow error");
  if (errorFlags & MCP2515::EFLG_RX1OVR)
      Serial.println("RXB1 overflow error");
  
  /* Error Confinement Counter*/
  // uint8_t REC = mcp2515.errorCountRX();
  // uint8_t TEC = mcp2515.errorCountTX();
}

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

float* readMPU() {
  static float imuData[6];
  mpu.update(); // Update the sensor register with new values 

  // Acceleration Data is multiple of gravity constant g = 9.81 m/s^2
  imuData[0] = mpu.getAccX()*9.80665; // Accel X
  imuData[1] = mpu.getAccY()*9.80665; // Accel Y
  imuData[2] = mpu.getAccZ()*9.80665; // Accel Z
  imuData[3] = mpu.getAngleX(); // Gyro X
  imuData[4] = mpu.getAngleY(); // Gyro Y
  imuData[5] = mpu.getAngleZ(); // Gyro Z
  return imuData; // Return the array of IMU data
}

