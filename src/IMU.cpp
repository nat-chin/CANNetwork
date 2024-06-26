// This node maybe change to PI pico instead of UNO R3
#include "Arduino.h"

/* mcp2515 init */
#include <SPI.h>
#include <mcp2515.h> 
#define standard_bitrate CAN_125KBPS
#define standard_delay 100
#define standard_dlc 4
MCP2515 mcp2515(10);
// Use the Same as Source Clock of MCP2515 ,
struct can_frame canMsg1 , canMsg2 , canMsg3 , canMsg4 , canMsg5, canMsg6, canMsg7;  
//  1-7 CAN frame

/*  IMU  */
  #include "Wire.h"
  #include <MPU6050_light.h>
  MPU6050 mpu(Wire); unsigned long lasttime = 0;

// --------------------------------------------------------//

volatile bool interrupt = false;
void IRQ_HANDLER() {
  interrupt = true;
}

float* readMPU();  
void readEncoder(); void resetEncoder(); 
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);

 
void setup() { 
  Serial.begin (115200);
  Wire.begin(); // join I2C bus (IMu is I2C COM)

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(standard_bitrate); //Set Bit rate to 125KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /*  IMU init  */
    mpu.begin(); // initialize mpu instance
    Serial.println(F("Calculating gyro offset, so do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(); // Calibrate both gyro and acc offset 
    Serial.println("Done: \n");

    attachInterrupt(digitalPinToInterrupt(2), IRQ_HANDLER ,FALLING); // interrupt pin driven low after triggered

  /*  Set CAN Frame struct. */
  // canMsg1.can_id  = 0x10; // 11 bit ID (standard CAN)
  canMsg1.can_id  = 0x10; 
  canMsg2.can_id  = 0x11; 
  canMsg3.can_id  = 0x12; 
  canMsg4.can_id  = 0x13; 
  canMsg5.can_id  = 0x14; 
  canMsg6.can_id  = 0x15; 
  
  canMsg1.can_dlc = canMsg2.can_dlc = canMsg3.can_dlc = 1;
  canMsg4.can_dlc = canMsg5.can_dlc = canMsg6.can_dlc = 1; 

  // Dummy data for test
  canMsg1.data[0] = 0x01;
  canMsg2.data[0] = 0x02;
  canMsg3.data[0] = 0x03;
  canMsg4.data[0] = 0x04;
  canMsg5.data[0] = 0x05;
  canMsg6.data[0] = 0x06;

  //Set Mask and Filter Receiving of ID to receive only from Head unit , using only RXB0 
  // mcp2515.setFilter(MCP2515::RXF0, false, 0x01); // Filter for RXB0 , accept ID 0x01 (Head Unit ID)

  // Some how Setting Receive filter makes this shit don't transmit??
} 

void loop() { 
  
  // float* mpuData = readMPU();
  
    
  if(millis()-lasttime >= standard_delay){      
    
    // // Accel X 2nd Frame
    // unsigned char* sendByte_Accelx = Encode_bytearray(mpuData[0]);  
    // for(int i=0 ; i < standard_dlc ; i++){
    //   canMsg1.data[i] = sendByte_Accelx[i];
    // }

    
    // // Accel Y 3rd Frame
    // unsigned char* sendByte_Accely = Encode_bytearray(mpuData[1]);  
    // for(int i=0 ; i< standard_dlc  ; i++){
    //   canMsg2.data[i] = sendByte_Accely[i];
    // }


    // // Accel Z 4th Frame
    // unsigned char* sendByte_Accelz = Encode_bytearray(mpuData[2]);  
    // for(int i=0 ; i< standard_dlc  ; i++){
    //   canMsg3.data[i] = sendByte_Accelz[i];
    // }
  // mcp2515.getStatus();
    // // Gyro X 5th Frame
    // unsigned char* sendByte_Gyrox = Encode_bytearray(mpuData[3]);  
    // for(int i=0 ; i< standard_dlc  ; i++){
    //   canMsg4.data[i] = sendByte_Gyrox[i];
    // }

    // // Gyro Y 6th Frame
    // unsigned char* sendByte_Gyroy = Encode_bytearray(mpuData[4]);  
    // for(int i=0 ; i< standard_dlc  ; i++){
    //   canMsg5.data[i] = sendByte_Gyroy[i];
    // }

    // // Gyro Z 7th Frame
    // unsigned char* sendByte_Gyroz = Encode_bytearray(mpuData[5]);  
    // for(int i=0 ; i< standard_dlc  ; i++){
    //   canMsg6.data[i] = sendByte_Gyroz[i];
    // }

    // Use write Interrupt Instead
    // Transmit CAN frame out of mcp2515 TX Buffer
    // Que 6 message into 3 Transmit Buffer manually , therefore send 2 time with delay
    // 1st we will receive the RTR packet from esp32 to trigger the fire receive interrupt to trigger a condition to start stream data
    // but we won't use that RTR packet to response back  
    

// // Error Detection
  uint8_t errorFlags = mcp2515.getErrorFlags();
    // Serial.println(errorFlags);
    
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
    
// RX buffer overflow and anykind of Bus off status
if (errorFlags & MCP2515::EFLG_RX0OVR)
    Serial.println("RXB0 overflow error");
if (errorFlags & MCP2515::EFLG_RX1OVR)
    Serial.println("RXB1 overflow error");
if (errorFlags & MCP2515::EFLG_TXBO)
    Serial.println("Bus-off error");

// Error Handling


    mcp2515.sendMessage(&canMsg1);
    // mcp2515.sendMessage(MCP2515::TXB0 ,&canMsg1);
    // mcp2515.sendMessage(MCP2515::TXB1 ,&canMsg2);
    // mcp2515.sendMessage(MCP2515::TXB2 ,&canMsg3);

    // mcp2515.sendMessage(MCP2515::TXB0 ,&canMsg4);
    // mcp2515.sendMessage(MCP2515::TXB1 ,&canMsg5);
    // mcp2515.sendMessage(MCP2515::TXB2 ,&canMsg6);    
    // Serial.println("Messages sent");
    lasttime = millis(); // reset timer variable
  }
  

//Time for timed Schedule , with Current TWAI CAN low level that I need to config : It is out of scope now.

  
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

