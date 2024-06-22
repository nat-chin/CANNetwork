// This node maybe change to PI pico instead of UNO R3
#include "Arduino.h"

/* mcp2515 init */
#include <SPI.h>
#include <mcp2515.h> 
#define standard_bitrate CAN_125KBPS
#define standard_delay 10
#define standard_dlc 4
MCP2515 mcp2515(10,MCP_8MHZ);
struct can_frame canMsg1 , canMsg2 , canMsg3 , canMsg4 , canMsg5, canMsg6, canMsg7 , canMsg8 , canMsg9;  
//  1-7 CAN frame

/*  Rotary Encoder (Will change Code to Motor Encoder Later) */
  #define CLK 4 // Input B
  #define DT 5 // Input A
  #define SW 3 // Reset Switch
  float counter = 0;
  int currentStateCLK; int lastStateCLK;
  char currentDir = ' '; // Consider CW as +

/*  IMU  */
  #include "Wire.h"
  #include <MPU6050_light.h>
  MPU6050 mpu(Wire); unsigned long lasttime = 0;

// --------------------------------------------------------//
float* readMPU();  
void readEncoder(); void resetEncoder(); 
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);

 
void setup() { 
  Serial.begin (115200);
  Wire.begin(); // join I2C bus (IMu is I2C COM)

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    delay(500);
    mcp2515.reset();
    mcp2515.setBitrate(standard_bitrate); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /*  Set CAN Frame format -> Simulate 9 message from Sensors */
  canMsg1.can_id  = 0x10; // 11 bit ID (standard CAN)
  canMsg2.can_id  = 0x20; 
  canMsg3.can_id  = 0x30; 
  canMsg4.can_id  = 0x40; 
  canMsg5.can_id  = 0x50; 
  canMsg6.can_id  = 0x60; 
  canMsg7.can_id  = 0x70;
  canMsg8.can_id  = 0x80; 
  canMsg9.can_id  = 0x90; // Priority if send simultaneously : canMsg1 -> canMsg9  
  canMsg1.can_dlc = canMsg2.can_dlc =  canMsg3.can_dlc = canMsg4.can_dlc = 1;
  canMsg5.can_dlc = canMsg6.can_dlc = canMsg7.can_dlc = canMsg8.can_dlc = canMsg9.can_dlc =  1; 
  

  canMsg1.data[0] = 0x01;
  canMsg2.data[0] = 0x02;
  canMsg3.data[0] = 0x03;
  canMsg4.data[0] = 0x04;
  canMsg5.data[0] = 0x05;
  canMsg6.data[0] = 0x06;
  canMsg7.data[0] = 0x07;
  canMsg8.data[0] = 0x08;
  canMsg9.data[0] = 0x09;
} 

void loop() { 
    // CAN message Event Scheduling Rate (Expect 9 message in total) (10 ms each)

    // Send Message By Polling
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg1);
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg2); 
    
    delay(standard_delay); 
    mcp2515.sendMessage(&canMsg3);
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg4);
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg5);  
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg6);  
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg7); 
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg8); 
    
    delay(standard_delay);
    mcp2515.sendMessage(&canMsg9); 
    
    
    // Delay by 25 ms for 4 message , reader side will start reading at 1st millis() , assume both is reboot at the same time
    // if the reboot isn't at the same time , problem might occur

  // if(millis()-lasttime >= standard_delay){
   
      
       
  //   // Serial.println("Messages sent");
  //   lasttime = millis(); // reset timer variable
  // }
  
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

// Rotary Encoder need debouncing , as spinning to fast generate noise

void readEncoder() {
        
	// Read the current state of CLK
	currentStateCLK = digitalRead(CLK);

	// If last and current state of CLK are different, then pulse occurred
	// React to only 1 state change to avoid double count

	if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
		// If the DT state is different than the CLK state then
		// the encoder is rotating CW so Increment
		if (digitalRead(DT) != currentStateCLK) {
      counter ++;
			currentDir ='+';
		} else {
			// Encoder is rotating CCW so decrement instead
      counter--; 
			currentDir ='-';
		}
	}
	// Remember last CLK state
	lastStateCLK = currentStateCLK;
}

void resetEncoder(){
      counter = 0 ;
      currentDir = ' ';
}





