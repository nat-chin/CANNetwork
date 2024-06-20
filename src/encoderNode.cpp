// This node maybe change to PI pico instead of UNO R3
#include "Arduino.h"

/* mcp2515 init */
#include <SPI.h>
#include <mcp2515.h> 
#define standard_bitrate CAN_500KBPS
#define standard_delay 100
#define standard_dlc 4
MCP2515 mcp2515(10);
struct can_frame canMsg1 , canMsg2 , canMsg3 , canMsg4 , canMsg5, canMsg6, canMsg7;  
//  1-7 CAN frame

/*  Rotary Encoder (Will change Code to Motor Encoder Later) */
  #define CLK 4 // Input B
  #define DT 5 // Input A
  #define SW 3 // Reset Switch
  int counter = 0;
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
  Serial.begin (9600);
  Wire.begin(); // join I2C bus (IMu is I2C COM)

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /*  IMU init  */
    mpu.begin(); // initialize mpu instance
    Serial.println(F("Calculating gyro offset, so do not move MPU6050"));
    delay(500);
    mpu.calcOffsets(); // Calibrate both gyro and acc offset 
    Serial.println("Done: \n");

  /*  Encoder init  */
    // Set encoder pins as inputs
    pinMode(CLK,INPUT); pinMode(DT,INPUT); pinMode(SW, INPUT_PULLUP);

    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);
    attachInterrupt(digitalPinToInterrupt(SW), resetEncoder, FALLING); // (Switch is activelow in this case)

  /*  Set CAN Frame struct. */
  canMsg1.can_id  = 0x0F3; // 11 bit ID (standard CAN)
  canMsg2.can_id  = 0x0F4; 
  canMsg3.can_id  = 0x0F5; 
  canMsg4.can_id  = 0x0F6; 
  canMsg5.can_id  = 0x0F7; 
  canMsg6.can_id  = 0x0F8; 
  canMsg7.can_id  = 0x0F9; 
  canMsg1.can_dlc = standard_dlc;
  canMsg2.can_dlc = canMsg3.can_dlc = canMsg4.can_dlc = canMsg5.can_dlc = canMsg6.can_dlc = canMsg7.can_dlc = standard_dlc; 
} 

void loop() { 
  
  float* mpuData = readMPU();
  readEncoder();
  if(millis()-lasttime >= 100){
    Serial.print(currentDir);
    Serial.println(counter);

    // RPM 1st Frame (The direction is included in the last bit) (Though for optical Encoder this might not be possible)
    unsigned char* sendByte_RPM = Encode_bytearray(counter);  
    for(int i=0 ; i < standard_dlc  ; i++){
      canMsg1.data[i] = sendByte_RPM[i];
    }

    // Accel X 2nd Frame
    unsigned char* sendByte_Accelx = Encode_bytearray(mpuData[0]);  
    for(int i=0 ; i < standard_dlc ; i++){
      canMsg2.data[i] = sendByte_Accelx[i];
    }

    // Accel Y 3rd Frame
    unsigned char* sendByte_Accely = Encode_bytearray(mpuData[1]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg3.data[i] = sendByte_Accely[i];
    }


    // Accel Z 4th Frame
    unsigned char* sendByte_Accelz = Encode_bytearray(mpuData[2]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg4.data[i] = sendByte_Accelz[i];
    }

    // Gyro X 5th Frame
    unsigned char* sendByte_Gyrox = Encode_bytearray(mpuData[3]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg5.data[i] = sendByte_Gyrox[i];
    }

    // Gyro Y 6th Frame
    unsigned char* sendByte_Gyroy = Encode_bytearray(mpuData[4]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg6.data[i] = sendByte_Gyroy[i];
    }

    // Gyro Z 7th Frame
    unsigned char* sendByte_Gyroz = Encode_bytearray(mpuData[5]);  
    for(int i=0 ; i< standard_dlc  ; i++){
      canMsg7.data[i] = sendByte_Gyroz[i];
    }

    // Transmit CAN frame out of mcp2515 FIFO Buffer then transmit into CAN Bus
    // mcp2515.sendMessage(&canMsg1);  
    mcp2515.sendMessage(&canMsg2);  
    mcp2515.sendMessage(&canMsg3);  
    mcp2515.sendMessage(&canMsg4);  
    mcp2515.sendMessage(&canMsg5);  
    mcp2515.sendMessage(&canMsg6);  
    mcp2515.sendMessage(&canMsg7);    
    //Might need Extended CAN to cram more message
      // readVoltage();
      // readCurrent();

  
    // Serial.println("Messages sent");
    lasttime = millis(); // reset timer variable
  }
  
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





