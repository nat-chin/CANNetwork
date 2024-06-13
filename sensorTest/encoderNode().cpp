/*  Rotary Encoder Define */

  // Rotary Encoder Inputs
  #define inputSW 3
  #define inputCLK 4
  #define inputDT 5

  int counter = 0; 
  int currentStateCLK; int previousStateCLK; 
  char encdir;


/*  IMU define  */
  #include "Wire.h"
  #include <MPU6050_light.h>
  MPU6050 mpu(Wire); unsigned long lasttime = 0;

/*  mcp2515 define  */
  #include <SPI.h>
  #include <mcp2515.h>
  struct can_frame canMsg1; // init can frame structure
  MCP2515 mcp2515(10); // Clock select at Digital pin 10

// --------------------------------------------------------//

// Function Definition
void readIMU(); void readEncoder();void resetEncoder();
 

void setup() { 
  Serial.begin (9600);
  Wire.begin(); // join I2C bus

  /*  CAN frame prep  */
    canMsg1.can_id  = 0x0F7; // 11 bit ID (standard CAN)
    canMsg1.can_dlc = 8; // 8 byte DLC

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(CAN_500KBPS); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /*  IMU init  */
    mpu.begin(); // initialize mpu instance
    
    Serial.println(F("Calculating gyro offset, so do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(); // Calibrate both gyro and acc offset 
    Serial.println("Done: \n");

  /*  Encoder init  */
     
    pinMode (inputCLK,INPUT);
    pinMode (inputDT,INPUT);
    
    // Read the initial state of inputCLK , then Assign to previousStateCLK variable
    previousStateCLK = digitalRead(inputCLK);
    attachInterrupt(digitalPinToInterrupt(inputSW), resetEncoder, FALLING);
} 

void loop() { 

  if(millis()-lasttime > 100){
  readIMU();
  readEncoder();
  mcp2515.sendMessage(&canMsg1);
  // Pass the address to this method as a way to pass array argument 
  Serial.println("Messages sent");
  lasttime = millis(); // reset timer variable
  }
  
}

void readIMU(){
  mpu.update(); // update the sensor register with new value 
  canMsg1.data[0] = mpu.getAccX(); // Accel X
  canMsg1.data[1] = mpu.getAccY(); // Accel Y
  canMsg1.data[2] = mpu.getAccZ(); // Accel Z
  canMsg1.data[3] = mpu.getAngleX(); // GyRo X
  canMsg1.data[4] = mpu.getAngleY(); // GyRo Y
  canMsg1.data[5] = mpu.getAngleZ(); // GyRo Z
}

void readEncoder(){
  //CLK = OUTPUT A
  //DT = OUTPUT B
  // We use outputA as reference

  // Read the current state of inputCLK
   currentStateCLK = digitalRead(inputCLK);
    
   // If the previous and the current state of the inputCLK are different then a pulse has occured
   if (currentStateCLK != previousStateCLK){ 
       
     // If the inputDT state is different than the inputCLK state then the encoder is rotating clockwise
     if (digitalRead(inputDT) != currentStateCLK) { 
       counter ++;
       encdir = '+';
       
     } else if(digitalRead(inputDT) == currentStateCLK) {
       // Encoder is rotating counter clockwise
       counter --;
       encdir ='-';
     }
   } 
   // Update previousStateCLK with the current state
   previousStateCLK = currentStateCLK; 
  canMsg1.data[6] = counter ; // Encoder increment
  canMsg1.data[7] = encdir; // Encoder direction
}

void resetEncoder(){
      counter = 0 ;
      encdir = 'R';
}


