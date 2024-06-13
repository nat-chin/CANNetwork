// This node maybe change to PI pico instead of UNO R3

#include "Arduino.h"
#include <SPI.h>
#include <mcp2515.h>
struct can_frame canMsg1; // 1st CAN frame
struct can_frame canMsg2  // 2nd CAN frame
MCP2515 mcp2515(10); // Slave select at pin D10

/*  Motor Encoder (Optical) */
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

/* Voltage and Current Sense from Battery */
  #define AmpsPin A2
  #define VoltsPin A0

// --------------------------------------------------------//
float* readMPU();  
void readEncoder(); void resetEncoder(); 
float readCurrent(); float readVoltage();

 
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

    /*  Write CAN frame into the Buffer   */
    canMsg1.can_id  = 0x0F7; // 11 bit ID (standard CAN)
    canMsg1.can_dlc = 8; // 8 byte DLC (Standard CAN Maximum)
} 

void loop() { 
  
  float* mpuData = readMPU();
  readEncoder();
  if(millis()-lasttime >= 100){
  Serial.print(currentDir);
  Serial.println(counter);
  canMsg1.data[0] = mpuData[0];   // Accel X
  canMsg1.data[1] = mpuData[1];   // Accel Y
  canMsg1.data[2] = mpuData[2] ;  // Accel Z
  canMsg1.data[3] = mpuData[3];   // GyRo X
  canMsg1.data[4] = mpuData[4];   // GyRo Y
  canMsg1.data[5] = mpuData[5];   // GyRo Z
  canMsg1.data[6] = abs(counter); // Encoded step (Only positive value < 255 , since it is 1 byte )
  canMsg1.data[7] = currentDir;   // Encoding Direction

  canMsg2.data[0] = mpuData[0];   // Battery Nominal Voltage
  canMsg2.data[1] = mpuData[1];   // Discharge Current
  canMsg2.data[2] = mpuData[2] ;  // Temperature of Motor
  canMsg2.data[3] = mpuData[3];   // GyRo X
  canMsg2.data[4] = mpuData[4];   // GyRo Y
  canMsg2.data[5] = mpuData[5];   // GyRo Z
  canMsg2.data[6] = abs(counter);      // Encoded step (Only positive value < 255 , since it is 1 byte )
  canMsg2.data[7] = currentDir;   // Encoding Direction
  //Might need Extended CAN to cram more message
    // readVoltage();
    // readCurrent();

  // Transmit CAN frame out of mcp2515 FIFO Buffer
  mcp2515.sendMessage(&canMsg1);  
  // Serial.println("Messages sent");
  lasttime = millis(); // reset timer variable
  }
  
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


float readCurrent() {
  /*---------Current Calculation--------------*/
  float voltage_offset = 5000.0/2.0; // in mV Can be other value if we more voltage is applied in Series measurement
  float Vhall = 0.0; // Induced voltage from Hall effect sensor
  float mVperAmp = 100.0; // Sensitivity from sensor mV/A (20A model variant)
  float current = 0.0;

  Vhall = analogRead(AmpsPin)* 5000.0 / 1023.0; 
  current = ((Vhall - voltage_offset) / mVperAmp); 
  // The offset when Current sensor ACS712 sense no current is Vcc/2 , and the measured Vhall is with respect to that point , we need to subtract that out 
  return current;
}

float readVoltage() { 
  float Vsignal = 0.0; // Voltage from divider
  float Vin = 0.0; // Actual voltage measured , which is voltage across our divider circuit
  float dividerRatio = (7500.0/(30000.0+7500.0)); // Voltage divider Calculation (R2/R1+R2) unit ohms
  float ref_voltage = 5.0; // Default Analog Reference Voltage of Arduino UNO R3 (reference)

  // Convert the read value with the scale of 5/1023 since ADC produce digital voltage that is pulled from MCU power rail, not the measured voltage
  Vsignal = analogRead(VoltsPin) * (ref_voltage/ 1023.0); 
  // The read value will be 1/5 of the measured voltage, Revert back to original voltage we divide by the Divider ratio , or times 5
  Vin = Vsignal / dividerRatio; 
  return Vin;
}


