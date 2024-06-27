#include <ArduinoSTL.h> // C standard Libs , already include Arduino.h

/* GPS init */
  #include <TinyGPSPlus.h>
  #include <SoftwareSerial.h>
  static const int TXPin = 6, RXPin = 7;
  TinyGPSPlus gps;
  // The serial connection to the GPS device
  SoftwareSerial ss(RXPin, TXPin);

/* mcp2515 init */
  #include <SPI.h>
  #include <mcp2515.h> 
  #define standard_bitrate CAN_125KBPS
  #define standard_delay 80
  #define standard_dlc 4
  MCP2515 mcp2515(10);
  struct can_frame canMsg1, canMsg2, canMsg3 ,canMsg4, doneMsg; ; // CAN frame Transmit to Headunit

  #define AcceptedTriggerID 0x01
  struct can_frame trgMsg; // CAN frame Received From Headunit
  
/*  Rotary Encoder (Will change Code to Motor Encoder Later) */
  #define CLK 4 // Input B
  #define DT 5 // Input A
  #define SW 3 // Reset Switch
  float counter = 0;
  int currentStateCLK; int lastStateCLK;
  char currentDir = ' '; // Consider CW as +

/* Current Drawn to Motor */
  #define AmpsPin A0


float readCurrent(); void readEncoder(); void resetEncoder();
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);
void mcp2515Error();

void setup() {
  Serial.begin(115200); ss.begin(9600);
  pinMode(LED_BUILTIN,OUTPUT); digitalWrite(LED_BUILTIN,0);

  /*  Encoder init  */
    // Set encoder pins as inputs
    pinMode(CLK,INPUT); pinMode(DT,INPUT); pinMode(SW, INPUT_PULLUP);
    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);
    attachInterrupt(digitalPinToInterrupt(SW), resetEncoder, FALLING); // (Switch is activelow in this case)

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(standard_bitrate,MCP_16MHZ); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();

  /* Set CAN Frame struct.*/
    canMsg1.can_id  = 0x10; canMsg2.can_id  = 0x11; 
    canMsg3.can_id  = 0x12; canMsg4.can_id  = 0x13; 
    canMsg1.can_dlc = canMsg2.can_dlc = canMsg3.can_dlc = canMsg4.can_dlc = standard_dlc;

    
  
  //Set Mask and Filter Receiving of ID to receive only from Head unit , using only RXB0 
  // mcp2515.setFilter(MCP2515::RXF0, false, 0x01); // Filter for RXB0 , accept ID 0x01 (Head Unit ID)
}

// CAN message Scheduling
// volatile bool trigger = false;   
volatile uint8_t msg_counter = 0; // will be set to one after the 1st trigger 
unsigned long last_msg_time = 0; // get this out the loop
volatile bool doneFlag = false;

void loop() {
  // Error Detection Code (If Error is not showing , put this function after every time we Write To TX buffer)
  mcp2515Error();
  // Polling for Trigger from ESP32 Head unit (Better to use Interrupt , but INT pin is damaged rn.)
  if (mcp2515.readMessage(&trgMsg) == MCP2515::ERROR_OK) { 
    // Manual Filter , since Acceptance Filter doesn't work
    if(trgMsg.can_id == AcceptedTriggerID){
      Serial.print(trgMsg.can_id);
      msg_counter = 1; 
      Serial.println("Node A Triggered");} 
  }


  /* Read Motor RPM */
    readEncoder(); 
    // float motorcurrent = readCurrent();
    float motorcurrent = 0.0;
    // RPM 1st Frame
    uint8_t *sendByteRPM = Encode_bytearray(counter);
    for(int i = 0; i< standard_dlc; i++){
        canMsg1.data[i] = sendByteRPM[i]; 
        // Serial.print(sendByteRPM[i]);
        } 
        // Serial.println();

    // Motor Amp 2nd Frame
    uint8_t *sendByteAmp = Encode_bytearray(motorcurrent);
    for(int i = 0; i< standard_dlc; i++){
        canMsg2.data[i] = sendByteAmp[i];}
  
  /* Read GPS Data */
    while (ss.available()) 
      gps.encode(ss.read());
    if(gps.location.isValid()){
      // Encode integer and floating point of both lat long , 4 byte each , precision point of 1e7
      float lat = gps.location.lat();
      float lng = gps.location.lng();
      
      // Latitude (3rd frame)
      uint8_t *sendByteLat = Encode_bytearray(lat);
      for(int i = 0; i< standard_dlc; i++){
        canMsg3.data[i] = sendByteLat[i]; }
      // Longitude (4th frame)
      uint8_t *sendByteLng = Encode_bytearray(lng);
      for(int i = 0; i< standard_dlc; i++){
        canMsg4.data[i] = sendByteLng[i]; }
    }
    // Failure GPS will print the following (5 ms of not Receiving the GPS)
    if (millis() > 5000 && gps.charsProcessed() < 10) {
      Serial.println(F("No GPS data received: check wiring"));
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN)); // Blinking
    }

  /* Write Message to TX buffer then let protocol Engine transmit into CAN Bus with Proper Bit timing */
    
  // sending 1st message
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
    msg_counter = 0; // Reset Back to Bus Off state waiting for Head unit to allow Next Msg.
    doneFlag = true;
    last_msg_time = millis();
  }

  /* sending Done Flag to signal Head unit to sent The trigger again */
  if ((doneFlag == true) && (millis() - last_msg_time >= standard_delay)) {
    // Msg to Signal HeadUnit When all Packet is successfully sent.
    doneMsg.can_id  = 0x1F; doneMsg.can_dlc = 1; doneMsg.data[0] = 'A';
    mcp2515.sendMessage(&doneMsg);
    Serial.println("Done Flag A Sent");
    doneFlag = false;
    last_msg_time = millis();
  }
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
      counter--; 
			currentDir ='-';
		}
	}
	// Remember last CLK state
	lastStateCLK = currentStateCLK;
} 
void resetEncoder(){ counter = 0 ; currentDir = ' ';}

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
