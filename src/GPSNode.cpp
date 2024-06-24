#include <ArduinoSTL.h> // C standard Libs

#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int TXPin = 3, RXPin = 4;
static const uint32_t GPSBaud = 9600;

// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

/* mcp2515 init */
  #include <SPI.h>
  #include <mcp2515.h> 
  #define standard_bitrate CAN_125KBPS
  #define standard_delay 100
  #define standard_dlc 4
  MCP2515 mcp2515(10, MCP_8MHZ);
  struct can_frame canMsg1, canMsg2, canMsg3 ,canMsg4,canMsg5; // CAN frame format

/*  Rotary Encoder (Will change Code to Motor Encoder Later) */
  #define CLK 4 // Input B
  #define DT 5 // Input A
  #define SW 3 // Reset Switch
  float counter = 0;
  int currentStateCLK; int lastStateCLK;
  char currentDir = ' '; // Consider CW as +

  // /* Voltage and Current Sense from Battery */
//   #define AmpsPin A11
//   #define VoltsPin A0

  volatile bool interrupt = false;
void resetEncoder(){
      counter = 0 ;
      currentDir = ' ';
}

void ISR_CAN(){
  interrupt = true;
 }

static void smartDelay(unsigned long ms);
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);

void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);

  /*  Encoder init  */
    // Set encoder pins as inputs
    pinMode(CLK,INPUT); pinMode(DT,INPUT); pinMode(SW, INPUT_PULLUP);
    // Read the initial state of CLK
    lastStateCLK = digitalRead(CLK);

  /*  mcp2515 init  */
    while (!Serial); // halt communication if Uart Serial port isn't available
    mcp2515.reset();
    mcp2515.setBitrate(standard_bitrate); //Set Bit rate to 500KBPS (Need to match with target device)
    mcp2515.setNormalMode();
  /* Set CAN Frame struct.*/
    canMsg1.can_id  = 0x10; // 11 bit identifier(standard CAN)
    canMsg2.can_id  = 0x11; 
    canMsg3.can_id  = 0x12; 
    canMsg4.can_id  = 0x13;
    canMsg5.can_id  = 0x14;  
    canMsg1.can_dlc = canMsg2.can_dlc = canMsg3.can_dlc = canMsg4 = standard_dlc;

  //Set Mask and Filter Receiving of ID to receive only from Head unit , using only RXB0 
    // mcp2515.setFilter(MCP2515::RXF0, false, 0x01); // Filter for RXB0 , accept ID 0x01 (Head Unit ID)

  attachInterrupt(digitalPinToInterrupt(SW), resetEncoder, FALLING); // (Switch is activelow in this case)
  attachInterrupt(digitalPinToInterrupt(2), ISR_CAN, FALLING); // (Interrupt pin being driven low when fired)
}


void loop() {
  readEncoder();
  smartDelay(standard_delay); // 100 ms delay for feeding gps object (This delay doesn't affect MCU operation)
  unsigned char *sendByteRPM = Encode_bytearray(lat);
  
  // RPM 1st Frame
  for(int i = 0; i< standard_dlc; i++){
      canMsg1.data[i] = sendByteLat[i];
    }

  // unsigned char *sendByteVolt = Encode_bytearray();
  // unsigned char *sendByteAmp = Encode_bytearray();
  
    // Longitude (2nd frame)
  if(gps.location.isValid()){

    // Encode integer and floating point of both lat long , 4 byte each , precision point of 1e7
    float lat = gps.location.lat();
    float lng = gps.location.lng();
    
    unsigned char *sendByteLat = Encode_bytearray(lat);
    unsigned char *sendByteLng = Encode_bytearray(lng);
    
    // /* Display result  */
    //   Serial.print("Latitude (Deg.): ");
    //   Serial.println(lat,7);
    //   for(int i = 0; i < standard_dlc; i++){
    //     Serial.print(sendByteLat[3-i]);
    //     Serial.print(',');
    //   } 
    //   Serial.println();

    //   for(int i = 0; i < standard_dlc; i++){
    //     Serial.print(sendByteLat[3-i],HEX);
    //     Serial.print(',');
    //   } 
    //   Serial.println();
 
    // Latitude (1st frame)
    for(int i = 0; i< standard_dlc; i++){
      canMsg4.data[i] = sendByteLat[i];
    }
    // Longitude (2nd frame)
    for(int i = 0; i< standard_dlc; i++){
      canMsg5.data[i] = sendByteLng[i];
    }

    // RPM 3rd Frame

    // Voltage

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

      

  if (errorFlags & MCP2515::EFLG_RX0OVR)
      Serial.println("RXB0 overflow error");
  if (errorFlags & MCP2515::EFLG_RX1OVR)
      Serial.println("RXB1 overflow error");
  if (errorFlags & MCP2515::EFLG_TXBO)
      Serial.println("Bus-off error");

    // Transmit CAN frame out of mcp2515 FIFO Buffer then transmit into CAN Bus
    // mcp2515.sendMessage(&canMsg1);
    // mcp2515.sendMessage(&canMsg2);   

    // May add some Acknowledgement functionality
    /*
    
    */
    
  }

  // Failure GPS will print the following
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
    // Error Handing of CAN frame

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

/* Custom BlinkWithout Delay Code that ensures the gps object is being fed by UART reading on time. and contain read function itself */
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();

  // Execute command in do{} while milis-start < input ms. The condition is deemed to be false after milis and start difference grow past ms
  do {
    // if available, read NMEA and encode to TinyGPS+ instance for fine format.
    while (ss.available()) 
      gps.encode(ss.read());
  } 
  while (millis() - start < ms);
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

// float readCurrent() {
//   /*---------Current Calculation--------------*/
//   float voltage_offset = 5000.0/2.0; // in mV Can be other value if we more voltage is applied in Series measurement
//   float Vhall = 0.0; // Induced voltage from Hall effect sensor
//   float mVperAmp = 100.0; // Sensitivity from sensor mV/A (20A model variant)
//   float current = 0.0;

//   Vhall = analogRead(AmpsPin)* 5000.0 / 1023.0; 
//   current = ((Vhall - voltage_offset) / mVperAmp); 
//   // The offset when Current sensor ACS712 sense no current is Vcc/2 , and the measured Vhall is with respect to that point , we need to subtract that out 
//   return current;
// }

// float readVoltage() { 
//   float Vsignal = 0.0; // Voltage from divider
//   float Vin = 0.0; // Actual voltage measured , which is voltage across our divider circuit
//   float dividerRatio = (7500.0/(30000.0+7500.0)); // Voltage divider Calculation (R2/R1+R2) unit ohms
//   float ref_voltage = 5.0; // Default Analog Reference Voltage of Arduino UNO R3 (reference)

//   // Convert the read value with the scale of 5/1023 since ADC produce digital voltage that is pulled from MCU power rail, not the measured voltage
//   Vsignal = analogRead(VoltsPin) * (ref_voltage/ 1023.0); 
//   // The read value will be 1/5 of the measured voltage, Revert back to original voltage we divide by the Divider ratio , or times 5
//   Vin = Vsignal / dividerRatio; 
//   return Vin;
// }
