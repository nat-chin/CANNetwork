//==================================================================================//
// #include "Wire.h"
#include <Arduino.h>
#include <CAN.h>
// #include <esp32-hal.h>
extern "C"{
  #include <driver/twai.h>
}


#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
#define standard_bitrate 125E3
#define standard_delay 100
#define standard_dlc 4
unsigned long lasttime =0;

//==================================================================================//
void canSender(); void canReceiver();
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);
float readCurrent(); float readVoltage();

/* Voltage and Current Sense from Battery */
  #define AmpsPin A11
  #define VoltsPin A0

void setup() {

  Serial.begin (115200);
  // Serial1.begin (115200);
  // Serial2.begin (115200);


  
  while (!Serial); // halt if serial port is not available
  Serial.println ("CAN Receiver/Receiver");
  delay(500);
  
  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps , E3 indicates 10^3 or kilos
  if (!CAN.begin (standard_bitrate)) {
    Serial.println ("Starting CAN failed!");
    while (true);
  }
  else {
    Serial.println ("CAN Initialized");
  }

    // CAN.end();
    // // Configure bit timing parameters
    // twai_timing_config_t t_config = TWAI_TIMING_CONFIG_125KBITS();
    // // Adjust timing configuration if necessary
    // t_config.brp = 32;    // Example value for Baud rate prescaler
    // t_config.tseg_1 = 15; // Example value for Time segment 1
    // t_config.tseg_2 = 4;  // Example value for Time segment 2
    // t_config.sjw = 3;     // Example value for Synchronization jump width

    // twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, TWAI_MODE_NORMAL);
    // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    //     Serial.println("TWAI driver installed");
    // } else {
    //     Serial.println("Failed to install TWAI driver");
    // }

    // if (twai_start() == ESP_OK) {
    //     Serial.println("TWAI driver started");
    // } else {
    //     Serial.println("Failed to start TWAI driver");
    // }



}

// Receiving & Record Data to Local SD Card

unsigned char message[4];

void loop() {
  
  if(millis()-lasttime >= standard_delay){
    canReceiver(); // Read from RX buffer Per Iteration  (Now trying to do multiple transmission)

    /* Bit arbritation Order (UNO)
    From GPS Node
    Lat : 0xF1
    Lng : 0xF2 

    From Motor Controller Node
    RPM : 0xF3
    Accelx  : 0xF4
    Accely  : 0xF5
    AccelZ  : 0xF6
    GyroX   : 0xF7
    GyroY   : 0xF8
    GyroZ   : 0xF9

    From This head unit
    Monitor : Nominal Voltage of Battery the voltage (If possible Calculate for SOC and SOH )
              Current (Discharge) of Battery to Motor

    So the order of received message in 500 Kbps (with 100ms timer delay) (Sampling Rate) is 
    Lat Lng RPM Accelx , Accely , Accelz , Gyro x , Gyro y , Gyro z
    */

    /*Decode Message back to 4 byte float */
    float receiveFloat = Decode_bytearray(message);
    Serial.println(receiveFloat,7);

    /* Confirm 4 bytes message each CAN frame*/
    for (int i = 0; i < 4; i++) {
      Serial.print(message[3-i]);
      Serial.print(',');
    } Serial.println();

    for (int i = 0; i < 4; i++) {
      Serial.print(message[3-i], HEX);
      Serial.print(',');
    } Serial.println();

    // Serial.println();


    /* Acknowledgement Frame */
    // canSender();


    /* Function to Record all data to local SD card in CSV format (Optional) */

    lasttime = millis();
  }
 
}
/* Sample of Received Bit */


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


//==================================================================================//

void canReceiver() {
  int packetSize = CAN.parsePacket(); // Parse packet

  // received a packet
  if (packetSize > 0) {
    Serial.print ("Received ");
    // Extended CAN ID check
    if (CAN.packetExtended()) { Serial.print ("extended ");}
    
    /* Check Standard CAN ID */
    Serial.print ("packet with id 0x");
    Serial.print (CAN.packetId(), HEX);
    
    // RTR check (if RTR , then RTR bit is 1)
    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print ("RTR ");
      Serial.print (" with requested length ");
      Serial.println (CAN.packetDlc());
    } else {

      /* Print out NON-RTR packet  (read Each byte)*/
      Serial.print (" Of length: ");
      Serial.println (packetSize);

      int i = 0; // set iteration at zero
      while (CAN.available()) {
        // This system BUS is only 4 byte data , so 4 iteration
        // Serial.print(CAN.read()); 
        message[i] = CAN.read();  
        i++;
        } 
      // Serial.println();
    }
  } 
}

// Sending Acknowledgement Bit (??)
void canSender() {
  Serial.print ("Sending packet ... ");
  // CAN.beginPacket (0x12);  //sets the ID and clears the transmit buffer
  // // CAN.beginExtendedPacket(0xabcdef);
  // CAN.write ('1'); //write data to buffer. data is not sent until endPacket() is called.
  // CAN.endPacket();

  //RTR packet with a requested data length
  // RTR sends empty packet and request some data length back
  CAN.beginPacket (0x12, 0, true);
  CAN.endPacket();

  Serial.println ("done");

  // delay(1000);
}

//==================================================================================//




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
