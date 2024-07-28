//==================================================================================//
// #include "Wire.h"
#include <Arduino.h>
#include <CAN.h>
// #include <esp32-hal.h>
extern "C"{
  #include <driver/twai.h>
}

// Brownout issue
#include "soc/soc.h"
#include "soc/rtc.h"

/* WiFi , and Blynk credential setting */

#define BLYNK_TEMPLATE_ID "TMPL63Z1xA9KM"
#define BLYNK_TEMPLATE_NAME "CANnetwork"
#define BLYNK_AUTH_TOKEN "ZUTcXKGa403Gsy837POdZMgX5LrfyPh_"
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
const char* ssid     = "ton kong 2.4G";
const char* password = "022872225";
// Timer object to set Blynk Transmission rate
BlynkTimer timer; 

/* CAN Controller setting*/
#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
#define standard_bitrate 125E3
#define standard_delay 80
#define standard_dlc 4

//==================================================================================//

/* Function to Record all data to local SD card in CSV format (Optional) */

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


void myTimerEvent();

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
  Serial.begin (115200);
  // Serial1.begin (115200);

  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, password); // call blynk class method
  // Setup a timer function to be called every second => Change to 5 sec to ensure proper syncing

  // The problem is that Blynk run is time blocking , I need to make a non-blocking implementation of Blynk
  timer.setInterval(5000L, myTimerEvent);

  // // We can also specify a server domain with tcp port OR providing Device Local IP address instead
  // Blynk.begin(BLYNK_AUTH_TOKEN,ssid,password,"blynk.cloud",80);
  // Blynk.begin(BLYNK_AUTH_TOKEN,ssid,password,IPAddress(192,168,1,118),8080);

  while (!Serial) // halt if serial port is not available
    Serial.println ("CAN Receiver/Receiver");
  
  // Set the pins
  CAN.setPins (RX_GPIO_NUM, TX_GPIO_NUM);

  // start the CAN bus at 500 kbps , E3 indicates 10^3 or kilos
  if (!CAN.begin (standard_bitrate)) {
    Serial.println ("Starting CAN failed!");
    while (true);
  }
  else { Serial.println ("CAN Initialized"); }

  // TWAI Dual Filter (How to activate???)
  // Okay let's use single filter for now  (32 bit Acceptance)
  CAN.filter(0x10,0x10); // 0x10-0x1F

  delay(30); // Delay for all Nodes to boot , (Not Exceeding standard delay on Sender Side) ,
  Serial.print ("Sending Trigger to start the System ... ");
  CAN.beginPacket (0x01, 1, false); CAN.write(1); CAN.endPacket();
  Serial.println ("done");
}

const short canMsgNum = 10;  // Number of arrays buffer correspond to Blynk Virtual Pin
const short candlc = 4;  // Assuming canMsg.can_dlc is 8
float blynkdata[canMsgNum];  // Array to hold a float data
uint8_t canMsgCount = 0; // incrementation mechanism , will explain later
unsigned long lasttime = 0;
unsigned char message[4];
volatile bool doneflag = false;
// // After Blynk is connected do the state syncing
// BLYNK_CONNECTED() { Blynk.syncAll(); // Request Blynk Server to resend the last known data state
// }

// //here are handlers for the sync command
// BLYNK_WRITE(V0) {
//   float value = param.asFloat(); 
//   blynkdata[0] = value;
// }
// BLYNK_WRITE(V1) {
//   float value = param.asFloat(); 
//   blynkdata[1] = value;
//   }
// BLYNK_WRITE(V2) {
//   float value = param.asFloat(); 
//   blynkdata[2] = value;
//   }
// BLYNK_WRITE(V3) {
//   float value = param.asFloat(); 
//   blynkdata[4] = value;
//   }
// BLYNK_WRITE(V4) {
//   float value = param.asFloat(); 
//   blynkdata[5] = value;
//   }

// Disconnect Blynk if the following situation happened
// Use this in somekind of timeout condition to disconnect blynk from device
// Blynk.disconnect();

// // Blynk API code , get data from CAN msg array shove into virtual pin , use this as void function , then pass them into timer
void myTimerEvent(){
  Blynk.virtualWrite(V0, blynkdata[0]);
  Blynk.virtualWrite(V1, blynkdata[1]);
  Blynk.virtualWrite(V2, blynkdata[2]);
  Blynk.virtualWrite(V3, blynkdata[3]);
  Blynk.virtualWrite(V4, blynkdata[4]);
  // Should remake blynkData , it shouldn't be 2D array, I can just decode to float and append it not just individual byyr
}

void canlogger(){


}

void loop() {
  
  
  if(millis()-lasttime >= standard_delay) { // 80 millis delay for 10 messages
    // Polling from RX Buffer [Better to use Receive Interrupt, but this will suffice , since bus load isn't high]
    if (CAN.parsePacket()) {
      
      uint8_t canMsgID = CAN.packetId();
      uint8_t canMsgDlc = CAN.packetDlc();
      uint8_t canMsgbuf[canMsgDlc];
      // Display Message Content
      Serial.print ("Packets of id 0x"); Serial.print (canMsgID, HEX);
      Serial.print (" of length "); Serial.println (canMsgDlc);

      /* Only Sample to data that isn't a done flag (Sample data to Buffer Before Blynk) : 0x1F is TX Done Flag */
      if(canMsgID != 0x1F){
        // Read CAN data field Byte by Byte after parsing the message from RX buffer
        int i = 0; while (CAN.available()){ 
          canMsgbuf[i] = CAN.read(); // Read , and Clear Rx buffer
          i++; }

        /*Decode Message back to 4 byte float */
        float receiveFloat = Decode_bytearray(canMsgbuf);
        blynkdata[canMsgCount] = receiveFloat;
        Serial.print("Data for Each Blynk Virtual Pin: "); 
        Serial.println(blynkdata[canMsgCount],7); 
        canMsgCount = (canMsgCount + 1) % canMsgNum; // Increment counter for next Receive interval, reset upon meeting maximum count

      } 
      else if(canMsgID == 0x1F) {
        /* Send packet of ID 0x01 to signal all Non-Head unit Node to Starts streaming Sensor Data */

        // 1. Read from RX buffer when ID = 0x1F
        int i = 0; while (CAN.available()){ 
        canMsgbuf[i] = CAN.read(); 
        Serial.print((char)canMsgbuf[i]);
        i++; } Serial.println();

        // Message 'A' mean NodeA finished Transmission, Trigger NodeB
        if(canMsgbuf[0] == 'A'){ 
          CAN.beginPacket(0x02,1,false); CAN.write(1); CAN.endPacket(); // TrgMsg ID 0x02 : Node B
          Serial.println("Done Flag Detected A");
        }
        // Message B mean NodeB finished Transmission , Trigger NodeA 
        else if(canMsgbuf[0] == 'B' ){ 
          CAN.beginPacket(0x01,1,false);CAN.write(1); CAN.endPacket(); // TrgMsg ID 0x01 : Node A
          Serial.println("Done Flag Detected B");
        }

      }
    }
    lasttime = millis();
  }
  Blynk.run();
  if(!Blynk.connected())
    Serial.println("DISCONNECTED");
  timer.run();
}
  

//==================================================================================//







