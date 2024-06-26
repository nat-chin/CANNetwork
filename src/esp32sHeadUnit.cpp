//==================================================================================//
// #include "Wire.h"
#include <Arduino.h>
#include <CAN.h>
// #include <esp32-hal.h>
extern "C"{
  #include <driver/twai.h>
  // #include <driver/gpio.h>
  // #include "soc/clk_tree_defs.h"
}

#define TX_GPIO_NUM   23  // Connects to CTX
#define RX_GPIO_NUM   22  // Connects to CRX
#define standard_bitrate 125E3
#define standard_delay 90
#define standard_dlc 4
unsigned long lasttime =0;

//==================================================================================//
volatile bool interrupt = false;


void canSender(); void canReceiver();
unsigned char *Encode_bytearray(float f); float Decode_bytearray(unsigned char* c);
float readCurrent(); float readVoltage();


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

  // TWAI Dual Filter (How to activate???)
  // 0x10 - 0x17 (First Seven ID) , Mask only two 1st bit as (10)000 ,  0x18 = 11000
  // CAN.filter(0x10,0x18);
  // 0x10-0x19 (Last 2 ID) , Mask only 4 1st bit (1100)0 , 0x1E = 11110 
  // CAN.filter(2,0x18,0x1E);

  // Okay let's use single filter for now , (Allow all 16 ID range (Whole HEX digit))
  // In reality I need only 10 ID range
  CAN.filter(0x10,0x10);

  // Enable The All interrupt source for proper Error Handling ( TX , RX Interrupt is The most essential )
  //  uint32_t alerts_to_enable = TWAI_ALERT_TX_IDLE           // Transmit buffer is idle
  //                         | TWAI_ALERT_TX_SUCCESS        // Transmission successfully completed
  //                         | TWAI_ALERT_RX_DATA           // Message received and added to RX queue
  //                         | TWAI_ALERT_BELOW_ERR_WARN    // Error warning limit has been reached
  //                         | TWAI_ALERT_ERR_ACTIVE        // TWAI controller has become error active
  //                         | TWAI_ALERT_ERR_PASS          // TWAI controller has become error passive
  //                         | TWAI_ALERT_BUS_ERROR         // Bus error has occurred
  //                         | TWAI_ALERT_BUS_OFF           // Bus-off condition occurred
  //                         | TWAI_ALERT_RX_QUEUE_FULL     // RX queue is full
  //                         | TWAI_ALERT_ARB_LOST          // Arbitration was lost
  //                         | TWAI_ALERT_RECOVERY_IN_PROGRESS // Bus recovery in progress
  //                         | TWAI_ALERT_BUS_RECOVERED     // Bus has been recovered
  //                         | TWAI_ALERT_AND_LOG;          // Enables logging of alerts
  //   if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
  //       printf("TWAI alerts enabled\n");
  //   } else {
  //       printf("Failed to enable TWAI alerts\n");
  //       return;
  //   }

    // attachInterrupt(digitalPinToInterrupt(), IO_INT_ISR, FALLING); // (Interrupt pin being pull down after event detect)
    // what is interrupt pin of twai wth

  canNodeTrigger(0x01); // Initiate all CAN bus Node with 1st Trigger
}

unsigned char message[4];
volatile bool doneflag = false;
void loop() {

  // twai_message_t message;
  // uint32_t alerts_triggered;

  //   // Check for triggered alerts
  //   if (twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(100)) == ESP_OK) {
  //       if (alerts_triggered & TWAI_ALERT_RX_DATA) {
  //           // Message received
  //           if (twai_receive(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
  //               // Process the received message
  //               printf("Message received: ID 0x%lX, DLC %d, Data: ", message.identifier, message.data_length_code);
  //               for (int i = 0; i < message.data_length_code; i++) {
  //                   printf("0x%02X ", message.data[i]);
  //               }
  //               printf("\n");
  //           }
  //       }
  //       if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
  //           printf("TWAI controller entered error passive state\n");
  //       }
  //       if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
  //           printf("TWAI bus error occurred\n");
  //       }
  //   }
  
  if(millis()-lasttime >= standard_delay){ // 90 millis delay for 10 messages
    canReceiver(); // Read data from RX buffer Per Iteration (Per message transmission delay)

    /*Decode Message back to 4 byte float */
    float receiveFloat = Decode_bytearray(message);
    Serial.println(receiveFloat,7);

    /* Confirm 4 bytes message each CAN frame*/
    for (int i = 0; i < 4; i++) {
      Serial.print(message[3-i],DEC);
      Serial.print(',');
    } Serial.println();

    for (int i = 0; i < 4; i++) {
      Serial.print(message[3-i], HEX);
      Serial.print(',');
    } Serial.println();

    lasttime = millis();
  }
  // Send packet of ID 0x01 to signal all Non-Head unit Node to Starts streaming Sensor Data
  // Trigger Non Header CAN node
  // Two 1st message will be   
  // This will always read from buffer and clear a buffer !!! (Shouldn't be)
  if(millis()-lasttime >= standard_delay + 10){
    canReceiver(); // Read the last data (Done flag sent by every Node After Their Transmission is done)
    if(message[0] = 'A') // Message A mean NodeA is Finsihed, Trigger NodeB
      canNodeTrigger(0x02);
    else if(message[0] = 'B') // Message B mean The opposite
      canNodeTrigger(0x01);
    lasttime = millis();
  }
}
/* Sample of Received Bit */
// Blynk API code


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
  // else{ Serial.println("NO Packet");}
}

void canNodeTrigger(uint16_t ID) {
  Serial.print ("Sending Trigger to start the System ... ");
  CAN.beginPacket (ID, 1, false);
  CAN.write('T');
  CAN.endPacket();
  Serial.println ("done");
}

//==================================================================================//





