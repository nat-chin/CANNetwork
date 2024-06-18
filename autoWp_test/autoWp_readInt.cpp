#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

volatile bool interrupt = false;
struct can_frame canMsg;
MCP2515 mcp2515(10);
// MCP2515 mcp2515(5);

void irqHandler() {
    interrupt = true;
}

void setup() {
    //
    attachInterrupt(digitalPinToInterrupt(2), irqHandler, FALLING);
}

void loop() {
    if (interrupt) {
        interrupt = false;

        uint8_t irq = mcp2515.getInterrupts();

        // Standard CAN message (in RX buffer 0)
        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &canMsg) == MCP2515::ERROR_OK) {
                // frame contains received from RXB0 message
                // The ID and DLC has 1 data each
                Serial.print(canMsg.can_id, HEX); // print ID in hex format
                Serial.print(" "); 
                Serial.print(canMsg.can_dlc, HEX); // print DLC in hex format
                Serial.print(" ");
                
                for (int i = 0; i<canMsg.can_dlc; i++)  {  // print the data in hex format also
                Serial.print(canMsg.data[i],HEX); // Printout the data packet that has been kept as array
                Serial.print(" ");
                }

                Serial.println();
            }
        }

        // Extended CAN message (In RX buffer 1)
        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &canMsg) == MCP2515::ERROR_OK) {
                // frame contains received from RXB1 message
            }
        }
    }
}