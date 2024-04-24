#include <Arduino.h>
#include <SPI.h>
#include <mcp2515.h>

volatile bool interrupt = false;
struct can_frame frame;
MCP2515 mcp2515(10);
// MCP2515 mcp2515(5);

void irqHandler() {
    interrupt = true;
}

void setup() {
    //
    attachInterrupt(0, irqHandler, FALLING);
}

void loop() {
    if (interrupt) {
        interrupt = false;

        uint8_t irq = mcp2515.getInterrupts();

        if (irq & MCP2515::CANINTF_RX0IF) {
            if (mcp2515.readMessage(MCP2515::RXB0, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB0 message
            }
        }

        if (irq & MCP2515::CANINTF_RX1IF) {
            if (mcp2515.readMessage(MCP2515::RXB1, &frame) == MCP2515::ERROR_OK) {
                // frame contains received from RXB1 message
            }
        }
    }
}