#include <Arduino.h>
#include <CAN.h>

#include "settings.h"

#define DEBUG false
#define TEST_DELAY 10  // ms

char* readBytesFromSerial();
void sendDataToOtherNodes(char* SerialData);

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    // Deal with LEDs
    pinMode(MAIN_STATUS_LED_PIN, OUTPUT);
    pinMode(CAN_TX_LED_PIN, OUTPUT);
    pinMode(CAN_RX_LED_PIN, OUTPUT);
    pinMode(IMU_CALIBRATION_LED_PIN, OUTPUT);
    pinMode(CAN_SYNC_LED_PIN, OUTPUT);

    digitalWrite(CAN_TX_LED_PIN, LOW);  // initial
    digitalWrite(CAN_RX_LED_PIN, LOW);  // not implemented
    digitalWrite(IMU_CALIBRATION_LED_PIN, LOW);  // not implemented
    digitalWrite(CAN_SYNC_LED_PIN, LOW);  // not implemented

    // This node does not handle any motors
    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    digitalWrite(PWM_PIN, LOW);  // not implemented
    digitalWrite(DIR_PIN, LOW);  // not implemented

    #if DEBUG
        Serial.println("Translator node");
    #endif

    // Initialize CAN
    CAN.setPins(CRX_PIN, CTX_PIN);

    bool statusLEDState = LOW;
    while (!CAN.begin(CAN_BAUD_RATE)) {
        #if DEBUG
            Serial.println("Starting CAN failed!");
        #endif

        delay(100);
        digitalWrite(MAIN_STATUS_LED_PIN, statusLEDState);
        statusLEDState = !statusLEDState;
    }

    #if DEBUG
        Serial.println("CAN init success");
        Serial.print("Expecting bytes: ");
        Serial.println(SERIAL_DATA_LENGTH);
    #endif

    digitalWrite(MAIN_STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(MAIN_STATUS_LED_PIN, LOW);
}

void loop() {
    char SerialData[SERIAL_DATA_LENGTH * 2];  // double length to prevent overflows
                                              // see the Serial.readBytesUntil() call for more details

    // justification to declare inside loop
    // https://stackoverflow.com/a/7959658/10907280

    // testNode();

    while (Serial.available()) {
        #if DEBUG
            Serial.println("Serial.available()");
        #endif

        // make sure that data is read until the null terminator
        // buffer length is double to ensure that some stray sequence of SERIAL_DATA_LENGTH is not read
        int dataSize = Serial.readBytesUntil('\n', SerialData, SERIAL_DATA_LENGTH * 2);

        #if DEBUG
            Serial.print("dataSize: ");
            Serial.println(dataSize);
        #endif

        if (dataSize == SERIAL_DATA_LENGTH) {
            digitalWrite(MAIN_STATUS_LED_PIN, HIGH);

            digitalWrite(CAN_TX_LED_PIN, HIGH);
            sendDataToOtherNodes(SerialData);
            digitalWrite(CAN_TX_LED_PIN, LOW);
        }
    }
}

void testNode() {
    byte data[2];

    for (size_t node = 0; node < TOTAL_NODE_COUNT; node++) {
        #if DEBUG
            Serial.print("Sending to node: ");
            Serial.println(FIRST_NODE_ADDRESS + node, HEX);
        #endif

        for (int direction = 0; direction < 2; direction++) {
            for (uint8_t speed = 0; speed < 255; speed++) {
                digitalWrite(CAN_TX_LED_PIN, HIGH);

                CAN.beginPacket(FIRST_NODE_ADDRESS + node);

                data[0] = speed;
                data[1] = direction;

                CAN.write(data, BYTES_PER_NODE);

                CAN.endPacket();

                digitalWrite(CAN_TX_LED_PIN, LOW);

                delay(TEST_DELAY);
            }

            for (uint8_t speed = 255; speed > 0; speed--) {
                digitalWrite(CAN_TX_LED_PIN, HIGH);

                CAN.beginPacket(FIRST_NODE_ADDRESS + node);

                data[0] = speed;
                data[1] = direction;

                CAN.write(data, BYTES_PER_NODE);

                CAN.endPacket();

                digitalWrite(CAN_TX_LED_PIN, LOW);

                delay(TEST_DELAY);
            }
        }

        #if DEBUG
            Serial.print("Sent to node: ");
            Serial.println(FIRST_NODE_ADDRESS + node, HEX);
        #endif
    }

    #if DEBUG
        Serial.println("Sent to all nodes");
    #endif
}

void sendDataToOtherNodes(char* SerialData) {
    #if DEBUG
        Serial.print("read: ");
        Serial.println(SerialData);
    #endif

    byte data[BYTES_PER_NODE];

    for (size_t node = 0; node < TOTAL_NODE_COUNT; node++) {
        digitalWrite(CAN_TX_LED_PIN, HIGH);

        #if DEBUG
            Serial.println("CTx loop");
        #endif

        CAN.beginPacket(FIRST_NODE_ADDRESS + node);
        #if DEBUG
            Serial.println("Packet begun");
        #endif

        data[0] = SerialData[node * 2];
        data[1] = SerialData[node * 2 + 1];

        CAN.write(data, BYTES_PER_NODE);
        #if DEBUG
            Serial.println("Packet written");
        #endif

        CAN.endPacket();
        #if DEBUG
            Serial.println("Packet ended");
        #endif

        digitalWrite(CAN_TX_LED_PIN, LOW);

        #if DEBUG
            Serial.print("Sent to node: ");
            Serial.println(FIRST_NODE_ADDRESS + node, HEX);
        #endif
    }

    #if DEBUG
        Serial.println("Sent to all nodes");
    #endif
}