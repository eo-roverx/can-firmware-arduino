#include <Arduino.h>
#include <CAN.h>

#include "settings.h"

#define DEBUG false

void readCAN(int packetSize);
void writeMotors(byte* data);
byte readDipSwitch();

void setup() {
    #if DEBUG
        Serial.begin(SERIAL_BAUD_RATE);

        while (!Serial) {
            delay(100);
        }

        Serial.print("Arm node ");
        Serial.println(readDipSwitch());
    #endif

    pinMode(ADDRESS_DIP_1_PIN, INPUT);
    pinMode(ADDRESS_DIP_2_PIN, INPUT);
    pinMode(ADDRESS_DIP_3_PIN, INPUT);
    pinMode(ADDRESS_DIP_4_PIN, INPUT);

    pinMode(PWM_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);

    digitalWrite(PWM_PIN, LOW);  // initial
    digitalWrite(DIR_PIN, LOW);  // initial

    pinMode(MAIN_STATUS_LED_PIN, OUTPUT);
    pinMode(CAN_SYNC_LED_PIN, OUTPUT);
    pinMode(CAN_RX_LED_PIN, OUTPUT);
    pinMode(CAN_TX_LED_PIN, OUTPUT);
    pinMode(IMU_CALIBRATION_LED_PIN, OUTPUT);

    digitalWrite(MAIN_STATUS_LED_PIN, LOW);      // initial
    digitalWrite(CAN_TX_LED_PIN, LOW);           // initial
    digitalWrite(CAN_RX_LED_PIN, LOW);           // initial
    digitalWrite(IMU_CALIBRATION_LED_PIN, LOW);  // not implemented
    digitalWrite(CAN_SYNC_LED_PIN, LOW);         // not implemented

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
        Serial.println("CAN Initialized successfully.");
    #endif

    // Blink twice to signal success and identify as an Arm node
    digitalWrite(MAIN_STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(MAIN_STATUS_LED_PIN, LOW);
    delay(100);
    digitalWrite(MAIN_STATUS_LED_PIN, HIGH);
    delay(100);
    digitalWrite(MAIN_STATUS_LED_PIN, LOW);

}

void loop() {
    int packetSize = CAN.parsePacket();

    readCAN(packetSize);
}

void readCAN(int packetSize) {
    static bool CAN_RX_LED_State = false;
    byte data[BYTES_PER_NODE];

    if (packetSize && (CAN.packetId() == readDipSwitch())) {
        digitalWrite(CAN_RX_LED_PIN, HIGH);
        digitalWrite(MAIN_STATUS_LED_PIN, HIGH);

        #if DEBUG
            Serial.println("Received packet for this node.");
        #endif

        while (CAN.available()) {
            CAN.readBytes(data, BYTES_PER_NODE);
        }

        writeMotors(data);

        digitalWrite(CAN_RX_LED_PIN, LOW);
    }
}

void writeMotors(byte* data) {
    int motorSpeed = data[0];
    int direction = data[1];

    analogWrite(PWM_PIN, motorSpeed);
    digitalWrite(DIR_PIN, direction);

    #if DEBUG
        Serial.print("Motor speed: ");
        Serial.print(motorSpeed);
        Serial.print(", direction: ");
        Serial.println(direction);
    #endif
}

byte readDipSwitch() {
    byte address = 0;

    address = digitalRead(ADDRESS_DIP_1_PIN);
    address <<= 1;
    address += digitalRead(ADDRESS_DIP_2_PIN);
    address <<= 1;
    address += digitalRead(ADDRESS_DIP_3_PIN);
    address <<= 1;
    address += digitalRead(ADDRESS_DIP_4_PIN);

    #if DEBUG
        Serial.print("Dip switch value: ");
        Serial.println(address, HEX);
    #endif

    return address;
}