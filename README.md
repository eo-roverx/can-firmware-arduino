# CAN Node Arduino Firmware

This is the firmware for the CAN Nodes used on the robotic arm. It is designed for our [custom PCBs](https://github.com/eo-roverx/can-node), powered by an ESP32-WROOM-32.

This is a part of the new arm system, wherein each PCB drives one motor, provides it feedback control, and (if needed) gives back IMU and encoder data. Our CAN node will have the following features:

- CAN bus communication
- IMU (accelerometer, gyroscope)
- Motor encoder input
- Motor driver
- 4 user defined LEDs + one status LED
- 4 bit address DIP switch
- ESP32 microcontroller: integration and feedback control

## Translator Node
The translator code should be uploaded to the Translator node, which is responsible for converting Serial messages from the SBC into CAN messages for the other nodes. This is **not** mounted on the arm, but rather on the base of the robot.

Make sure to run the serial monitor at 115200 baud; this seems to work best with the ESP32.

## Arm Node
The arm node code should be uploaded to all Arm nodes, which is responsible for controlling the arm. This is mounted on the arm itself, and each node handles one motor.

Make sure to set the address of each node using its dip switches.

## CAN Library
Make sure to install [sandeepmistry's](https://github.com/sandeepmistry) CAN library in the Arduino IDE. https://github.com/sandeepmistry/arduino-CAN/

## Serial data format
The data is sent as a simple byte array, terminated by a newline character. Each pair of bytes is the data for one motor. The first byte is the motor speed, and the second byte is the motor direction. The speed is a value from 0 to 255, and the direction is either 0 or 1. The first pair of bytes is for the first motor, the second pair is for the second motor, and so on.

By "byte", we mean that the following. If the value `67` is to be sent, it will be sent as one byte with the value `67` (or `'C'`), not two bytes of `'6'` and `'7'`. This more similar to Arduino's [`Serial.write()`](https://www.arduino.cc/en/Serial/Write) function than [`Serial.print()`](https://www.arduino.cc/en/Serial/Print).

## Usage
Simply connect all assembled nodes with RJ45 cables, and connect the translator node to the SBC with a USB cable. The nodes should automatically connect to the CAN bus and start listening for messages. The translator node will start sending messages to the CAN bus as soon as it receives a message from the SBC.

Upon powering on or resetting any node, the main status LED will blink if everything is initialized correctly.
* On the Translator node, the LED will blink once.
* On an Arm node, the LED will blink twice.
* If the LED continues blinking, there is a fault with initializing the CAN bus.
* If the LED does not blink, there is a fault with initializing the ESP32.

The main status will turn on and stay on after the **first valid message** is received on any node. If the LED does not turn on (on the Translator node), there is a fault with the serial communication. If it does not turn on (on the Arm node), there is a fault with the CAN communication or dip switch address.