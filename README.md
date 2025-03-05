Remote-Controlled Forklift with Video Streaming


Project Overview
This project is a remote-controlled forklift system designed for industrial use, controlled using Arduino Nano with live video streaming via ESP32-CAM. The forklift is controlled remotely using two joysticks: one for 4WD vehicle movement and the other for forklift lift and angle control. The project uses NRF24L01 wireless modules for long-range communication between the transmitter and receiver. The ESP32-CAM module streams live video, allowing real-time monitoring of the forklift’s environment.

System Architecture
The system is divided into two main sections: the transmitter and the receiver.

Transmitter:
Arduino Nano: 
Processes inputs from joysticks.

NRF24L01:
Transmits the joystick control signals wirelessly.

Two Joysticks:
Joystick 1: Controls vehicle movement (forward, reverse, left, right).
Joystick 2: Controls forklift lifting and fork angle.

Power Supply:
Powered by USB or a regulated 5V power source.

Receiver:
Arduino Nano:
Receives commands and controls the motors.

NRF24L01:
Receives wireless signals from the transmitter.

L298N Motor Drivers (2):
One motor driver controls 4WD motors for vehicle movement.
The other motor driver controls the forklift lifting motor.

ESP32-CAM:
Streams live video for real-time monitoring of the forklift's surroundings.

Li-ion Battery Packs:
Two packs of 4 Li-ion cells (2500mAh each) connected in series and in parallel for sufficient voltage and current.

Buck Converter:
Steps down voltage from the battery pack for the Arduino and other components.

Stop Switch:
Prevents the forklift from over-extending when lifting.

Hardware Components

Transmitter:

Arduino Nano: 
Main controller for joystick input.

NRF24L01 Module with External Antenna: 
Wireless communication.

Two Joysticks: For controlling the forklift.
USB Power Supply: Or external 5V regulated supply.
Receiver:
Arduino Nano: Receives signals from the transmitter and controls the motors.
NRF24L01 Module with External Antenna: Wireless communication.
ESP32-CAM: Streams live video over Wi-Fi.
L298N Motor Drivers (2):
Motor Driver 1: Controls 4WD vehicle motors.
Motor Driver 2: Controls forklift lift motor.
Li-ion Batteries (8x 2500mAh): Two packs connected in parallel and series (14.8V).
Buck Converter: To step down voltage for the Arduino Nano.
Stop Switch: To control the upward motion of the forklift.
Power Supply Setup
Batteries: Two packs of 4 Li-ion batteries (2500mAh) in series, connected in parallel. Each battery provides 3.7V, so the total voltage is 14.8V.
Buck Converter: Reduces the voltage from 14.8V to 5V for the Arduino Nano and NRF24L01.
How to Set Up the Project
1. Transmitter Side Setup:
Hardware Connections:

Connect two joysticks to the Arduino Nano:
Joystick 1 controls 4WD motors.
Joystick 2 controls forklift lifting and angle.
Connect the NRF24L01 module to the Arduino Nano.
Power the transmitter using USB or a regulated 5V power source.
Upload Code:

Write the code to read joystick inputs and send the data via NRF24L01.
Use the Arduino IDE to upload the transmitter code.
2. Receiver Side Setup:
Hardware Connections:

Connect the Arduino Nano to the NRF24L01 module.
Connect the ESP32-CAM module to stream live video over Wi-Fi.
Connect the L298N motor drivers to control the 4WD motors and forklift motor.
Connect the battery packs and buck converter to provide power to the receiver.
Add the stop switch to prevent the forklift from over-extending when lifting.
Upload Code:

Write the receiver code to handle motor control and video streaming.
Use the Arduino IDE to upload the receiver code.
Code and Software
1. Transmitter Code:
Reads inputs from the joysticks.
Sends control signals (for forklift movement and lifting) via NRF24L01.
2. Receiver Code:
Receives control signals via NRF24L01.
Controls the L298N motor drivers to move the forklift and operate the lift.
Streams live video using the ESP32-CAM module over Wi-Fi.
3. Video Streaming Code (ESP32-CAM):
The ESP32-CAM streams live video using its onboard camera module.
The video feed is accessible via a local IP address over the same Wi-Fi network.
Testing and Results
1. Wireless Communication Range:
The system maintains reliable communication within a range of approximately 200 meters using the NRF24L01 module with an external antenna.
2. Video Streaming:
The ESP32-CAM provides a live video feed with low latency, allowing the operator to view the forklift’s surroundings in real-time.
3. Motor Control:
The L298N motor drivers efficiently control the 4WD motors and the forklift lift motor, providing smooth operation.
Challenges and Solutions
1. Power Management:
Ensuring stable power supply for both the high-power motors and the low-power Arduino Nano and communication modules was a challenge.
The use of a buck converter to step down voltage ensured reliable operation without power fluctuations.
2. Communication Range:
The NRF24L01 module with an external antenna was chosen to ensure a reliable communication range of up to 200 meters, which is suitable for most industrial applications.
Future Improvements
Autonomous Features: Adding sensors for collision detection or autonomous movement.
Improved Video Quality: Upgrading to a higher resolution camera for clearer video feeds.
Extended Range: Using a more powerful communication module like LoRa for extended range operation.
Battery Management System (BMS): Integrating a BMS to protect the Li-ion batteries from overcharging and over-discharging.
Conclusion
This project successfully developed a remote-controlled forklift with live video streaming. It allows the operator to control the forklift’s movement and lifting mechanism remotely while viewing the surroundings via a live video stream. The system uses NRF24L01 for long-range wireless communication and ESP32-CAM for video streaming, making it a versatile and practical solution for industrial applications.

References
NRF24L01 Datasheet
ESP32-CAM Documentation
L298N Motor Driver Datasheet
Arduino Nano Pinout and Specifications
