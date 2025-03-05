#include <SPI.h>
#include <RF24.h>

// Define pins for L298N motor drivers
const int motor1Pin1 = A1;  // Left Motor Forward (IN1) connected to A1
const int motor1Pin2 = A2;  // Left Motor Reverse (IN2) connected to A2
const int motor2Pin1 = A3;  // Right Motor Forward (IN3) connected to A3
const int motor2Pin2 = A4;  // Right Motor Reverse (IN4) connected to A4
const int enablePin1 = 3;   // Left Motor Enable (ENA) connected to digital pin 3 (PWM control)
const int enablePin2 = 6;   // Right Motor Enable (ENB) connected to digital pin 6 (PWM control)

const int motor3Pin1 = A5;  // Lifting Motor Forward control pin connected to A5
const int motor3Pin2 = A6;  // Lifting Motor Reverse control pin connected to A6

const int stopSwitchPin = 7; // Stop switch for the fork connected to digital pin 7 (input)
const int ledPin1 = 2;       // Headlight LED1 control pin connected to digital pin 2
const int ledPin2 = 5;       // Headlight LED2 control pin connected to digital pin 5

// Define RF24 module pins
const int cePin = 9;  // CE pin for the NRF24L01 module connected to digital pin 9
const int csnPin = 10; // CSN pin for the NRF24L01 module connected to digital pin 10

// Create an RF24 object using CE and CSN pins
RF24 radio(cePin, csnPin);

// Define the address of the receiving node
const uint64_t pipe = 0xE8E8F0F0E1LL;  // This is the unique address for communication between the transmitter and receiver

// Declare variables for headlights toggle
bool headlightsState = false;
bool previousHeadlightsState = false;

// Function to set PWM and direction for the motors
void setMotorPWM(int enablePin, int motorPin1, int motorPin2, int speed) {
  if (speed > 0) {
    analogWrite(enablePin, speed);
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else if (speed < 0) {
    analogWrite(enablePin, -speed);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else {
    analogWrite(enablePin, 0);
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}

void setup() {
  // Initialize serial communication for debugging
  Serial.begin(9600);

  // Initialize RF24 module
  if (!radio.begin()) {  // Begin communication with the NRF24L01 module
    Serial.println("RF24 initialization failed!");  // Print an error if the radio fails to initialize
    while (1); // Halt the program
  }
  radio.openReadingPipe(1, pipe);  // Open a reading pipe at address 'pipe'
  radio.setPALevel(RF24_PA_HIGH);  // Set the power amplifier level to high for better performance
  radio.setDataRate(RF24_1MBPS);  // Set data rate to 1Mbps for a good balance of speed and range
  radio.startListening();  // Start listening for incoming data
  Serial.println("RF24 initialized successfully.");  // Print confirmation if initialization is successful

  // Set motor driver pins as outputs
  pinMode(motor1Pin1, OUTPUT);  // Left motor forward pin as output
  pinMode(motor1Pin2, OUTPUT);  // Left motor reverse pin as output
  pinMode(motor2Pin1, OUTPUT);  // Right motor forward pin as output
  pinMode(motor2Pin2, OUTPUT);  // Right motor reverse pin as output
  pinMode(enablePin1, OUTPUT);  // Enable pin for left motor as output (PWM control)
  pinMode(enablePin2, OUTPUT);  // Enable pin for right motor as output (PWM control)
  pinMode(motor3Pin1, OUTPUT);  // Lifting motor forward pin as output
  pinMode(motor3Pin2, OUTPUT);  // Lifting motor reverse pin as output
  pinMode(stopSwitchPin, INPUT_PULLUP);  // Stop switch pin set as input with internal pull-up resistor
  pinMode(ledPin1, OUTPUT);  // LED1 pin as output
  pinMode(ledPin2, OUTPUT);  // LED2 pin as output
}

void loop() {
  // Struct to hold received data
  struct Data {
    int vehicleSpeedX;  // Speed for vehicle movement on the X-axis (left-right)
    int vehicleSpeedY;  // Speed for vehicle movement on the Y-axis (forward-backward)
    int forkSpeed;      // Speed for controlling the fork up and down
    bool headlightsOn;  // Boolean to control headlights (this indicates if the button is pressed)
  };
  Data receivedData;  // Create an instance of the struct to store received data

  // Check if data is available from the transmitter
  if (radio.available()) {
    radio.read(&receivedData, sizeof(receivedData));  // Read the data into 'receivedData'

    Serial.print("Received Data - Vehicle Speed X: ");
    Serial.print(receivedData.vehicleSpeedX);
    Serial.print(" | Vehicle Speed Y: ");
    Serial.print(receivedData.vehicleSpeedY);
    Serial.print(" | Fork Speed: ");
    Serial.print(receivedData.forkSpeed);
    Serial.print(" | Headlights On: ");
    Serial.println(receivedData.headlightsOn ? "True" : "False");

    int leftMotorSpeed = receivedData.vehicleSpeedY + receivedData.vehicleSpeedX;
    int rightMotorSpeed = receivedData.vehicleSpeedY - receivedData.vehicleSpeedX;

    // Ensure motor speeds are within the valid range (0-255 for PWM)
    leftMotorSpeed = constrain(leftMotorSpeed, -255, 255);
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);

    // Debug motor speeds
    Serial.print("Left Motor Speed: ");
    Serial.print(leftMotorSpeed);
    Serial.print(" | Right Motor Speed: ");
    Serial.println(rightMotorSpeed);

    // Control vehicle motors for tank-like movement with PWM using the setMotorPWM function
    setMotorPWM(enablePin1, motor1Pin1, motor1Pin2, leftMotorSpeed);
    setMotorPWM(enablePin2, motor2Pin1, motor2Pin2, rightMotorSpeed);

    // Control lifting motor for fork movement with the second joystick's Y-axis (no PWM)
    if (receivedData.forkSpeed > 0) { 
      // Lift fork up
      digitalWrite(motor3Pin1, HIGH); 
      digitalWrite(motor3Pin2, LOW); 
      Serial.println("Lifting fork up");
    } else if (receivedData.forkSpeed < 0) { 
      // Lower fork
      digitalWrite(motor3Pin1, LOW); 
      digitalWrite(motor3Pin2, HIGH); 
      Serial.println("Lowering fork");
    } else { 
      // Hold fork position (no movement)
      digitalWrite(motor3Pin1, LOW);
      digitalWrite(motor3Pin2, LOW);
      Serial.println("Holding fork position");
    }

    // Toggle headlights on button press
    if (receivedData.headlightsOn && !previousHeadlightsState) {  // Detect a rising edge (button press)
      headlightsState = !headlightsState;  // Toggle the state of the headlights
    }
    previousHeadlightsState = receivedData.headlightsOn;  // Update the previous button state

    // Set the headlights based on the toggled state
    if (headlightsState) {
      digitalWrite(ledPin1, HIGH);  // Turn on LED1 (headlight)
      digitalWrite(ledPin2, HIGH);  // Turn on LED2 (headlight)
    } else {
      digitalWrite(ledPin1, LOW);   // Turn off LED1
      digitalWrite(ledPin2, LOW);   // Turn off LED2
    }

    // Print all data to the serial monitor for debugging
    Serial.print("Vehicle Speed X: ");
    Serial.print(receivedData.vehicleSpeedX);
    Serial.print(" | Vehicle Speed Y: ");
    Serial.print(receivedData.vehicleSpeedY);
    Serial.print(" | Fork Speed: ");
    Serial.print(receivedData.forkSpeed);
    Serial.print(" | Headlights On: ");
    Serial.println(headlightsState ? "True" : "False");
  }
}
