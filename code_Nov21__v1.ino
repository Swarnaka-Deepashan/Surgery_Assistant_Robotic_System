#include <Servo.h>

#define stepPin 8
#define dirPin 9

const int stepsPerRotation = 200;  // Number of steps for one rotation
const float leadOfScrew = 8.0;  // Distance the nut moves for one rotation in mm
bool moved = false;  // Flag to ensure the motor moves only once
bool busy = false;  // Flag to indicate if the system is busy

Servo servoGripper;
Servo servo90;
Servo servoDisc;
int currentStation = 1;  // Variable to keep track of the current station

void setup() {
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  Serial.begin(9600);

  servoGripper.attach(6); // Attach the servoGripper on digital pin 6
  servo90.attach(5);      // Attach the servo90 on digital pin 5
  servoDisc.attach(10);   // Attach the servoDisc on digital pin 10
  
  // Home the servos by setting them to 0 degrees
  servoGripper.write(0);
  servo90.write(0);
  servoDisc.write(90);    // Initial position for servoDisc
  delay(2000); // Wait a bit to ensure servos have reached the home position
  Serial.println("Ready");
}

void operateServoDisc(int targetStation) {
  int delayTime = 0;

  if (targetStation == currentStation) {
    delayTime = 0;
  } else {
    int distance = targetStation - currentStation;
    if (distance < 0) {
      distance += 3; // Wrap around for negative distances
    }

    switch(distance) {
      case 1:
        delayTime = 350;
        break;
      case 2:
        delayTime = 700;
        break;
      case 0:
        delayTime = 1050;
        break;
    }
  }

  servoDisc.write(80);
  delay(delayTime);
  servoDisc.write(90); // Return to initial position after delay

  currentStation = targetStation;
}

void moveUpward(float distance_mm) {
  int steps = (int)((distance_mm / leadOfScrew) * stepsPerRotation);
  digitalWrite(dirPin, HIGH); // Assuming HIGH direction is upwards, change if necessary

  for(int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Adjust delay for speed control
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500); // Adjust delay for speed control
  }
}

void moveDownward(float distance_mm) {
  int steps = (int)((distance_mm / leadOfScrew) * stepsPerRotation);
  digitalWrite(dirPin, LOW); // Assuming LOW direction is downwards, change if necessary

  for(int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500); // Adjust delay for speed control
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500); // Adjust delay for speed control
  }
}


void extendServoGripper() {
  for(int pos = 0; pos <= 180; pos++) {
    servoGripper.write(pos); 
    delay(15);
  }

  delay(1000);
}

void retractServoGripper() {
  for(int pos = 180; pos >= 0; pos--) {
    servoGripper.write(pos); 
    delay(15);
  }

  delay(1000);
}

void turnServo90() {
  for(int pos = 0; pos <= 90; pos++) {
    servo90.write(pos); 
    delay(15);
  }

  delay(1000);  // Wait for 1 second at 90 degrees
}

void returnServo90() {
  for(int pos = 90; pos >= 0; pos--) {
    servo90.write(pos); 
    delay(15);
  }

  delay(1000);  // Wait for 1 second at 90 degrees
}

void loop() {
  // Check if new data is available on the serial port and system is not busy
  if (Serial.available() > 0 && !busy) {
    int targetStation = Serial.parseInt(); // Read input from serial monitor

    // Check if the input is one of the valid stations
    if (targetStation >= 1 && targetStation <= 3) {
      busy = true;  // Set the system to busy
      operateServoDisc(targetStation); // Operate the servo based on the input

      // Execute the following sequence for any valid station
      delay(1000);
      moveUpward(30.0);  // Move stepper motor 30 mm
      extendServoGripper(); 
      delay(50);
      moveUpward(30.0);
      delay(1000);
      turnServo90();     // Turn servo90 after servoGripper has retracted
      delay(1000);
      returnServo90();
      moveDownward(30.0);
      retractServoGripper();
      moveDownward(30.0);

      busy = false;  // Operation complete, system is not busy anymore

      // Clear the Serial buffer to discard any input received during the operation
      while (Serial.available() > 0) {
        Serial.read();
      }

      Serial.println("Operation complete. Enter next position (1, 2, or 3):");
    }
  }
}
