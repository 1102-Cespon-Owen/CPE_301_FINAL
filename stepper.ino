#include <Stepper.h>

#define STEPS 200

// Define the pins connected to the stepper motor
Stepper myStepper(STEPS, 2, 3, 4, 5);

int currentPosition = 0; // Current position of the stepper motor

void setup() {
  // Initialize serial communication
  Serial.begin(9600);
  myStepper.setSpeed(60);
}

void loop() {
  // Check if there's serial data available
  if (Serial.available() > 0) {
    // Read the input from serial monitor
    int targetPosition = Serial.parseInt();
    
    // Move the stepper motor to the target position
    moveStepper(targetPosition);
  }
}

// Function to move the stepper motor to the target position
void moveStepper(int targetPos) {
  if(targetPos < 0){
    myStepper.step(targetPos);
  }
  else{
    myStepper.step(targetPos);
  }
  delay(100);
}
