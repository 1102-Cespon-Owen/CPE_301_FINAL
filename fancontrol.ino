//Fan control code


int motorPin = 3;
 
void setup() 
{ 
  pinMode(motorPin, OUTPUT);
  Serial.begin(9600);
  while (! Serial);
  Serial.println("Speed 0 to 255");
} 
 
 
void loop() {
    // Check if there is data available in the Serial Monitor
    if (Serial.available() > 0) {
        int option = Serial.parseInt();

        // Switch statement to determine fan operation based on input
        switch (option) {
            case 1: // Turn off the fan
                analogWrite(motorPin, 0);
                break;
            case 3: // Run the fan
                analogWrite(motorPin, 255); // Full speed (you can adjust this value)
                break;
            default: // Invalid option
                Serial.println("Invalid option!");
                break;
        }
    }
}