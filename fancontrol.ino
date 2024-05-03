
const int fanPin = 10; // Connect the fan's PWM wire to pin 8
int fanSpeed = 0;      // Initialize fan speed (0-255)

void setup() {
    pinMode(fanPin, OUTPUT);
    Serial.begin(9600);
}

void loop() {
    // Check if there is data available in the Serial Monitor
    if (Serial.available() > 0) {
        int option = Serial.parseInt();

        // Switch statement to determine fan operation based on input
        switch (option) {
            case 1: // Turn off the fan
                analogWrite(fanPin, 0);
                break;
            case 3: // Run the fan
                analogWrite(fanPin, 255); // Full speed (you can adjust this value)
                break;
            default: // Invalid option
                Serial.println("Invalid option!");
                break;
        }
    }
}