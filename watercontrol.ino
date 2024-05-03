const int waterPin = A0; // Analog input pin for water detector
const int threshold = 5; // Adjust this threshold according to your sensor and environment


void setup() {
    Serial.begin(9600);

    //green
    pinMode(10, OUTPUT);

    //red
    pinMode(9, OUTPUT);
}

void loop() {
    int waterValue = analogRead(waterPin); // Read analog value from water detector

    if (waterValue > threshold) {
        Serial.println("Water detected!"); // Output message indicating water detected
          digitalWrite(10, HIGH);
          digitalWrite(9,LOW); 
    } else {  
        Serial.println("No water detected");
        // Add code here to perform actions when no water is detected
          digitalWrite(10,LOW);
          digitalWrite(9,HIGH);
    }

    delay(1000); // Delay for stability
}