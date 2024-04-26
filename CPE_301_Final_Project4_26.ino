char num = '0';
bool error, idle, start = 1, reset, fan, disabled, moniWater, moniTemp; 
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;  
double waterLvl = 0, temp = 0, humidity = 0, maxTemp = 0, thresholdWtr = 0; 
const int fanPin = 10; // Connect the fan's PWM wire to pin 8
int fanSpeed = 0;      // Initialize fan speed (0-255)


const int waterPin = A0; // Analog input pin for water detector
const int threshold = 5; // Adjust this threshold according to your sensor and environment


void setup() {
  Serial.begin(9600); 
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);

  // Setup for the fan
  pinMode(fanPin, OUTPUT);

}


// Need to finish states/double check the states. On Friday. start looking at non library stuff
void loop() {
  // put your main code here, to run repeatedly:



  if(start == 1){
    //THIS IS WHERE WE WILL CALL CHECKWATER AND CHECKTEMP FUNCTIONS
    moniWater = 1;
    moniTemp = 1;

    // CHECKING IF TEMP IS GREATER THAN MAX TO TURN ON COOLER
    if(temp > maxTemp){
      idle = 0; // means running 
      fan = 1; 

      bLED = 1;
      gLED = 0;
      yLED = 0;
      rLED = 0; 

      // need to display temp and humidity

    // TURNS THE COOLER TO IDLE IF TEMP IS BELOW THE MAXTEMP
    }if(temp <= maxTemp){
      idle = 1; // means idling
      fan = 0; 

      bLED = 0;
      gLED = 1;
      yLED = 0;
      rLED = 0; 

      // need to display temp and humidity
    } 
    
    // ERROR CHECKING FOR WATER LEVEL
    if(waterLvl < thresholdWtr){
      //println("Water level is too low");
      error = 1; 
      idle = 0; 
      
      rLED = 1;
      bLED = 0;
      yLED = 0;
      gLED = 0; 


      // Check if reset button is pressed
      if(reset == 1 && waterLvl >= thresholdWtr){
        idle = 1;

        rLED = 0;
        bLED = 0;
        yLED = 0;
        gLED = 1; 
      }    
    }
  }
  // if start = 0 anywhere
  //This is essentially just disabled
  else{
    fan = 0; 
    //disabled = 1;
    moniWater = 0;
    moniTemp = 0;

    rLED = 0;
    bLED = 0;
    yLED = 1;
    gLED = 0; 
  }
  if(rLED == 1){
    digitalWrite(10, LOW);
    digitalWrite(9, HIGH);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);

    

    // for water level is too low
    Serial.println("No water detected");
    // Add code here to perform actions when no water is detected
    digitalWrite(10,LOW);
    digitalWrite(9,HIGH);

    analogWrite(fanPin, 0); // Full speed (you can adjust this value)

  }else if(bLED == 1){
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);  

    // this is for the control fan
    analogWrite(fanPin, 255); // Full speed (you can adjust this value)

  }else if(yLED == 1){
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);

    analogWrite(fanPin, 0); // Full speed (you can adjust this value)


  }else if(gLED == 1){
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);



    // for water level is good
    Serial.println("Water detected!"); // Output message indicating water detected
    digitalWrite(10, HIGH);
    digitalWrite(9,LOW); 
    

    analogWrite(fanPin, 0); // Full speed (you can adjust this value)

  }else{}
  if(Serial.available() > 0) {
    num = Serial.read();
  }
  switch(num){
    case '1': start = 0;
      break;
    case '2': start = 1;
      break;
    case '3': waterLvl = -1;
      break;
    case '4': waterLvl = 2;
      break;
    case '5': temp = -1;
      break;
    case '6': temp = 3;
      break;
    case '\n': 
      break;
    default:
      break;
  }
  Serial.println(num);
  delay(1000);
}



//Work on next

double checkWater(){
  //Use water level sensor from kit

}

double checkTemp(){
// Use temperature and humidity module.
}


/*
THIS IS FOR THE FAN CONTROL 

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
*/





///// THIS IS FOR THE WATER LEVEL DETECTOR
/*
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
*/