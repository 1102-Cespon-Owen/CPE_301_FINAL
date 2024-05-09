
// FINAL PROJECT CPE 301
// CAL BRADFORD REGINALD III, OWEN CESPON, BRENDAN MCPARTLIN, SCOTT MCKENZIE
//5/8/24

// LIBRARIES
#include <Stepper.h>
#define STEPS 200

#include <Bonezegei_DS1307.h>
Bonezegei_DS1307 rtc(0x68);
 
#include <LiquidCrystal.h>

#include <DHT.h>
#define DHTPIN 7
#define DHTTYPE DHT11

// DHT SET UP
DHT dht(DHTPIN,DHTTYPE); 

//LCD SETUP
LiquidCrystal lcd(11,12,2,3,4,5);

// SWAMP COOLER STATES
char num = '0';
//STATE BOOLS
bool error = 0, idle = 1, start = 0, runState = 0;

// COMPONENT BOOLS
bool reset = 0, vent = 0, fan = 0, moniWater = 0, moniTemp = 0, ventPos = 0; 

// LEDS
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;
int red = 53, green = 51, blue = 49, yellow = 47;  

// TEMPURATURE & HUMIDITY
float tempThres = 27, maxTemp = 0; 


//WATER CONTROLS
const int waterPin = A15; // Analog input pin for water detector
const int thresholdWTR = 5; // Adjust this threshold according to your sensor and environment

// STEPPER MOTOR CONTROLS
Stepper myStepper(STEPS, 30, 32, 34, 36);
int currentPosition = 0; // Current position of the stepper motor
int openClose = 0;


// FAN CONTROL
const int fanPin = 25; // pin for fan
int fanSpeed = 0;      // Initialize fan speed (0-255)

// BUTTON CONTROLS
  // ON/OFF
  int button = 0;
  int lastButtonState = 0;

  // RESET
  const int resetPin = 33; 


// SETUP ============================================================================
void setup() {
  Serial.begin(9600); 
  //START BUTTON
  pinMode(29, INPUT);


  // LED PINS 46-52
  pinMode(yellow, OUTPUT); // YELLOW
  pinMode(blue, OUTPUT); // BLUE
  pinMode(green, OUTPUT); // GREEN
  pinMode(red, OUTPUT); // RED

  // RESET
  pinMode(resetPin, INPUT); 
  
  // STEPPER
  pinMode(9,OUTPUT); 

  // Setup for the fan
  pinMode(fanPin, OUTPUT);

  //Stepper set up
  myStepper.setSpeed(100); 

  //RTC SET UP
  rtc.begin();
  rtc.setFormat(12);        //Set 12 Hours Format
  rtc.setAMPM(1);           //Set AM or PM    0 = AM  1 =PM
  rtc.setTime("9:58:10");  //Set Time    Hour:Minute:Seconds
  rtc.setDate("5/08/24");   //Set Date    Month/Date/Year

  // DHT SET UP
  dht.begin();

  //LCD SETUP
  lcd.begin(16,2); 

}

//MAIN CODE=========================================================================================
void loop() {
  Serial.print(idle);
  
  // start button
  int button1 = digitalRead(29);
  
  // reset button
  int button2 = digitalRead(33);
  
  if(button1 == HIGH){
    start = !start;
  }
  
  // if button pressed goes into idle 
  if(start == 1){

     // comstantly reads
    int humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    if(idle == 1){
      // CHANGE STATE TO IDLE
      idle = 1; 
      error = 0; 
      runState = 0; 
      start = 1; 

      
      moniTemp = 1;
      moniWater = 1;
      tempHum(moniTemp, humidity, temperature);
      rLED = 0;
      gLED = 1;
      bLED = 0;
      yLED = 0;
      changeLED();

      delay(5000);
      
      //CHECK WATER LEVEL ////////////////////////////////////
      int waterValue = analogRead(waterPin); // Read analog value from water detector
      if (waterValue <= thresholdWTR) {  
          Serial.println("No water detected");

          //CHANGES STATES 
          error = 1;
          idle = 0;
          runState = 0; 
          start = 1; 
          
          // LED CHANGE
          rLED = 1;
          gLED = 0;
          bLED = 0;
          yLED = 0;
          changeLED();
      }

          // IF ERROR IS THROWN ////////////////////////////////////////////////////
    }else if(error == 1){
        Serial.print("here");
        lcd.clear();
        lcd.print("!!!ERROR!!!!"); 
        idle = 0;
        runState = 0;  
        start = 1;

        // CHECK IF THE RESET BUTTON IS PRESSED /////////////////////////////
        if(button2 == HIGH){
          // CHANGE SET
          error = 0;
          idle = 1; 
          runState = 0; 
          start = 1;

          Serial.print("in error");
          delay(500);

        }
        
     }
      
    
 
    

    
    //if(vent){
     // ventPos = moveVent(ventPos);
    //}
  }
  
  //DISABLED STATE
  else{
    rLED = 0;
    gLED = 0;
    bLED = 0;
    yLED = 1;
    changeLED();
    fan = 0;
    moniWater = 0;
    moniTemp = 0;
    if(fan == 1){
      fanMode(0);
    }
    tempHum(moniTemp, 0, 0);
  } 
  delay(500);
}

void changeLED(){
  if(rLED == 1){
    digitalWrite(yellow, LOW);
    digitalWrite(red, HIGH);
    digitalWrite(blue, LOW);
    digitalWrite(green, LOW);
  }else if(bLED == 1){
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
    digitalWrite(blue, HIGH);
    digitalWrite(green, LOW);  
  }else if(yLED == 1){
    digitalWrite(yellow, HIGH);
    digitalWrite(red, LOW);
    digitalWrite(blue, LOW);
    digitalWrite(green, LOW);
  }else if(gLED == 1){
    digitalWrite(yellow, LOW);
    digitalWrite(red, LOW);
    digitalWrite(blue, LOW);
    digitalWrite(green, HIGH);
  }
}

// Function to move the stepper motor to the target position
int moveVent(bool state){
  if(state == 0){
    myStepper.step(1000);
    state = 1;
  }
  else{
    myStepper.step(-1000);
    state = 0;
  }
  return state;
}



// Display time stamp
void displayTime(){
  if(rtc.getTime()){
    Serial.print("Time: ");
    Serial.print(rtc.getHour());
    Serial.print(":");
    Serial.print(rtc.getMinute());
    Serial.print(":");
    Serial.print(rtc.getSeconds());
    if (rtc.getFormat() == 12) {  // returns 12 or 24 hour format

      if (rtc.getAMPM()) {  //return 0 = AM  1 = PM
        Serial.print("PM  ");
      } else {
        Serial.print("AM  ");
      }
    }
    

    Serial.print("Date ");
    Serial.print(rtc.getMonth());
    Serial.print("/");
    Serial.print(rtc.getDate());
    Serial.print("/");
    Serial.println(rtc.getYear());
  }
}


// This is to display tempurature and humidity 
void tempHum(bool onOff, int hum, float temp){
  if(onOff){
    lcd.setCursor(0,0);
    lcd.print("Temp:");
    lcd.print(temp);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Hum:");
    lcd.print(hum);
    lcd.print("%");
  }
  else{
    lcd.clear();
  }
    //delay(1000);
}

void fanMode(bool onOff){
  if(onOff){
    analogWrite(fanPin, 255);
  }
  else{
    analogWrite(fanPin,0);
  }
}

/*
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
    

    //analogWrite(fanPin, 0); // Full speed (you can adjust this value)

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

void fanOn(){
  
}
void fanOff(){
  
}


//Work on next

double checkWater(){
  //Use water level sensor from kit

}

double checkTemp(){
// Use temperature and humidity module.
}

*/
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
