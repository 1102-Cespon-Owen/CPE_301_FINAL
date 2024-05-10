
// FINAL PROJECT CPE 301
// CAL BRADFORD REGINALD III, OWEN CESPON, BRENDAN MCPARTLIN, SCOTT MCKENZIE
//5/8/24


// STEPPER MOTOR CONTROLS
Stepper myStepper(STEPS, 30, 32, 34, 36);
int currentPosition = 0; // Current position of the stepper motor
int openClose = 0;
#include <Stepper.h>
#define STEPS 200

#include <Bonezegei_DS1307.h>
Bonezegei_DS1307 rtc(0x68);
 

// DHT SET UP
DHT dht(DHTPIN,DHTTYPE); 
#include <DHT.h>
#define DHTPIN 7
#define DHTTYPE DHT11
// TEMPURATURE & HUMIDITY
float tempThres = 28, maxTemp = 0; 


//LCD SETUP
#include <LiquidCrystal.h>
LiquidCrystal lcd(11,12,2,3,4,5);

// SWAMP COOLER STATES
#define DISABLED_STATE 0
#define IDLE_STATE 1
#define ERROR_STATE 2
#define RUNNING_STATE 3
int state = DISABLED_STATE; 

// COMPONENT BOOLS
bool reset = 0, vent = 0, fan = 0, moniWater = 0, moniTemp = 0, ventPos = 0; 

// LEDS
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;
int red = 53, green = 51, blue = 49, yellow = 47;  



//WATER CONTROLS
const int waterPin = A15; // Analog input pin for water detector
const int thresholdWater = 5; // Adjust this threshold according to your sensor and environment


// FAN CONTROL
const int fanPin = 25; // pin for fan
int fanSpeed = 0;      // Initialize fan speed (0-255)

// BUTTON CONTROLS
  // ON
  const int startPin = 29; 

  // STOP

  // RESET
  const int resetPin = 33; 


// SETUP ============================================================================
void setup() {
  Serial.begin(9600); 

  //START BUTTON
  pinMode(startPin, INPUT_PULLUP);


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
    switch(state){
      case DISABLED_STATE:

        // change LEDs
        rLED = 0;
        gLED = 0;
        bLED = 0;
        yLED = 1;
        changeLED();

        attatchInterrupt(digitalPinToInterrupt(startPin), startPressed, FALLING);
      break; 

      case IDLE_STATE: 
        //Display Time
        diplayTime();

        // comstantly monitor temp and humid
        int humidity = dht.readHumidity();
        float temperature = dht.readTemperature();
        
        moniTemp = 1;
        moniWater = 1;

        tempHum(moniTemp, humidity, temperature);

        // change LEDs
        rLED = 0;
        gLED = 1;
        bLED = 0;
        yLED = 0;
        changeLED();


        int waterValue = analogRead(waterPin); // Read analog value from water detector
        if(waterValue <= threshold){
          state = ERROR_STATE; 
          break; 
        } else if(temperature > tempThres){
          state = RUNNING_STATE;
          break; 
        }
      break; 

      case ERROR_STATE:
        //Diplay Time
        displayTime(); 
        //print error messege
        lcd.clear();
        lcd.print("ERROR: Low Water");

        // change LEDs
        rLED = 1;
        gLED = 0;
        bLED = 0;
        yLED = 0;
        changeLED();

        // check water and if reset button was pressed
        int waterValue = analogRead(waterPin); // Read analog value from water detector
        if(digitalRead(33) == LOW && waterValue > thresholdWater){
          state = IDLE_STATE;
          break; 
        }
      break;
      
      case RUNNING_STATE:
        // Display Time
        displayTime(); 
        // Turn on FAN
        fanMode(1); 

        // change LEDs
        rLED = 0;
        gLED = 0;
        bLED = 1;
        yLED = 0;
        changeLED();

        // Monitor Tempurature
        float currentTemp = dht.readTempurature();
        if(currentTemp < tempThres){
          state = IDLE_STATE;
          fanMode(0); 
          break; 
        }

        //Check water level
        int waterValue = analogRead(waterPin); // Read analog value from water detector
        if(waterValue <= thresholdWater){
          state = ERROR_STATE;
          break; 
        }

      break; 
    }

}


void startButtonPressed() {
  // Disable start button ISR
  detachInterrupt(digitalPinToInterrupt(startPin));
  STATE = IDLE_STATE; 
}

// ADD INTERUPT FOR STOP BUTTON ????

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