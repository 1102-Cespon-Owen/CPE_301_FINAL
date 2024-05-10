// FINAL PROJECT CPE 301
// CAL BRADFORD REGINALD III, OWEN CESPON, BRENDAN MCPARTLIN, SCOTT MCKENZIE
//5/8/24


// STEPPER MOTOR CONTROLS
#include <Stepper.h>
#define STEPS 200
Stepper myStepper(STEPS, 30, 32, 34, 36);
int currentPosition = 0; // Current position of the stepper motor
int openClose = 0;


#include <Bonezegei_DS1307.h>
Bonezegei_DS1307 rtc(0x68);
 

// DHT SET UP
#include <DHT.h>
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE); 

// TEMPURATURE & HUMIDITY
float tempThres = 10; 


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
bool reset = 0, vent = 0, fan = 0, moniWater = 0, moniTemp = 0;
int ventPos = 0;

// LEDS
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;
int red = 53, green = 51, blue = 49, yellow = 47;  



//WATER CONTROLS
const int waterPin = A0; // Analog input pin for water detector
const int thresholdWater = 20; // Adjust this threshold according to your sensor and environment


// FAN CONTROL
const int fanPin = 45; // pin for fan
int fanSpeed = 0;      // Initialize fan speed (0-255)

// BUTTON CONTROLS
  // ON
  const int startPin = 18; 

  // STOP

  // RESET
  const int resetPin = 33; 

  int humidity = 0;
  int temperature = 0;
  int waterValue = 0;


// SETUP ============================================================================
void setup() {
  Serial.begin(9600); 

  //START BUTTON
  pinMode(startPin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(18), startPressed, FALLING);



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
   // Serial.println(digitalRead(2));
    switch(state){
      case 0:
        fanMode(0);
        // change LEDs
        rLED = 0;
        gLED = 0;
        bLED = 0;
        yLED = 1;
        changeLED();

        //attachInterrupt(digitalPinToInterrupt(startPin), startPressed, FALLING);
      break; 

      case 1: 

      fanMode(0);
        //Display Time
        //displayTime();

        // comstantly monitor temp and humid
        humidity = dht.readHumidity();
        temperature = dht.readTemperature();
        
        moniTemp = 1;
        moniWater = 1;

        tempHum(moniTemp, humidity, temperature);

        // change LEDs
        rLED = 0;
        gLED = 1;
        bLED = 0;
        yLED = 0;
        changeLED();


        waterValue = analogRead(waterPin); // Read analog value from water detector
        if(waterValue <= thresholdWater){
          state = ERROR_STATE; 
          break; 
        } else if(temperature > tempThres){
          state = RUNNING_STATE;
          break; 
        }
      break; 

      case 2:
      fanMode(0);
        //Diplay Time
        //displayTime(); 
        
        //print error messege
        lcd.clear();
        lcd.print("ERROR: Low Water");

        // change LEDs
        rLED = 1;
        gLED = 0;
        bLED = 0;
        yLED = 0;
        changeLED();

        //check water and if reset button was pressed
        //Serial.println(digitalRead(33));
        waterValue = analogRead(waterPin); // Read analog value from water detector
        
        if(digitalRead(33) == HIGH ){
          
          Serial.print("RESET HIT"); 
          state = IDLE_STATE;
          break; 
        }
        Serial.print("AFTER BUTTON TEST");
        
      break;
      
      case 3:
        // Display Time
        //displayTime(); 
        
        // Turn on FAN
        fanMode(1); 

        // change LEDs
        rLED = 0;
        gLED = 0;
        bLED = 1;
        yLED = 0;
        changeLED();

        // Monitor Tempurature
        float currentTemp = dht.readTemperature();
        if(currentTemp < tempThres){
          fanMode(0); 
          state = IDLE_STATE;
          break; 
        }

        //Check water level
        waterValue = analogRead(waterPin); // Read analog value from water detector
        if(waterValue <= thresholdWater){
          fanMode(0);
          state = ERROR_STATE;
          break; 
        }

      break; 
    }

}


void startPressed() {
  // Disable start button ISR
  Serial.print("in interrupt");
  //detachInterrupt(digitalPinToInterrupt(startPin));
  state = IDLE_STATE; 
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
