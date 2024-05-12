// FINAL PROJECT CPE 301
// CAL PETERS, OWEN CESPON, BRENDAN MCPARTLIN, SCOTT MCKENZIE
//5/12/24

//PIN LIBRARIES
volatile unsigned char* port_l = (unsigned char*) 0x10B;
volatile unsigned char* ddr_l = (unsigned char*) 0x10A;
volatile unsigned char* pin_l = (unsigned char*) 0x109;
volatile unsigned char* port_h = (unsigned char*) 0x102; 
volatile unsigned char* ddr_h  = (unsigned char*) 0x101; 
volatile unsigned char* pin_h  = (unsigned char*) 0x100;
volatile unsigned char* port_d = (unsigned char*) 0x2B; 
volatile unsigned char* ddr_d  = (unsigned char*) 0x2A; 
volatile unsigned char* pin_d  = (unsigned char*) 0x29;
volatile unsigned char* port_c = (unsigned char*) 0x28; 
volatile unsigned char* ddr_c  = (unsigned char*) 0x27; 
volatile unsigned char* pin_c  = (unsigned char*) 0x26;
volatile unsigned char* port_b = (unsigned char*) 0x25;
volatile unsigned char* ddr_b = (unsigned char*) 0x24;
volatile unsigned char* pin_b = (unsigned char*) 0x23;

#define RDA 0x80
#define TBE 0x20  
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0   = (unsigned char *)0x00C6;

volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// STEPPER MOTOR CONTROLS
#include <Stepper.h>
#define STEPS 200
Stepper myStepper(STEPS, 30, 32, 34, 36);
int currentPos = 0; // Current position of the stepper motor

//TIME
#include <Bonezegei_DS1307.h>
Bonezegei_DS1307 rtc(0x68);
 
// DHT SET UP
#include <DHT.h>
#define DHTPIN 7
#define DHTTYPE DHT11
DHT dht(DHTPIN,DHTTYPE); 

// TEMPURATURE & HUMIDITY
float tempThres = 26.5; 

//LCD SETUP
#include <LiquidCrystal.h>
LiquidCrystal lcd(11,12,2,3,4,5);

// SWAMP COOLER STATES
#define DISABLED_STATE 0
#define IDLE_STATE 1
#define ERROR_STATE 2
#define RUNNING_STATE 3
int state = DISABLED_STATE; 

//LEDS
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;  

//
bool displayOnce = 0;

//WATER CONTROLS
const int waterPin = 0; // Analog input pin for water detector
const int thresholdWater = 20; // Adjust this threshold according to your sensor and environment 

//VARIABLES USED THROUGHOUT CODE
int humidity = 0;
float temperature = 0;
int waterValue = 0;

//Variables for dealy
unsigned long lastExecutionTime = 0;
const unsigned long stateDelay = 7000;


// SETUP ============================================================================
void setup(){
   U0init(9600); 
   adc_init();
  //START BUTTON
  *ddr_d &= ~(0x01 << 3);
  *pin_d |= 0x01 << 3;
  attachInterrupt(digitalPinToInterrupt(18), startPressed, FALLING);

  // LED PINS 47-53
  *ddr_l |= 0x01 << 2;  //YELLOW
  *ddr_l |= 0x01;       //BLUE
  *ddr_b |= 0x01 << 2;  //GREEN
  *ddr_b |= 0x01;       //RED
  
  // RESET
  *ddr_c &= ~(0x01 << 4); 
  
  // STEPPER
  *ddr_h &= ~(0x01 << 6); 

  // Setup for the fan
  *ddr_l |= 0x01 << 6;  //DIRECTION
  *ddr_l |= 0x01 << 4;  //SPEED

  //Stop Button
  *ddr_b &= ~(0x01 << 7);
  
  //Stepper set up
  myStepper.setSpeed(100); 

  //RTC SET UP
  rtc.begin();
  rtc.setFormat(12);        //Set 12 Hours Format
  rtc.setAMPM(1);           //Set AM or PM    0 = AM  1 =PM
  rtc.setTime("6:37:12");  //Set Time    Hour:Minute:Seconds
  rtc.setDate("5/11/24");   //Set Date    Month/Date/Year

  // DHT SET UP
  dht.begin();

  //LCD SETUP
  lcd.begin(16,2); 
   
}

//MAIN CODE=========================================================================================
void loop() {
    unsigned long currentTime = millis();



    //DIFFERENT STATES
    switch(state){
      case 0:
        displayOnce = 0;
        lcd.clear();
        
        //MOVES THE VENT
        if(*pin_h & (0x01 << 6)){
          currentPos = moveVent(currentPos);
        }
        fanMode(0);

        //CHANGES LEDS
        rLED = 0;
        gLED = 0;
        bLED = 0;
        yLED = 1;
        changeLED();

      break; 

      case 1: 
        //DISPLAYS TIME WHEN ENTERING IDLE ONCE
        if(displayOnce == 0){
          displayTime();
          displayOnce = 1;
        }
        
        //SWAPS TO DISABLED
        if(*pin_b & (0x01 << 7)){
          state = DISABLED_STATE;
          displayTime(); 
          break;
        }
        //MOVES THE VENT
        if(*pin_h & (0x01 << 6)){
          currentPos = moveVent(currentPos);
        }
        
        fanMode(0);


        //CONSTANTLY MONITOR HUM & TEMP, DISPLAY TO LCD
        humidity = dht.readHumidity();
        temperature = dht.readTemperature();


        //CHANGE LEDS
        rLED = 0;
        gLED = 1;
        bLED = 0;
        yLED = 0;
        changeLED();
            
        if(currentTime - lastExecutionTime >= stateDelay){
            tempHum(humidity, temperature);
        
            waterValue = adc_read(waterPin);
            if(waterValue <= thresholdWater){
              state = ERROR_STATE;
              displayTime();  
              displayOnce = 0;
              
              lastExecutionTime = currentTime; 
               
              break; 
            } else if(temperature > tempThres){
              state = RUNNING_STATE;
              displayTime(); 
              displayOnce = 0;

              lastExecutionTime = currentTime; 
               
              break; 
            }
            
            lastExecutionTime = currentTime; 
        }
       
      break; 

      case 2:
        //SWAPS TO DISABLED
        if(*pin_b & (0x01 << 7)){
          state = DISABLED_STATE;
          displayTime(); 
          break;
        }
        
        fanMode(0);
        
        //PRINT ERROR
        lcd.clear();
        lcd.print("ERROR: LOW WATER");

        //CHANGE LEDS
        rLED = 1;
        gLED = 0;
        bLED = 0;
        yLED = 0;
        changeLED();

        //CHECK RESET BUTTON
        if(*pin_c & (0x01 << 4)){ 
          state = IDLE_STATE;
          displayTime(); 
          break; 
        }
        
      break;
      
      case 3:
        //SWAP DISABLED
        if(*pin_b & (0x01 << 7)){
          state = DISABLED_STATE;
          displayTime(); 
          break;
        }
        
        //MOVES THE VENT
        if(*pin_h & (0x01 << 6)){
          currentPos = moveVent(currentPos);
        }
        
        //TURN ON FAN
        fanMode(1); 

        //CHANGE LEDS
        rLED = 0;
        gLED = 0;
        bLED = 1;
        yLED = 0;
        changeLED();

        //MONITOR TEMP & HUM, DISPLAY TO LCD
        humidity = dht.readHumidity();
        temperature = dht.readTemperature();


        if(currentTime - lastExecutionTime >= stateDelay){
           tempHum(humidity, temperature);

            //IF TEMP BELOW THRES, GO TO IDLE
            if(temperature <= tempThres){
              fanMode(0); 
              state = IDLE_STATE;
              displayTime(); 

              lastExecutionTime = currentTime; 
              
              break; 
            }
    
            //CHECK WATER LEVEL
            waterValue = adc_read(waterPin);
            if(waterValue <= thresholdWater){
              fanMode(0);
              state = ERROR_STATE;
              displayTime(); 

              lastExecutionTime = currentTime; 
               
              break; 
            }

            lastExecutionTime = currentTime; 
        }
       

      break; 
    }

    

}


void startPressed() {
  state = IDLE_STATE; 
}


void changeLED(){
  if(rLED == 1){
    *port_l &= ~(0x01 << 2);  //YELLOW
    *port_b &= ~(0x01 << 2);  //GREEN
    *port_l &= ~(0x01);       //BLUE
    *port_b |= 0x01;          //RED
  }else if(bLED == 1){
    *port_l &= ~(0x01 << 2);  //YELLOW
    *port_b &= ~(0x01 << 2);  //GREEN
    *port_l |= 0x01;          //BLUE
    *port_b &= ~(0x01);       //RED
  }else if(yLED == 1){
    *port_l |= 0x01 << 2;     //YELLOW
    *port_b &= ~(0x01 << 2);  //GREEN
    *port_l &= ~(0x01);       //BLUE
    *port_b &= ~(0x01);       //RED
  }else if(gLED == 1){
    *port_l &= ~(0x01 << 2);  //YELLOW
    *port_b |= 0x01 << 2;     //GREEN
    *port_l &= ~(0x01);       //BLUE
    *port_b &= ~(0x01);       //RED
  }
}

//MOVES STEPPER MOTOR VENT
int moveVent(bool state){
  if(state == 0){
    myStepper.step(200);
    state = 1;
  }
  else{
    myStepper.step(-200);
    state = 0;
  }
  return state;
}



// Display time stamp
void displayTime(){
  if(rtc.getTime()){
    U0printInt(rtc.getHour());
    U0putchar(':');
    U0printInt(rtc.getMinute());
    U0putchar(':');
    U0printInt(rtc.getSeconds());  
    U0putchar('\n');
    U0printInt(rtc.getMonth());
    U0putchar('/');
    U0printInt(rtc.getDate());
    U0putchar('/');
    U0printInt(rtc.getYear());
    U0putchar('\n');
  }
}


// This is to display tempurature and humidity 
void tempHum(int hum, float temp){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Temp:");
    lcd.print(temp);
    lcd.print("C");
    lcd.setCursor(0,1);
    lcd.print("Hum:");
    lcd.print(hum);
    lcd.print("%");
}

void fanMode(bool onOff){
  if(onOff){
    *port_l |= 0x01 << 6;
    *port_l |= 0x01 << 4;
  }
  else{
    *port_l |= 0x01 << 6;
    *port_l &= ~(0x01 << 4);    
  }
}

//INITIALIZES BAUD RATE AND ALLOWS USART
void U0init(unsigned long U0baud){
  unsigned long FCPU = 16000000;
  unsigned int tbaud;
  tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
  *myUCSR0A = 0x20;
  *myUCSR0B = 0x18;
  *myUCSR0C = 0x06;
  *myUBRR0  = tbaud;
}

//WRITE VALUE
void U0putchar(unsigned char U0pdata){
  while(!(*myUCSR0A & TBE));
  *myUDR0 = U0pdata;
}

void U0printInt(int number){
  if(number >= 10){
    U0printInt(number/10);
  }
  U0putchar('0' + (number % 10));
}

void adc_init(){
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
