char num = '0';
bool error, idle, start = 1, reset, fan, disabled, moniWater, moniTemp; 
bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;  
double waterLvl = 0, temp = 0, humidity = 0, maxTemp = 0, thresholdWtr = 0; 

void setup() {
  Serial.begin(9600); 
  // put your setup code here, to run once:
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
 
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

  }else if(bLED == 1){
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
    digitalWrite(8, HIGH);
    digitalWrite(7, LOW);    
  }else if(yLED == 1){
    digitalWrite(10, HIGH);
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, LOW);
  }else if(gLED == 1){
    digitalWrite(10, LOW);
    digitalWrite(9, LOW);
    digitalWrite(8, LOW);
    digitalWrite(7, HIGH);
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