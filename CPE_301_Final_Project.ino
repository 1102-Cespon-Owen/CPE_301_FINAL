void setup() {
  // put your setup code here, to run once:

}


// Need to finish states/double check the states. On Friday. start looking at non library stuff
void loop() {
  // put your main code here, to run repeatedly:
  bool error, idle, start, reset, fan, disabled; 
  bool gLED = 0, yLED = 0, rLED = 0,  bLED = 0;  
  double waterLvl = 0, temp = 0, humidity = 0, maxTemp = 0. thresholdWtr = 0; 


  if(start = 1){

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
    } else if(temp <= maxTemp){
      idle = 1; // means idling
      fan = 0; 

      bLED = 0;
      gLED = 1;
      yLED = 0;
      rLED = 0; 

      // need to display temp and humidity
    } 
    
    // ERROR CHECKING FOR WATER LEVEL
    if(waterLvl <= thresholdWtr){
      println("Water level is too low");
      error = 1; 
      idle = 0; 
      
      rLED = 1;
      bLED = 0;
      yLED = 0;
      gLED = 0; 


      // Check if reset button is pressed
      if(reset == 1){
        idle = 1;

        rLED = 0;
        bLED = 0;
        yLED = 0;
        gLED = 1; 
      }


    
    }
      // if start = 0 anywhere
      if(start == 0){
        fan = 0; 
        disabled = 1;

        rLED = 0;
        bLED = 0;
        yLED = 1;
        gLED = 0; 


      }
  }
}



bool checkWater(){}

bool checkTemp(){}