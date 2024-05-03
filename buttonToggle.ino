int button = 0; 
int lastButtonState = 0;
bool isToggled = false; 

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  // switch
  pinMode(11,OUTPUT);

  // green
  pinMode(10,OUTPUT);

  // red
  pinMode(9,OUTPUT);

  // yellow
  pinMode(8,OUTPUT); 

}

void loop() {

  button = digitalRead(11); 
  // put your main code here, to run repeatedly:
  if(button != lastButtonState){
    if(button == HIGH){
      isToggled = !isToggled; 
      if(isToggled){
        digitalWrite(10, HIGH);
        digitalWrite(8, LOW);
        digitalWrite(9, LOW); 
      } else {
        digitalWrite(10, LOW);
        digitalWrite(8, HIGH);
        digitalWrite(9, LOW);
      }
    }
    lastButtonState = button; 
    delay(50); 
  }

}
