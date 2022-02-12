// simple uart communication

#include<HardwareSerial.h>

HardwareSerial SUART(1); 


void setup()
{
  Serial.begin(115200);
  SUART.begin(115200, SERIAL_8N1, 16, 17);
  delay(30);
}

void loop()
{
  byte n = Serial.available();//checking if charcater has come from InputBox of Serial Monitor 
  if(n !=0)
  {
    char x = Serial.read();  //yes, there is a charcater; read it
    Serial.print(x);        //send back the charctaer to OutputBox of Serial Monitor
    SUART.print(x);         //send the charcater to NANO as ASCII code
  }

  n = SUART.available();//checking if charcater has come from InputBox of Serial Monitor 
  if(n !=0)
  {
    char x = SUART.read();  //yes, there is a charcater; read it
    Serial.print(x);        //send back the charctaer to OutputBox of Serial Monitor
    SUART.print(x);         //send the charcater to NANO as ASCII code
  }

 delay(1000);
}
