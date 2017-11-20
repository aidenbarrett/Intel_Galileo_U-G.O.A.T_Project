
#include <Wire.h>
#include "rgb_lcd.h"

rgb_lcd lcd;

long duration;
long cm;

void setup() {
    
    // set up the LCD's number of columns and rows:
    lcd.begin(16, 2);
    // Print a message to the LCD.
    lcd.setRGB(255, 0, 0);
    lcd.display();
}

void loop() {
    lcd.clear();
    lcd.print("Sonar 101");
    lcd.setCursor(0, 2); 
    pingthing(7);
    cm = microsecondsToCentimeters(duration);
    lcd.print(cm);
    lcd.print("cm");
    
    delay(200);

}

long microsecondsToCentimeters(long microseconds)
{
  return microseconds / 29 / 2;
}

void pingthing(int pingPin)
{
  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);
  
  //delay(100);
}
