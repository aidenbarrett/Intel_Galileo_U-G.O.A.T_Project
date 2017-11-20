#include <Adafruit_MotorShield.h>

#include <Ultrasonic.h>

/* Ping))) Sensor

   This sketch reads a PING))) ultrasonic rangefinder and returns the
   distance to the closest object in range. To do this, it sends a pulse
   to the sensor to initiate a reading, then listens for a pulse
   to return.  The length of the returning pulse is proportional to
   the distance of the object from the sensor.

   The circuit:
	* +V connection of the PING))) attached to +5V
	* GND connection of the PING))) attached to ground
	* SIG connection of the PING))) attached to digital pin 7

   http://www.arduino.cc/en/Tutorial/Ping

   created 3 Nov 2008
   by David A. Mellis
   modified 30 Aug 2011
   by Tom Igoe

   This example code is in the public domain.

 */

#define LEFT 0
#define MIDDLE 1
#define RIGHT 2

const int pingPin[3] = {2,4,6};
const int pongPin[3] = {3,5,7};
long reading[3];

void setup() {
  // initialize serial communication:
  Serial.begin(9600);
  for(int i=0;i<3;i++)
  {
    pinMode(pingPin[i],OUTPUT);
    pinMode(pongPin[i],INPUT); 
  }
  
}

void loop()
{
  reading[LEFT] = getDistance(LEFT);
  delay(10);
  reading[MIDDLE] = getDistance(MIDDLE);
  delay(10);
  reading[RIGHT] = getDistance(RIGHT);
  delay(10);

  Serial.print("LEFT: ");
  Serial.println(reading[LEFT]);
  Serial.print("MIDDLE: ");
  Serial.println(reading[MIDDLE]);
  Serial.print("RIGHT: ");
  Serial.println(reading[RIGHT]);
  Serial.println();
  Serial.println();

  delay(1000);
}

long getDistance(const int i)
{
  long duration;
  
  digitalWrite(pingPin[i], LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin[i], HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin[i], LOW);

  duration = pulseIn(pongPin[i], HIGH);
  
  return (duration/29/2);
}

