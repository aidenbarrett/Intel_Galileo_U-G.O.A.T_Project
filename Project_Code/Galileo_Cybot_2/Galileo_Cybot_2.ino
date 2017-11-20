#include <Ultrasonic.h>

#include <Adafruit_MotorShield.h>

#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <TimerOne.h>
#include <Ultrasonic.h>

#define LEFT 0
#define MIDDLE 1
#define RIGHT 2
#define SENSOR_MAX 4000

#define DEFAULT_SPEED 100

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

const int echoPin[3] = {2,4,6};
const int trigPin[3] = {3,5,7};
long reading[3];
long read_diff = 0;

unsigned long SEC=1000; //1sec = 10^6microsecond
long  pstart=0;
long  pstop=0;
boolean got=false;

int freq=500; //in Hz (1/sec)
int tt=SEC/freq/2;

int rightSpeed, leftSpeed, brakes=1, loopsStopped=0;

void setup() {
  // initialize serial communication:
  //delay(5000);
  Timer1.initialize(SEC);
  Serial.begin(9600);
  for(int i=0;i<3;i++)
  {
    pinMode(trigPin[i],OUTPUT_FAST);
    pinMode(echoPin[i],INPUT_FAST); 
  }
 
  AFMS.begin();
  
  leftMotor->setSpeed(50);
  leftMotor->run(FORWARD);
  // turn on motor
  leftMotor->run(RELEASE);
  
  rightMotor->setSpeed(50);
  rightMotor->run(FORWARD);
  // turn on motor
  rightMotor->run(RELEASE);
  
  leftSpeed = 50;
  rightSpeed = 50;
  
}

void loop()
{
 
//  uint8_t i;
//  reading[LEFT] = getDistance(LEFT,5);
//  if(reading[LEFT]==-1) reading[LEFT]=50;
//  delay(20);
//  reading[MIDDLE] = getDistance(MIDDLE,5);
//  if(reading[MIDDLE]==-1) reading[MIDDLE]=50;
//  delay(20);
//  reading[RIGHT] = getDistance(RIGHT,5);
//  if(reading[RIGHT]==-1) reading[RIGHT]=50;

  reading[LEFT] = getDistanceBasic(LEFT,10);
  if(reading[LEFT]==-1) reading[LEFT]=50;
  delay(20);
  reading[MIDDLE] = getDistanceBasic(MIDDLE,10);
  if(reading[MIDDLE]==-1) reading[MIDDLE]=50;
  delay(20);
  reading[RIGHT] = getDistanceBasic(RIGHT,10);
  if(reading[RIGHT]==-1) reading[RIGHT]=50;
 
  Serial.print("LEFT: ");
  Serial.println(reading[LEFT]);
  Serial.print("MIDDLE: ");
  Serial.println(reading[MIDDLE]);
  Serial.print("RIGHT: ");
  Serial.println(reading[RIGHT]);
  Serial.println();
  Serial.println();
  
  read_diff = reading[RIGHT] - reading[LEFT];
  
  if((reading[MIDDLE]<5)||(reading[LEFT]<5)||(reading[RIGHT]<5))
  {
    leftSpeed = 0;
    rightSpeed = 0;
    loopsStopped = loopsStopped+1;
  }
  else
  {
    loopsStopped = 0;
    if(read_diff > 20)
    {
      //read_diff > 30 means something is much closer to the LEFT than RIGHT  
//      leftSpeed = 150;
//      rightSpeed = 25;
      leftSpeed = DEFAULT_SPEED*(6/3);
      rightSpeed = DEFAULT_SPEED*(3/3);
    }
    else if(read_diff > 15)
    {
      //read_diff > 20 means something is somewhat closer to the LEFT than RIGHT  
//      leftSpeed = 125;
//      rightSpeed = 50;
      leftSpeed = DEFAULT_SPEED*(5/3);
      rightSpeed = DEFAULT_SPEED*(7/6);
    }
    else if(read_diff > 10)
    {
      //read_diff > 5 means something is a bit closer to the LEFT than RIGHT  
//      leftSpeed = 100;
//      rightSpeed = 75;
      leftSpeed = DEFAULT_SPEED*(5/3);
      rightSpeed = DEFAULT_SPEED*(4/3);
    }
    else if(read_diff > -10)
    {
      //read_diff between -5 and +5; just go straight  
//      leftSpeed = 75;
//      rightSpeed = 75;
      leftSpeed = DEFAULT_SPEED*(6/3);
      rightSpeed = DEFAULT_SPEED*(6/3);
    }
    else if(read_diff > -15)
    {
      //read_diff > -20 means something is a bit closer to the RIGHT than LEFT  
//      leftSpeed = 75;
//      rightSpeed = 100;
      leftSpeed = DEFAULT_SPEED*(4/3);
      rightSpeed = DEFAULT_SPEED*(5/3);
    }
    else if(read_diff > -20)
    {
      //read_diff > -30 means something is somewhat closer to the RIGHT than LEFT  
//      leftSpeed = 50;
//      rightSpeed = 125;
      leftSpeed = DEFAULT_SPEED*(7/6);
      rightSpeed = DEFAULT_SPEED*(5/3);
    }
    else
    {
      //read_diff < -20 means something is much closer to the RIGHT than LEFT  
//      leftSpeed = 25;
//      rightSpeed = 150;
      leftSpeed = DEFAULT_SPEED*(3/3);
      rightSpeed = DEFAULT_SPEED*(6/3);
    }
  }
  
  if(loopsStopped > 10)
  {
    turnAround();
    delay(1000);
  }
  
  //leftSpeed = 0;
  //rightSpeed = 0;

  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(leftSpeed);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(rightSpeed);
  
  //delay(1000);
}

long getDistance(const int sensor_num, int num_reads)
{
  long current_read, total_read=0;
  int reads=0;
  
  for(int i=0;i<num_reads;i++)
  {
    delayMicroseconds(2);
    digitalWrite(trigPin[sensor_num], LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin[sensor_num], HIGH);
    delayMicroseconds(5);
    digitalWrite(trigPin[sensor_num], LOW);
  
    current_read = pulseIn(echoPin[sensor_num], HIGH, SENSOR_MAX);
    
    if((current_read > 10) && (current_read < 3500))
    {
      total_read += current_read;
      reads++; 
    }
  }
  
  //Serial.println(reads);
  return (total_read/29/2/reads);
  //return duration;
}

long getDistanceBasic(const int sensor_num, int num_reads)
{
  int reads = 0;
  long ave=0;
  
  for(int i=0;i<num_reads;i++)
  {
    fastDigitalWrite(trigPin[sensor_num], LOW); 
    delayMicroseconds(2);
    fastDigitalWrite(trigPin[sensor_num], HIGH);
    delayMicroseconds(10);
    fastDigitalWrite(trigPin[sensor_num], LOW);// NOT checking for LOW. it seemed to hang here if i used it.
    pstart=micros();
  
   
    while((digitalRead(echoPin[sensor_num])==HIGH)&&((micros()-pstart)<7000))
    {
    }
    pstop=micros();
    
    if((((pstop-pstart)/57)+4)>0)
    {
      ave += ((pstop-pstart)/57)+4;
      reads++;
    }
  }
  ave = ave/reads;
//  if(ave==0)
//  {
//    ave = 50;
//  }
  return ave;
}

void turnAround(void)
{
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(100);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(100);
  //delayMicroseconds(400);
  delay(400);
  
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(150);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  //delayMicroseconds(550);
  delay(550);
  
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  //delayMicroseconds(50);
  delay(50);
  
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  //delayMicroseconds(50);
  delay(50);
  
//  Serial.println("TURNING");
}
