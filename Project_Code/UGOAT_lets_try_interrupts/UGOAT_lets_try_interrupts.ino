#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_PWMServoDriver.h"
#include <Ultrasonic.h>

#define LEFT 0
#define MIDDLE 1
#define RIGHT 2
#define SENSOR_MAX 1200//WAS 7000 before 25/04

#define DEFAULT_SPEED 125
#define STOP_DISTANCE 2

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

const int echoPin[3] = {0,4,6};
const int trigPin[3] = {1,5,7};
long reading[3];
//long read_diff = 0;

unsigned long SEC=1000; //1sec = 10^6microsecond
long  pstart=0;
long  pstop=0;
boolean got=false;

int rightSpeed, leftSpeed, loopsStopped=0;

int pinInt0 = 2, state=1;;  

void setup() {
  // initialize serial communication:
  //delay(5000);
  //Timer1.initialize(SEC);
  Serial.begin(9600);
  for(int i=0;i<3;i++)
  {
    pinMode(trigPin[i],OUTPUT_FAST);
    pinMode(echoPin[i],INPUT_FAST); 
  }
 
  //pinMode(1,INPUT);
  attachInterrupt(pinInt0, freeze, CHANGE);
 
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

void freeze()
{
  while(digitalRead(pinInt0)==0)
  {
    leftMotor->run(RELEASE);
    //leftMotor->setSpeed(0);
    rightMotor->run(RELEASE);
    //rightMotor->setSpeed(0);
    delay(500);
  }
}

void loop()
{
//  while(digitalRead(1)==LOW)
//  {
//    leftMotor->run(RELEASE);
//    //leftMotor->setSpeed(0);
//    rightMotor->run(RELEASE);
//    //rightMotor->setSpeed(0);
//  }

  reading[LEFT] = getDistanceBasic(LEFT,5);
  if(reading[LEFT]==-1) reading[LEFT]=50;
  delay(20);
//  reading[MIDDLE] = getDistanceBasic(MIDDLE,5);
//  if(reading[MIDDLE]==-1) reading[MIDDLE]=50;
//  delay(20);
  reading[RIGHT] = getDistanceBasic(RIGHT,5);
  if(reading[RIGHT]==-1) reading[RIGHT]=50;
 
  Serial.print("LEFT: ");
  Serial.println(reading[LEFT]);
  Serial.print("MIDDLE: ");
  Serial.println(reading[MIDDLE]);
  Serial.print("RIGHT: ");
  Serial.println(reading[RIGHT]);
  Serial.println();
  Serial.println();
  
//  read_diff = reading[RIGHT] - reading[LEFT];
  
  //if((reading[MIDDLE]<STOP_DISTANCE)||(reading[LEFT]<STOP_DISTANCE)||(reading[RIGHT]<1))
  if((reading[LEFT]<STOP_DISTANCE)||(reading[RIGHT]<1))
  {
    leftSpeed = 0;
    rightSpeed = 0;
    loopsStopped = loopsStopped+1;
  }
  else
  {
    loopsStopped = 0;
    
//    leftSpeed = (read_diff)*(6) + 125;
//    rightSpeed = (read_diff)*(-6) + 125;

//    leftSpeed = 255 - (reading[RIGHT]*6);
//    rightSpeed = 255 - (reading[LEFT]*6);

    leftSpeed = pow(reading[RIGHT],1.75);
    rightSpeed = pow(reading[LEFT],1.75);
    
    if(leftSpeed<25)leftSpeed=25;
    else if(leftSpeed>255)leftSpeed=255;
    
    if(rightSpeed<25)rightSpeed=25;
    else if(rightSpeed>255)rightSpeed=255;
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
  
   
    while((digitalRead(echoPin[sensor_num])==HIGH)&&((micros()-pstart)<SENSOR_MAX))
    {
    }
    pstop=micros();
    
    //if((((pstop-pstart)/57)+1)>0)
    //{
      //ave += ((pstop-pstart)/57)+4;
      ave += ((pstop-pstart)/30);
      //ave += ((pstop-pstart)/20)+1;
      reads++;
    //}
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
  delay(400);
  
  leftMotor->run(BACKWARD);
  leftMotor->setSpeed(150);
  rightMotor->run(FORWARD);
  rightMotor->setSpeed(150);
  delay(550);
  
  leftMotor->run(FORWARD);
  leftMotor->setSpeed(200);
  rightMotor->run(BACKWARD);
  rightMotor->setSpeed(200);
  delay(50);
  
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(50);
}

