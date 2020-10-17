/*
 * Robotic arm servo driving code
 * 
 * Written by Joshua Galland 29806224 
 * Date 15/09/2020
 */

#include <Servo.h>
#include <AccelStepper.h>

//Defining pins
#define ServoHPin 3
#define ServoVPin 4
#define ServoEndPin 5
#define StepIN1 6
#define StepIN2 9
#define StepIN3 10
#define StepIN4 11
#define StepInterfaceType 4


// Defining motor objects
Servo ServoH;  // horozontal movement
Servo ServoV;  // vertical movement
Servo ServoEnd;
AccelStepper StepB = AccelStepper(StepInterfaceType, StepIN1, StepIN2, StepIN3, StepIN4);

// Defining intital positions
int thetaH = 0;    // Horizontal arm servo position
int thetaV = 0;    // Vertical arm servo position
int thetaE = 0;    // End actuator servo postion
int thetaB = 0;     // Stepper motor inital position

void setup() {
  Serial.begin(9600);
  ServoH.attach(ServoHPin);  // attach horizontal servo pin to servo object
  ServoV.attach(ServoVPin);  // attaching vertical "
  ServoEnd.attach(ServoEndPin); // attaching end "
  StepB.setMaxSpeed(500);
  StepB.setAcceleration(100);
  StepB.setCurrentPosition(0);

  // Write initail servo pos
  thetaH, thetaV, thetaE = ServoWrites(thetaH, thetaV, thetaE); // 0,0,0
  //  Testing Of motors
  //testEndActuator(ServoEnd);
  //testHServo(ServoH);
  //testVServo(ServoV);
  testBStepper(StepB);
  
}

void loop() {

  
  
}
