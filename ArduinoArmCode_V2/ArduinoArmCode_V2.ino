/*
 * Robotic arm servo driving code
 * 
 * Written by Joshua Galland 29806224 
 * Date 15/09/2020
 */

#include <Servo.h>
#include <AccelStepper.h>

#define VERSION "1.2" 

//-------------------------------------- ALTER THESE VARIABLES FOR TESTING
bool test = false;

//--------------------------------------

//Defining pins
#define ServoHPin 3
#define ServoVPin 5
#define ServoEndPin 4
#define StepIN1 6
#define StepIN2 9
#define StepIN3 10
#define StepIN4 11
#define StepInterfaceType 4

// Defining motor objects
Servo ServoH;  // horozontal movement
Servo ServoV;  // vertical movement
Servo ServoEnd;

// Defining intital positions
float thetaH = 0.0;    // Horizontal arm servo position
float thetaV = 0.0;    // Vertical arm servo position
float thetaE = 0.0;    // End actuator servo postion
float thetaB = 0.0;     // Stepper motor inital position

int ServoWrites(int thetaH_new, int thetaV_new, int thetaE_new){
  Serial.print('(');
  Serial.print(thetaH_new);
  Serial.print(", ");
  Serial.print(thetaV_new);
  Serial.print(", ");
  Serial.print(thetaE_new);
  Serial.println(')');

  if (thetaH_new != thetaH) { ServoH.write(thetaH_new); }
  if (thetaV_new != thetaV) { ServoV.write(thetaV_new); }
  if (thetaE_new != thetaH) { ServoEnd.write(thetaE_new); }
  return thetaH_new, thetaV_new, thetaE_new ;
}

void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); // setup built in LED for flashing
  ServoH.attach(ServoHPin);  // attach horizontal servo pin to servo object
  ServoV.attach(ServoVPin);  // attaching vertical "
  ServoEnd.attach(ServoEndPin); // attaching end "

  while (!Serial); //delay until serial is alive
  Serial.println(F("START " __FILE__ "\r\nVersion " VERSION " from " __DATE__)); // confirm program is running

  // Write initail servo pos
  //thetaH, thetaV, thetaE = ServoWrites(thetaH, thetaV, thetaE); // 0,0,0  
  //thetaH, thetaV, thetaE = ServoWrites(90, 90, 90); // 0,0,0

  delay(500); // wait for servos to reach position

}

void loop(){
}
