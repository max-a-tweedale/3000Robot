


/*

   IMPORTANT: This program receives data over the serial monitor. This data must be in the form of:
   "<HelloWorld,10.0,90.7,81.5,80.0,1000,2000,1000,1000>"
   Where this is the equivalent of <string, float, float, float, float, int, int, int, int>
   This is interpreted as <instruction, jointAngleEE (degrees), jointAngle1, jointAngle2, jointAngle3, movementTimeEE (milli seconds), movementTime1, movementTime2, movementTime3>

*/
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Library Includes.                                                              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

// include these libraries for using the servo add on board. Taken from servo example code
#include <Arduino.h>
#include <Servo.h>
#include "ServoEasing.h"
/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Definitions                                                                    *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
#define VERSION "3.1"
const int SERVO1MAX = 180;
const int SERVO1MIN = 0;
const int SERVO2MAX = 145;
const int SERVO2MIN =  80;
const int SERVO3MAX = 120;
const int SERVO3MIN = 60;
const int SERVOEEMAX = 120;
const int SERVOEEMIN = 60;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Instantiating Servos                                                           *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
ServoEasing servo1;
ServoEasing servo2;
ServoEasing servo3;
ServoEasing servo4;

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  Variables                                                                      *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//-------- Variables for receiving serial data -------------
const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
char messageFromPC[buffSize] = {0};

// -------- Variables to hold time -------------
unsigned long curMillis; // Variable for current time
//unsigned long general_timer;

// -------- Variables to hold the parsed data -------------
float floatFromPC0 = 90.0; // initial values are mid range for joint angles
float floatFromPC1 = 90.0;
float floatFromPC2 = 90.0;
float floatFromPC3 = 90.0;
int intFromPC0 = 1000; // inital values are acceptable movement times
int intFromPC1 = 1000;
int intFromPC2 = 1000;
int intFromPC3 = 1000;
float last_servoAngle_q1 = floatFromPC1; // initial values are mid range for joint angles
float last_servoAngle_q2 = floatFromPC2;
float last_servoAngle_q3 = intFromPC3;
float last_servoAngle_EE = floatFromPC0;


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  START OF PROGRAM (Setup)                                                       *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void setup() {
  //flash LED so we know we are alive
  blinkLED();

  //Setup pins
  pinMode(LED_BUILTIN, OUTPUT); // setup built in LED for flashing

  //Begin serial communications
  Serial.begin(9600);

  //Wait for serial communications to start before continuing
  while (!Serial); //delay for Leonardo

  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ "\r\nVersion " VERSION " from " __DATE__));

  //Attach servo to pin
  servo1.attach(3);
  servo2.attach(6);
  servo3.attach(9);
  servo4.attach(11);

  servo1.setEasingType(EASE_CUBIC_IN_OUT);
  servo2.setEasingType(EASE_CUBIC_IN_OUT);
  servo3.setEasingType(EASE_CUBIC_IN_OUT);
  servo4.setEasingType(EASE_CUBIC_IN_OUT);

  servo1.setSpeed(40);
  servo2.setSpeed(40);
  servo3.setSpeed(40);
  servo4.setSpeed(40);
  //Default positions for servos
   servo1.write(0);
   servo2.write(90);
   servo3.write(90);
   servo4.write(90);
   


  // Just wait for servos to reach position
  delay(500); // delay() is OK in setup as it only happens once

  // tell the PC we are ready
  Serial.println("<Arduino is ready>");

}

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
  MAIN PROGRAM (Loop)                                                            *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

void loop() {
  // This part of the loop for the serial communication is not inside a timer -> it happens very quickly
  curMillis = millis(); // get current time
  getDataFromPC(); // receive data from PC and save it into inputBuffer

  // need if statement -> flag to say if new data is available
  if (newDataFromPC == true)
  {
    processMessageFromPC(); // Processes text message from PC
    actionInstructionsFromPC(); // Arrange for things to move, beep, light up
    replyToPC(); // Reply to PC
  }



}


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR MAKING THINGS MOVE OR LIGHT UP OR BEEP!              *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Blink LED~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

void blinkLED() {
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Action the instructions from the PC~~~~~~~~~~~~~~~~~~~~~~~

void actionInstructionsFromPC() {
  //Local variables
  // -- joint angles
  float servoAngle_q1 = floatFromPC1;
  float servoAngle_q2 = floatFromPC2;
  float servoAngle_q3 = floatFromPC3;
  float servoAngle_EE = floatFromPC0;
  // -- joint speeds
  int servoTime_q1 = intFromPC1;
  int servoTime_q2 = intFromPC2;
  int servoTime_q3 = intFromPC3;
  int servoTime_EE = intFromPC0;
  

  // Check if the joint angle has changed!
  if (servoAngle_q1 != last_servoAngle_q1) {
    Serial.println(F("Servo 1 moving to position using interrupts"));
    if (servoAngle_q1<SERVO1MIN){
      servo1.startEaseTo(SERVO1MIN);
    }
    else if (servoAngle_q1>SERVO1MAX){
      servo1.startEaseTo(SERVO1MAX);
    }
    else {
    servo1.startEaseTo(servoAngle_q1);
    }
  }

  if (servoAngle_q2 != last_servoAngle_q2) {
    Serial.println(F("Servo 2 moving to position using interrupts"));
    if (servoAngle_q2<SERVO2MIN){
      servo2.startEaseTo(SERVO2MIN);
    }
    else if (servoAngle_q2>SERVO2MAX){
      servo2.startEaseTo(SERVO2MAX);
    }
    else {
    servo2.startEaseTo(servoAngle_q2);
    }
  }
  
  if (servoAngle_q3 != last_servoAngle_q3) {
    Serial.println(F("Servo 3 moving to position using interrupts"));
    if (servoAngle_q3<SERVO3MIN){
      servo3.startEaseTo(SERVO3MIN);
    }
    else if (servoAngle_q3>SERVO3MAX){
      servo3.startEaseTo(SERVO3MAX);
    }
    else {
    servo3.startEaseTo(servoAngle_q3);
    }
  }

  if (servoAngle_EE != last_servoAngle_EE) {
    Serial.println(F("Servo EE moving to position using interrupts"));
    servo4.startEaseTo(servoAngle_EE);
  }

  // Store current joint angle
  last_servoAngle_q1 = servoAngle_q1;
  last_servoAngle_q2 = servoAngle_q2;
  last_servoAngle_q3 = servoAngle_q3;
  last_servoAngle_EE = servoAngle_EE;

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
              FUNCTIONS FOR RECEIVING DATA VIA SERIAL MONITOR                    *
* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

//~~~~~~~~~~~~~Fuction: Receive data with start and end markers~~~~~~~~~~~~~~~~~~

void getDataFromPC() {

  // This function receives data from PC and saves it into inputBuffer

  if (Serial.available() > 0 && newDataFromPC == false) {

    char x = Serial.read();

    // the order of these IF clauses is significant

    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }

    if (readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) {
      bytesRecvd = 0;
      readInProgress = true;
    }
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Split data into known component parts~~~~~~~~~~~~~~~~~~

void parseData() {

  // split the data into its parts

  char * strtokIndx; // this is used by strtok() as an index

  strtokIndx = strtok(inputBuffer, ",");     // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC

  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  floatFromPC0 = atof(strtokIndx);     // convert this part to a float -> to convert to an integer we would use atoi

  strtokIndx = strtok(NULL, ",");
  floatFromPC1 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  floatFromPC2 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  floatFromPC3 = atof(strtokIndx);     // convert this part to a float

  strtokIndx = strtok(NULL, ",");
  intFromPC0 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC1 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC2 = atoi(strtokIndx);     // convert this part to a int

  strtokIndx = strtok(NULL, ",");
  intFromPC3 = atoi(strtokIndx);     // convert this part to a int

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Send message back to PC~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


void replyToPC() {

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print(F("<Msg "));
    Serial.print(messageFromPC);
    Serial.print(F(" floatFromPC0 "));
    Serial.print(floatFromPC0);
    Serial.print(F(" floatFromPC1 "));
    Serial.print(floatFromPC1);
    Serial.print(F(" floatFromPC2 "));
    Serial.print(floatFromPC2);
    Serial.print(F(" floatFromPC3 "));
    Serial.print(floatFromPC3);
    Serial.print(F(" intFromPC0 "));
    Serial.print(intFromPC0);
    Serial.print(F(" intFromPC1 "));
    Serial.print(intFromPC1);
    Serial.print(F(" intFromPC2 "));
    Serial.print(intFromPC2);
    Serial.print(F(" intFromPC3 "));
    Serial.print(intFromPC3);
    Serial.print(F(" Time "));
    Serial.print(curMillis / 1000); // divide by 512 is approx = half-seconds
    Serial.println(F(">"));
  }
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

//~~~~~~~~~~~~~Fuction: Process the string message from the PC~~~~~~~~~~~~~~~~~~~~


void processMessageFromPC() {

  // this illustrates using different inputs to call different functions
  // strcmp compares two strings and returns zero if the strings are equal

  if (strcmp(messageFromPC, "LED") == 0) {
    blinkLED();
  }

}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
