/*
 * Contains motor testing scripts that are run on initialisation
 */
int testBStepper(AccelStepper stepper) {
  stepper.moveTo(-400);
  // Step the motor with constant speed as set by setSpeed():
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  stepper.moveTo(0);
  // Step the motor with constant speed as set by setSpeed():
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  return 0;
}
int testEndActuator(Servo s) {
    
    int Close = 20;
    int Open = 80;
    s.write(Close);
    delay(1000);
    // OPEN
    for (int p = Close; p<Open; p += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    s.write(p);              // tell servo to go to position in variable 'pos'
    delay(30);                       // waits 15ms for the servo to reach the position
    }
    // CLOSE
    for (int p = Open; p>=Close; p -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s.write(p);              // tell servo to go to position in variable 'pos'
      delay(10);                       // waits 15ms for the servo to reach the position
    }
    return Close;
}
int testHServo(Servo s) {
    int del= 10;
    int low = 70;
    int high = 150;
   // OPEN
    for (int p = low; p<high; p += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s.write(p);              // tell servo to go to position in variable 'pos'
      Serial.print("Theta is ");
      Serial.println(p);
      delay(del);                       // waits 15ms for the servo to reach the position
    }
    // CLOSE
    for (int p = high; p>=low; p -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s.write(p);              // tell servo to go to position in variable 'pos'
      Serial.print("Theta is ");
      Serial.println(p);
      delay(del);                       // waits 15ms for the servo to reach the position
    }
    return low;
}
int testVServo(Servo s) {
    int del= 10;
    int low = 80;
    int high = 150;
   // OPEN
    for (int p = low; p<high; p += 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s.write(p);              // tell servo to go to position in variable 'pos'
      Serial.print("Theta is ");
      Serial.println(p);
      delay(del);                       // waits 15ms for the servo to reach the position
    }
    // CLOSE
    for (int p = high; p>=low; p -= 1) { // goes from 0 degrees to 180 degrees
      // in steps of 1 degree
      s.write(p);              // tell servo to go to position in variable 'pos'
      Serial.print("Theta is ");
      Serial.println(p);
      delay(del);                       // waits 15ms for the servo to reach the position
    }
    return low;
}
