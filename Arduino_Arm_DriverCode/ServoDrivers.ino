
/*
 * This file contains functions for arm driver code
 */


int DriveServo(int goal, int start, int currentPos){
  
  
}

int ServoWrites(int thetaH_new, int thetaV_new, int thetaE_new){
  actH(thetaH_new);
  actV(thetaV_new);
  actE(thetaE_new);
  return thetaH_new, thetaV_new, thetaE_new ;
}

//Write theta to actuators
int actH(int theta) {
  //Theta bounds are -30 and 50
  int cal = 100; // ServoH is vertical at 100
  if (theta >= -30 && theta <= 50) { // 70 and 150 abs pos
    ServoH.write(theta+cal);  }
  else {
    Serial.println("ERROR function actH - theta out of bounds"); }
  return theta;
}
int actV(int theta) {
  // Theta bounds between -35 and 35
  int cal = 115; // ServoV is horizontal at 115
  if (theta >= -35 && theta <= 35) { // 80 -> 150 abs pos bound
    ServoV.write(theta+cal); }
  else {
    Serial.println("ERROR function actV - theta out of bounds"); }
  return theta;
}
int actE(int theta) {
  // Theta Bounds between 0 and 95
  int cal = 20; // ServoE is closed at 20 and open at 115
  if (theta >= 0 && theta <= 95) {
    ServoEnd.write(theta+cal); }
  else {
    Serial.println("ERROR function actE - theta out of bounds"); }
  return theta;
}
