
/*
 * Main function file
 */


bool inPosition(int thetaH_new, int thetaV_new, int thetaB_new){
  bool H = (thetaH_new == thetaH);
  bool V = (thetaV_new == thetaV);
  bool B = (0 == StepB.distanceToGo());
 return H && V && B;
}
