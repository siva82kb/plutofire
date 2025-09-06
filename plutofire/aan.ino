/* Functions to implement the HOMER AAN control for
 * the PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 30 Jan 2025
 */

// Function to compute assistance support for the given target. This depends 
// on where you are in the AROM, along with where the target is.
float getAssistanceSupportForTarget(float currAng, float currTarget) {
  return 0.0;
}

// Gradual rise function with clipping
float g(float x) {
  return (x >= 1) ? 1.0 : ((x <= 0) ? 0.0 : x);
}

// Minimum jerk trajectory function
float mjt(float t) {
  t = t > 1 ? 1.0 : t;
  t = t < 0 ? 0.0 : t;
  return 6.0 * pow(t, 5) - 15.0 * pow(t, 4) + 10 * pow(t, 3);
}
  
// Compute the AAN desired trajectory.
float getAANDesiredTrajectory() {
  if (target == INVALID_TARGET) return ang.val(0);
  float _t = runTime.num / 1000.0f;
  float _tn = reachDur > 0 ? (_t - initTime) / reachDur : 1.0; 
  return strtPos + (target - strtPos) * mjt(_tn);
}
  
// Compute the linear desired trajectory.
float getLinearDesiredTrajectory() {
  if (target == INVALID_TARGET) return ang.val(0);
  float _t = runTime.num / 1000.0f;
  float _tn = reachDur > 0 ? g((_t - initTime) / reachDur) : 1.0;
  // SerialUSB.println(_tn);
  return strtPos + (target - strtPos) * _tn;
}