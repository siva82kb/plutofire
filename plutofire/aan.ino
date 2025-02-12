/* Functions to implement the HOMER AAN control for
 * the PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 30 Jan 2025
 */

// Function to compute assistance support for the given target. This depends 
// on where you are in the AROM, along with where the target is.
float getAssistanceSupportForTarget(float currAng, float currTarget) {
  // Invalid targets get zero support.
  if (target == INVALID_TARGET) return 0.0;
  // Valid tatget.
  if (currTarget <= 1.0 * aRom[0]) {
    SerialUSB.print(currAng);
    SerialUSB.print(", ");
    SerialUSB.print(aRom[0]);
    SerialUSB.print(", ");
    SerialUSB.print(bndryDelta);
    SerialUSB.print(", ");
    SerialUSB.print(- (currAng - 1.0 * aRom[0] - bndryDelta) / bndryDelta);
    SerialUSB.print("\n");
    return g(- (currAng - 1.0 * aRom[0] - bndryDelta) / bndryDelta);
  } else if (currTarget >= 1.0 * aRom[1]) {
    SerialUSB.print(currAng);
    SerialUSB.print(", ");
    SerialUSB.print(aRom[1]);
    SerialUSB.print(", ");
    SerialUSB.print(bndryDelta);
    SerialUSB.print(", ");
    SerialUSB.print((currAng - 1.0 * aRom[1] + bndryDelta) / bndryDelta);
    SerialUSB.print("\n");
    return g((currAng - 1.0 * aRom[1] + bndryDelta) / bndryDelta);
  }
  return 0.0;
}

// Gradual rise function with clipping
float g(float x) {
  return (x >= 1) ? 1.0 : (x <= 0) ? 0.0 : x;
}