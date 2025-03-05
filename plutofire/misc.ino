

/*
 * Check if there is heartbeat.
 */
void checkHeartbeat() {
  // Check if a heartbeat was recently received.
  if (0.001 * (millis() - lastRxdHeartbeat) < MAX_HBEAT_INTERVAL) {
    // Everything is good. No heart beat related error.
    deviceError.num &= ~NOHEARTBEAT;
  } else {
    // No heartbeat received.
    // Setting error flag.
    deviceError.num |= NOHEARTBEAT;
  }
}

/*
 * Handles errors
 */
void handleErrors() {
  if (deviceError.num != 0) {
    setControlType(NONE);
  }
}


void _assignFloatUnionBytes(int inx, byte* bytes, floatunion_t* temp) {
  temp->bytes[0] = bytes[inx];
  temp->bytes[1] = bytes[inx + 1];
  temp->bytes[2] = bytes[inx + 2];
  temp->bytes[3] = bytes[inx + 3];
}


float readEncoderAngle() {
  long newPosition = plutoEncoder.read() - encOffsetCount;
  if (isActuated) {
    return (360.0 * newPosition / (enPPRActuated * 4));
  }
  return ((360.0 * newPosition / (enPPRnonActuated)));
}


/*
 * Read the status of the PLUTO button.
 */
void readPlutoButtonState(void) {
  bounce.update();
  plutoButton = bounce.read();
}

/*
 * Function to read the different sensors of the device and update the 
 * corresponding variables.
 */
void updateSensorData(void) {
  // Read the motor encoder
  ang.add(readEncoderAngle());

  // Check absolute angle errors.
  if (abs(ang.val(0)) > ABSANGPOSVALUE) {
    deviceError.num |= ANGPOSSENSERR;
  } else {
    deviceError.num &= !ANGPOSSENSERR;
  }

  // Check angle change errors.
  if (abs(ang.val(0) - ang.val(1)) > ABSANGVELVALUE) {
    deviceError.num |= ANGVELSENSERR;
  } else {
    deviceError.num &= !ANGVELSENSERR;
  }

  // Estimated torque from the motor current
  //   torque_est = (analogRead(MOTORCURR) * MCURRGAIN - maxCurrent) * mechnicalConstant;

  // Read the PLUTO button state
  readPlutoButtonState();
}

byte getProgramStatus(byte dtype) {
  // X | DATA TYPE | DATA TYPE | DATA TYPE | CONTROL TYPE | CONTROL TYPE | CONTROL TYPE | CALIB
  return ((dtype << 4) | (ctrlType << 1) | (calib & 0x01));
}

byte getMechActType(void) {
  // CURR MECH | CURR MECH | CURR MECH | CURR MECH | X | X | X | IS ACTUATED
  return ((currMech << 4) | isActuated);
}

// Set position target
void setTarget(byte* payload, int strtInx, byte ctrl) {
  int inx = strtInx;
  floatunion_t temp;

  // Start position and initial time.
  startPos = ang.val(0);
  initTime = 0.001f * runTime.num;

  // Research reach duration
  reachDur = 0;

  // The are two floats: target, duration.
  // Target
  _assignFloatUnionBytes(inx, payload, &temp);
  // Assignt only appropriate target values.
  target = (temp.num >= 0 && temp.num <= mechRangeValue[currMech]) ? temp.num : INVALID_TARGET;
  
  // Reach duration, only if we are in POSITION/POSITIONAAN control, and target is not INVALID.
  if (ctrlType == TORQUE || target == INVALID_TARGET) return;

  // Not TORQUE control, and target is valid.
  // Reach duration
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  reachDur = temp.num >= 0 ? temp.num : 0;
}
