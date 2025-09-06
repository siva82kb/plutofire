/* Miscellaneous function to handle various tasks.
 *
 * Author: Sivakumar Balasubramanian
 * /

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
  long newPosition = currMech == FPS ? -plutoEncoder.read() : plutoEncoder.read();
  newPosition = newPosition - encOffsetCount;
  if (isActuated) {
    return limbMechAngleScale * (360.0 * newPosition / (enPPRActuated * 4)) - mechOffsetValue[currMech];
  }
  return limbMechAngleScale * ((360.0 * newPosition / (enPPRnonActuated)));
}


/*
 * Function to handle setting of the limb-mechanism scale
 */
void setLimbMechScale() {
  // Set limb-mech scale.
  if (currLimb == NOLIMB) {
    limbMechAngleScale = 1.0;
    limbMechControlScale = 1.0;
  } else if (currLimb == RIGHT) {
    limbMechAngleScale = 1.0;
    limbMechControlScale = currMech == FPS ? -1.0 : 1.0; // ODD: MHCP strange stuff
  } else if (currLimb == LEFT) {
    limbMechAngleScale = currMech == HOC ? 1.0 : -1.0;
    limbMechControlScale = currMech == WFE ? -1.0 : 1.0; // ODD: MHCP strange
  }
  #if SERIALUSB_DEBUG
    SerialUSB.print("Scales: ");
    SerialUSB.print(currLimb);
    SerialUSB.print(",");
    SerialUSB.print(currMech);
    SerialUSB.print(",");
    SerialUSB.print(limbMechAngleScale);
    SerialUSB.print(",");
    SerialUSB.print(limbMechControlScale);
    SerialUSB.print("\n");
  #endif
}

/*
 * Read the status of the PLUTO button.
 */
void readPlutoButtonState(void) {
  bounce.update();
  plutoButton = bounce.read();
  if (bounce.changed()) {
    int deboucedInput = bounce.read();
  }
}

/*
 * Function to read the different sensors of the device and update the 
 * corresponding variables.
 */
void updateSensorData(void) {
  // Read the motor encoder
  ang.add(readEncoderAngle());

  // Read the PLUTO button state
  readPlutoButtonState();
}

void _displaySerialUSB() { }

byte getProgramStatus(byte dtype) {
  // X | DATA TYPE | DATA TYPE | DATA TYPE | CONTROL TYPE | CONTROL TYPE | CONTROL TYPE | CALIB
  return ((dtype << 4) | (ctrlType << 1) | (calib & 0x01));
}

byte getMechActType(void) {
  // CURR MECH | CURR MECH | CURR MECH | CURR MECH | CURR LIMB | CURR LIMB | X | IS ACTUATED
  return ((currMech << 4) | (currLimb << 2) | isActuated);
}

// Update the controller parameters
void setControlParameters(byte ctype, int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  switch (ctype) {
    case POSITION:
      // Position control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      // pcKp = temp.num;
      break;
    case TORQUE:
      // Torque control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      // tcKp = temp.num;
      break;
    case RESIST:
      // updateResistanceControlInfo(sz, inx, payload);
      break;
  }
}

// Set position target [This will be deprecated]
void setTargetOld(byte* payload, int strtInx, byte ctrl) {
  int inx = strtInx;
  floatunion_t temp;
  _assignFloatUnionBytes(inx, payload, &temp);
  if ((ctrl == POSITION) || (ctrl == TORQUE)) {
    target = temp.num;
  } else {
    target = INVALID_TARGET;
  }
}

// Set position target
void setTarget(byte* payload, int strtInx, byte ctrl) {
  int inx = strtInx;
  floatunion_t temp;
  _assignFloatUnionBytes(inx, payload, &temp);
  if ((ctrl == POSITION) 
      || (ctrl == POSITIONLINEAR)
      || (ctrl == TORQUE)
      || (ctrl == TORQUELINEAR)) {
    // target = temp.num;
    int inx = strtInx;
    floatunion_t temp;
    // The are four floats: start position, start time, target, duration.
    // Initial position
    _assignFloatUnionBytes(inx, payload, &temp);
    strtPos = temp.num;
    // Initial time
    inx += 4;
    _assignFloatUnionBytes(inx, payload, &temp);
    strtTime = min(0, temp.num);
    // Target
    inx += 4;
    _assignFloatUnionBytes(inx, payload, &temp);
    target = temp.num;
    // Duration
    inx += 4;
    _assignFloatUnionBytes(inx, payload, &temp);
    reachDur = max(0.0, temp.num);
  } else {
    target = INVALID_TARGET;
  }
}

// Set AAN position target
void setAANTarget(byte* payload, int strtInx) {
  int inx = strtInx;
  floatunion_t temp;
  // The are four floats: start position, start time, target, duration.
  // Initial position
  _assignFloatUnionBytes(inx, payload, &temp);
  strtPos = temp.num;
  // Initial time
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  strtTime = min(0, temp.num);
  // Target
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  target = temp.num;
  // Duration
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  reachDur = max(1.0, temp.num);
}

// Set the control hold parameter
void setControlHold(uint8_t hold) {
  ctrlHold = hold;
  ctrlDynamicsA = 0.0;
  ctrlDynamicsB = 1.0;
  if (ctrlHold == CONTROL_HOLD) {
    ctrlDynamicsA = 1.0;
    ctrlDynamicsB = 0.0;
  } else if (ctrlHold == CONTROL_DECAY) {
    ctrlDynamicsA = 0.99;
    ctrlDynamicsB = 0.0;
  }
  #if SERIALUSB_DEBUG
    SerialUSB.print("\n");
    SerialUSB.print("Control Hold: ");
    SerialUSB.print(ctrlHold);
    SerialUSB.print(",");
    SerialUSB.print("A : ");
    SerialUSB.print(ctrlDynamicsA);
    SerialUSB.print(",");
    SerialUSB.print("B : ");
    SerialUSB.print(ctrlDynamicsB);
    SerialUSB.print("\n");
  #endif
}

// Set object parameters
void setObjectParams(byte* payload, int strtInx) {
  int inx = strtInx;
  floatunion_t temp;
  // The are two floats: object stiffness and position.
  // Stiffness
  _assignFloatUnionBytes(inx, payload, &temp);
  objDelPos = max(0, temp.num);
  // Initial time
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  objPos = temp.num;
  #if SERIALUSB_DEBUG
    SerialUSB.print("\n");
    SerialUSB.print("Object Params: ");
    SerialUSB.print(objDelPos);
    SerialUSB.print(",");
    SerialUSB.print(objPos);
    SerialUSB.print("\n");
  #endif
}

// Generating smooth desired positions from the target.
float generateSmoothDesiredPosition(float x0) {
  static float ypast[] = { 0.0f, 0.0f };
  static float xpast[] = { 0.0f, 0.0f };
  // Check if the input is INVALID_TARGET
  if (x0 == INVALID_TARGET) {
    ypast[0] = 0;
    ypast[1] = 0;
    xpast[0] = 0;
    xpast[1] = 0;
    return INVALID_TARGET;
  }
  // Compute output.
  float _out = (b_filt[0] * x0
                + b_filt[1] * xpast[0]
                + b_filt[2] * xpast[1]
                - a_filt[1] * ypast[0]
                - a_filt[2] * ypast[1]);
  _out *= K_filt;
  // Update memory
  ypast[1] = ypast[0];
  ypast[0] = _out;
  xpast[1] = xpast[0];
  xpast[0] = x0;
  return _out;
}

void initSensorParam() { }
