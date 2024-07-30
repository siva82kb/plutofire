

void _assignFloatUnionBytes(int inx, byte* bytes, floatunion_t* temp) {
  temp->bytes[0] = bytes[inx];
  temp->bytes[1] = bytes[inx + 1];
  temp->bytes[2] = bytes[inx + 2];
  temp->bytes[3] = bytes[inx + 3];
}


float readEncoderAngle() {
    long newPosition = plutoEncoder.read() - encOffsetCount;
    // if (newPosition != oldPosition) {
    //     oldPosition = newPosition;
    // }

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
  if (bounce.changed()) {
    int deboucedInput = bounce.read();
    if (deboucedInput == LOW) {
    //   led.setColor(RGBLed::RED);
    //   plutoButton = 1;
    //   ledState = !ledState;             // SET ledState TO THE OPPOSITE OF ledState
    //   digitalWrite(LED_PIN, ledState);  // WRITE THE NEW ledState
    }
  }
}

/*
 * Function to read the different sensors of the device and update the 
 * corresponding variables.
 */
void updateSensorData(void) {
  // Read the motor encoder
  ang.add(readEncoderAngle());

  // Estimated torque from the motor current
//   torque_est = (analogRead(MOTORCURR) * MCURRGAIN - maxCurrent) * mechnicalConstant;

  // Read the PLUTO button state
  readPlutoButtonState();
}

void _displaySerialUSB() {
  // put your main code here, to run repeatedly:
  Serial.print(ang.val((byte)0, false));
  Serial.print(" ");
  Serial.print(angvel.val((byte)0, false));
  Serial.print(" ");
  Serial.print(mcurr.val((byte)0, false));
  Serial.print(" ");
  Serial.print(torque.val((byte)0, false));
  Serial.print("\n");
}

byte getProgramStatus(byte dtype) {
  // X | DATA TYPE | DATA TYPE | DATA TYPE | CONTROL TYPE | CONTROL TYPE | CONTROL TYPE | CALIB
  return ((dtype << 4) | (ctrlType << 1) | (calib & 0x01));
}

byte getMechActType(void) {
    // CURR MECH | CURR MECH | CURR MECH | CURR MECH | X | X | X | IS ACTUATED
    return ((currMech << 4) | isActuated);
}

// Update sensor parameter using  byte array
void updateSensorParameter(int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  // Torque sensor.
  _assignFloatUnionBytes(inx, payload, &temp);
  torqParam.m = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  torqParam.c = temp.num;
  inx += 4;
}
// Update sensor parameter using  byte array
void updateResistanceControlInfo(int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  // Torque sensor.
  _assignFloatUnionBytes(inx, payload, &temp);
  kp = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  kd = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  km = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  neutral_ang = temp.num;
  inx += 4;
}


// Update sensor param in the different buffers
void updateBufferSensorParam(bool reset) {
  if (reset == true) {
    torqParam.m = 1.0;
    torqParam.c = 0.0;
  }
  setTorqSensorParam();
}

// Update sensor parameters for torque sensor
void setTorqSensorParam() {
  torque.setconvfac(torqParam.m, torqParam.c);
}

// Update sensor parameters for angular velocity sensor
void setAngleVelSensorParam() {
  angvel.setconvfac(angvelParam.m, angvelParam.c);
}

// Update sensor parameters for motor current sensor
void setMCurrSensorParam() {
  mcurr.setconvfac(mcurrParam.m, mcurrParam.c);
}




// Update the controller parameters
void setControlParameters(byte ctype, int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  switch (ctype) {
    case ACTIVE:
      // Admittance control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      acKp = temp.num;
      break;
    case POSITION:
      // Position control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      pcKp = temp.num;
      break;
    case TORQUE:
      // Torque control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      tcKp = temp.num;
      break;
    case RESIST:
      updateResistanceControlInfo(sz, inx, payload);
  }
}

// Update the target parameters
void setTargetParameters(byte ctype, int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  switch (ctype) {
    case POSITION:
      // Position control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      desAng = temp.num;
      break;
    case TORQUE:
      // Torque control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      desTorq = temp.num;
      break;

    case RESIST:

      break;
  }
}


void initSensorParam() {
  torqParam.m = 1.0;
  torqParam.c = 0.0;
  angvelParam.m = 1.0;
  angvelParam.c = 0.0;
  mcurrParam.m = MCURRGAIN;
  mcurrParam.c = MCURROFFSET;
  setTorqSensorParam();
  setAngleVelSensorParam();
  setMCurrSensorParam();
}

// Check for any errors in the operation
void checkForErrors() {
  error = NOERR;
  uint16_t _errval = 0;

  // Check sensor values.


  if (abs(ang.valf(0, false)) > 120.0) {
    _errval = _errval | ANGSENSERR;
  }
  //  if (abs(angvel.valf(0, false)) > 500.) {
  //    _errval = _errval | VELSENSERR;
  //  }
  //  if (abs(torque.valf(0, false)) > 4.0) {
  //    _errval = _errval | TORQSENSERR;
  //  }
  //  if (abs(mcurr.valf(0, false)) > 10) {
  //    _errval = _errval | MCURRSENSERR;
  //



  // Update error status
  if (_errval != 0) {
    sendPWMToMotor(0);
    error = YESERR;
  }

  // Update error values
  errorval[0] = _errval & 0x00FF;
  errorval[1] = (_errval >> 8) & 0x00FF;
}

// void startCalibMode() {
//     ctrlType = CALIBRATION;
//     calibCount = 0;
//     angvelParam.m = 1.0;
//     angvelParam.c = 0.0;
//     mcurrParam.m = 1.0;
//     mcurrParam.c = 0.0;
//     setAngleVelSensorParam();
//     setMCurrSensorParam();
// }

// void updateExitCalibMode() {
//   // Reset encoder count

//   // Update gains for vel. and curr sensors
//   angvelParam.m = ANGVELGAIN;
//   angvelParam.c = 0;
//   mcurrParam.m = MCURRGAIN;
//   mcurrParam.c = MCURROFFSET;
//   setAngleVelSensorParam();
//   setMCurrSensorParam();

//   // Exit calibration mode.
//   ctrlType = NONE;
// }



// void calibProcess() {
//     // Check counter
//     ctrlType = NONE;
//     // Set the encoder offset count value.
//     encOffsetCount = plutoEncoder.read();
//     initSensorParam();
//     if (calibCount++ == maxCalibCount) {
//         // Calibration count done.
//         // Update parameters and exit calibration mode.
//         updateExitCalibMode();
//         // Update calibration.
//         calib = YESCALIB;
//     } else {
//         writeSensorStream();
//     }
// }

//void handleError() {
//  // First clear any control mode.
//  ctrlType = NONE;
//
//  if (stream) {
//    writeSensorStream();
//  }
//}
//
//void handleNormal() {
//  if (stream) {
//    writeSensorStream();
//  }
//  // Update control law.
//  if (ctrlType != NONE) {
//    updateControlLaw();
//  } else {
//      digitalWrite(ENABLE, HIGH);
//      digitalWrite(CW, LOW);
//      analogWrite(PWM, 10);
//  }
//}
