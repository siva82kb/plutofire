

void _assignFloatUnionBytes(int inx, byte* bytes, floatunion_t* temp) {
  temp->bytes[0] = bytes[inx];
  temp->bytes[1] = bytes[inx + 1];
  temp->bytes[2] = bytes[inx + 2];
  temp->bytes[3] = bytes[inx + 3];
}

 
float encAngle() {
    long newPosition = myEnc.read() - encOffsetCount;
    // Assuming enPPRActuated and enPPRnonActuated are constants or change infrequently,
    // consider moving the divisor calculation outside if their values are updated rarely.
    float divisor = isActuated ? (enPPRActuated * 4.0) : enPPRnonActuated;
    return 360.0 * newPosition / divisor;
}




void updateIO(){
    bounce.update();
  inputButton = bounce.read();
  if ( bounce.changed() ) {
    int deboucedInput = bounce.read();
    
    if ( deboucedInput == LOW ) {
//      led.setColor(RGBLed::RED);
      inputButton =  1;
     // delay(1);
      ledState = !ledState; // SET ledState TO THE OPPOSITE OF ledState
      digitalWrite(LED_PIN,ledState); // WRITE THE NEW ledState
      
    }
  }
}
void updateSensorData() {
  
  ang.add(encAngle());
 angvel.add(analogRead(MOTORSPEED));
 mcurr.add(analogRead(MOTORCURR));
  updateIO();

  transformed_torque = (analogRead(MOTORCURR)*MCURRGAIN - maxCurrent)* mechnicalConstant;
 
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
  Kp = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  Kd = temp.num;
  inx += 4;
   _assignFloatUnionBytes(inx, payload, &temp);
  Ki = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  maxTorq = temp.num;
  myPID.SetMode(AUTOMATIC);
  myPID.SetTunings(Kp, Ki, Kd); 

//  Serial.print("recived control info");
//    Serial.print(Kp);


  }

// Update sensor parameter using  byte array
void PIDinput(int sz, int strtInx, byte* payload) {
  int inx = strtInx;
  floatunion_t temp;
  // Torque sensor.
  _assignFloatUnionBytes(inx, payload, &temp);
 
  Setpoint = temp.num;
  inx += 4;
  _assignFloatUnionBytes(inx, payload, &temp);
  if( maxTorq != temp.num){
      maxTorq = temp.num;
      myPID.SetOutputLimits(-5*maxTorq,5*maxTorq);

  }
  
//  Serial.println("recived target info");
// Serial.println(maxTorq);

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
void setTorqSensorParam()
{
    torque.setconvfac(torqParam.m, torqParam.c);
}

// Update sensor parameters for angular velocity sensor
void setAngleVelSensorParam()
{
    angvel.setconvfac(angvelParam.m, angvelParam.c);
}

// Update sensor parameters for motor current sensor
void setMCurrSensorParam()
{
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
      PIDinput(inx, payload, payload);
      break;
    case TORQUE:
      // Torque control gain
      _assignFloatUnionBytes(inx, payload, &temp);
      tcKp = temp.num;
      break;
      case RESIST:
       updateResistanceControlInfo(sz,inx,payload);
       break;
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
      PIDinput(sz,inx,payload);
    break;
  }
}




// Check for any errors in the operation
void If checkForErrors() {
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





void calibProcess() {
  // Check counter
  ctrlType = NONE;
  encOffsetCount =    myEnc.read();

 
}

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
//}z
