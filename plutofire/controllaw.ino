/* Functions to implement the different controllers for the 
 *  PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 31 Aug 2018
 */

void updateControlLaw() {
    // float _currPWM = 0.0;
    // float _currError = 0.0;
    float _currI = 0.0;
    float _currPWM = 0.0;
    float _prevPWM = control.val(0);
    // float ffCurr = 0.0;
    // float ffPWM = 0.0;
    // float _delPWMSign;
    // If control is NONE. Switch off control and move on.
    if (ctrlType == NONE) {
        // Switch off controller.
        digitalWrite(ENABLE, LOW);
        control.add(0.0);
        return;
    }
    // Else we need to take the appropriate action.
    switch (ctrlType) {
        case POSITION:
            // Position control.
            _currI = controlPosition();
            _currPWM = boundPositionControl(convertCurrentToPWM(_currI));
            break;
        case TORQUE:
            // Feedfoward torque control.
            _currI = target.val(0) / mechnicalConstant;
            _currPWM = convertCurrentToPWM(_currI);
            break;
        case RESIST:
            // PD resistance control
            // desTorq = -(kp * (_ang - neutral_ang) + kd * 0.02 * _ang - kd * 0.02 * prev_ang);
            // cur = desTorq / mechnicalConstant;
            // __currpwm = constrain(map(abs(cur), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
            // prev_ang = _ang;
            break;
    }
    // Limit the rate of change of PWM
    _currPWM = rateLimitValue(_currPWM, _prevPWM, MAXDELPWM);
    // Clip PWM value
    _currPWM = min(MAXPWM, max(-MAXPWM, _currPWM));
    // Send PWM value to motor controller & update control.
    sendPWMToMotor(_currPWM);
    control.add(_currPWM);
}

// Position controller implementation
float controlPosition() {
    float _currang = ang.val(0);
    float _currtgt = target.val(0);
    float _prevang = ang.val(1);
    float _prevtgt = target.val(1);
    float _curr;
    float _currerr, _preverr;
    float _errsum = errsum.val(0);

    // Check if position control is disabled.
    if (_currtgt == INVALID_TARGET) {
        err.add(0.0);
        errdiff.add(0.0);
        errsum.add(0.0);
        return 0.0;
    }

    // Update error related information.
    // Current error
    _currerr = _currtgt - _currang;
    // Ignore small errors.
    _currerr = (abs((_currerr)) <= POS_CTRL_DBAND) ? 0.0 : _currerr;
    // Previous error
    _preverr = (_prevtgt != INVALID_TARGET) ? _prevtgt - _prevang : _currerr;
    // Error sum.
    _errsum = 0.9999 * _errsum + _currerr;
    float _intlim = ctrlBound * INTEGRATOR_LIMIT / pcKi;
    _errsum = min(_intlim, max(-_intlim, _errsum));
    
    // Compute the PID control current
    _curr = pcKp * (_currerr) + pcKi * _errsum + pcKd * (_currerr - _preverr);

    // Log error information
    err.add(_currerr);
    errdiff.add(_currerr - _preverr);
    errsum.add(_errsum);

    return _curr;
}

// Bound the position control output.
float boundPositionControl(float pwm_value) {
    if (pwm_value > MAXPWM) {
        return ctrlBound * MAXPWM;
    } else if (pwm_value < - MAXPWM) {
        return - ctrlBound * MAXPWM;
    }
    return ctrlBound * pwm_value;
}

// Rate limit a variable.
float rateLimitValue(float curr, float prev, float rlim) {
    float _del = curr - prev;
    if (_del >= rlim) {
        return prev + rlim;
    } else if (_del <= -rlim) {
        return prev - rlim;
    }
    return curr;
} 

// Convert current to PWM
float convertCurrentToPWM(float current) {
    float _sign = current >= 0? +1 : -1;
    return _sign * map(abs(current), 0, maxCurrent, MINPWM, MAXPWM);
}

void sendPWMToMotor(float pwm) {
    if (pwm > 0) {
        // Move counter clockwise
        digitalWrite(ENABLE, HIGH);
        digitalWrite(CW, LOW);
        analogWrite(PWM, min(MAXPWM, max(pwm, MINPWM)));
        // return min(MAXPWM, max(pwm, MINPWM));
    } else {
        // Move counter clockwise
        digitalWrite(ENABLE, HIGH);
        digitalWrite(CW, HIGH);
        analogWrite(PWM, min(MAXPWM, max(-pwm, MINPWM)));
        // return -min(MAXPWM, max(-pwm, MINPWM));
    }
}

void setControlParamForMech() {
  // Cannot set controller parameters for NOMECH
  if (currMech == NOMECH) {
    pcKp = 0;
    pcKd = 0;
    pcKi = 0;
    return;  
  }
  // Else
  pcKp = mechKp[currMech - 1];
  pcKd = mechKd[currMech - 1];
  pcKi = mechKi[currMech - 1];
}

byte limitPWM(float p) {
  if (p < 0) {
    p = -p;
  }
  if (p != 0)
    p = max(0.1, min(0.9, p));
  else
    p = 0.1;
  return (byte)(p * 255);
}
