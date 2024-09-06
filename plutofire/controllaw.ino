/* Functions to implement the different controllers for the 
 *  PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 31 Aug 2018
 */

void updateControlLaw() {
    float _ang = ang.val(0);
    float _tgt = target.val(0);
    // float _currPWM = 0.0;
    float _currError = 0.0;
    float curr = 0.0;
    float currPWM =0.0;
    // float ffCurr = 0.0;
    // float ffPWM = 0.0;
    float _delPWMSign;
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
            // Check if position control is disabled.
            if (_tgt == INVALID_TARGET) {
                currPWM = 0.0;
                err.add(0.0);
                errdiff.add(0.0);
                errsum.add(0.0);
                break;
            }
            // Update error related information.
            
            // If the target is not within a error band.
            _currError = _tgt - _ang;
            if (abs((_currError)) <= POS_CTRL_DBAND) {
                _currError = 0.0;
            }
            // Error more than the threshold.
            errorSum = 0.9999 * errorSum + _currError;
            prevError = (prevError == 999) ? _currError : prevError;
            // Compute the feedback control current
            curr = pcKp * (_currError) + pcKi * errorSum + pcKd * (_currError - prevError);
            // curr = pcKp * (_currError) + pcKd * (_currError - prevError);
            currPWM = convertCurrentToPWM(curr);
            // Compute the current for the feedforward torque
            // ffCurr = desTorq / mechnicalConstant;
            // ffPWM = convertCurrentToPWM(ffCurr);
            prevError = _currError;
            break;
        case TORQUE:
            curr = _tgt / mechnicalConstant;
            currPWM = convertCurrentToPWM(curr);
            break;
        case RESIST:
            // PD resistance control
            // desTorq = -(kp * (_ang - neutral_ang) + kd * 0.02 * _ang - kd * 0.02 * prev_ang);
            // cur = desTorq / mechnicalConstant;
            // _currpwm = constrain(map(abs(cur), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
            // prev_ang = _ang;
            break;
    }
    // Limit the rate of change of PWM
    currPWM = rateLimitValue(currPWM, prevPWM, MAXDELPWM);
    // Send PWM value to motor controller & update control.
    sendPWMToMotor(currPWM);
    control.add(currPWM);
    prevPWM = currPWM;
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
