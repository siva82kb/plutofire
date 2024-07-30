/* Functions to implement the different controllers for the 
 *  PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 31 Aug 2018
 */

void updateControlLaw() {
    float _ang = ang.val(0, false);
    float _currpwm = 0.0;
    float _currError = 0.0;
    float cur;
    float current;
    float _delpwmsign;
    switch (ctrlType) {
        // case ACTIVE:
        // // Admittance control.
        // if (abs(torque_est) < torqTh) {
        //     _currpwm = 0;
        // } else {
        //     _currpwm = -acKp * 10 * torque_est;
        // }
        // break;
        case POSITION:
            // Position control.
            // Check if position control is disabled.
            if (desAng == 999) {
                currPWM = 0.0;
                break;
            }
            // A valid position target has been set.
            if (abs((desAng - _ang)) <= 5) {
                break;
            }
            // Error more than the threshold.
            _currError = desAng - _ang;
            errorSum = 0.999 * errorSum + _currError;
            prevError = (prevError == 999) ? _currError : prevError;
            // Compute the control current
            current = pcKp * (_currError) + pcKi * errorSum + pcKd * (_currError - prevError);
            _currpwm = convertCurrentToPWM(current);
            prevError = _currError;
            break;
        case TORQUE:
            current = desTorq / mechnicalConstant;
            _currpwm = convertCurrentToPWM(current);
            break;
        case RESIST:
            // PD resistance control
            // desTorq = -(kp * (_ang - neutral_ang) + kd * 0.02 * _ang - kd * 0.02 * prev_ang);
            // cur = desTorq / mechnicalConstant;
            // _currpwm = constrain(map(abs(cur), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
            // prev_ang = _ang;
            break;
        case NONE:
            // Switch off controller.
            digitalWrite(ENABLE, LOW);
            control.add(0.0);
            return;
    }
    // Limit the rate of change of PWM
    currPWM = rateLimitValue(_currpwm, prevPWM, MAXDELPWM);
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
