
static float last_pwm = 0; // Stores the last PWM value for smoothing transitions
const float smoothingFactor = 0.0261; // Adjust this to control the rate of smoothing

// Function Prototypes
float sendPWMToMotor(float pwm);
float limitPWM(float p);

void updateControlLaw() {
    float _ang = ang.val(0, false);

    switch (ctrlType) {
        case ACTIVE:
            target_pwm = abs(transformed_torque) < torqTh ? 0 : -acKp * 10 * transformed_torque;
            break;
        case POSITION:
            if (desAng != 999) {
                float angDiff = desAng - _ang;
                target_pwm = abs(angDiff) < 1 ? 0.0 : (abs(angDiff) > 2 ? pcKp * (angDiff / abs(angDiff)) + 0.14 : 0.0);
            }
            break;
        case TORQUE:
            if (desTorq == 0) {
                target_pwm = 20;
            } else {
                float current = desTorq / mechnicalConstant;
                target_pwm = constrain(map(abs(current), 0, maxCurrent, 0.1 * 255, 0.9 * 255), -229, 229);
                target_pwm *= (desTorq < 0) ? -1 : 1;
            }
            break;
        case RESIST: // only this is being used for now.
            Input = ang.val(0, false);

                if(Setpoint != 999){
                   if(abs(Setpoint-Input) <3){ // to prevent oscillations when you are very close to the target. this method id called adaptive control
                      myPID.SetTunings(.0008, .003, Kd); 
                   }
                   else{
                      myPID.SetTunings(Kp, Ki, Kd); 
                   }
                myPID.Compute();
                target_pwm = constrain((map(abs(Output), 0, 5, 0.1 * 255, 0.9 * 255)), 0.10 * 255, 0.9*255);
                target_pwm *= (Output < 0) ? -1 : 1;
                }
                else{
                  target_pwm = 20;
                } 
            
            break;
        case NONE:
            target_pwm = 20;

            break;
    }

    // Smooth and limit the PWM value before sending it to the motor
    last_pwm += smoothingFactor * (target_pwm - last_pwm); // to remove the sudden jerks when the delta is high

    sendPWMToMotor(limitPWM(last_pwm));

    control.add(target_pwm);
}

float sendPWMToMotor(float target_pwm) {


    digitalWrite(ENABLE, target_pwm != 0 ? HIGH : LOW);
    digitalWrite(CW, target_pwm < 0 ? HIGH : LOW);
    analogWrite(PWM, abs(target_pwm)); // Use abs() to ensure PWM signal is always positive
   // return target_pwm; 
}

float limitPWM(float p) {
  
    // Ensure PWM stays within the specified range
    float limited_pwm = constrain(abs(p), 0.10 * 255, 0.9 * 255);
    return p < 0 ? -limited_pwm : limited_pwm; // Retain direction of the PWM
}
