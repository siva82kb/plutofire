/* Functions to implement the different controllers for the 
 *  PLUTO robot.
 *  
 *  Author: Sivakumar Balasubramanian.
 *  Date: 31 Aug 2018
 */

 void updateControlLaw() {
 
 
  float _ang = ang.val(0, false);
  float _pwm = 0.0;
  
  switch(ctrlType) {
    float cur;
    float current;
    case ACTIVE:
      // Admittance control.
      if (abs(torque_est) < torqTh) {
        _pwm = 0;
      } else {
        _pwm = -acKp *10* torque_est;
      }
      break;
    case POSITION:
      // Position control.
      // Check if position control is disabled.
      if (desAng == 999) {
        _pwm = 0.0;
      } 
      else {
       
        if(abs((desAng - _ang))<1)
        _pwm =0.0;  
        else if(abs((desAng - _ang))>2)
        _pwm =  (pcKp)* ((desAng - _ang)/abs((desAng - _ang))) +0.14;
           
      }
      break;
      case TORQUE:
        if(desTorq == 0){
          _pwm = 20; 
        }
        else{
          current = desTorq/mechnicalConstant;
           _pwm =  constrain(map(abs(current),0,maxCurrent,0.1*255,0.9*255),-229,229);
//          
        }
     

      break;

     case RESIST:
      // PD resistance control
        
        desTorq =  -(kp*(_ang - neutral_ang) + kd*0.02*_ang - kd*0.02*prev_ang);
        cur = desTorq/mechnicalConstant;
        _pwm =  constrain(map(abs(cur),0,maxCurrent,0.1*255,0.9*255),-229,229);
        prev_ang = _ang;     

        break;
      
      
      
    case NONE:
      // Switch off controller.
      digitalWrite(ENABLE, LOW);
      control.add(0.0);
      return;

     
  }

  // Send PWM value to motor controller & 
  // update control.
  _pwm = sendPWMToMotor(_pwm);
  control.add(_pwm);
 }
 
 float sendPWMToMotor(float pwm) {
  if (desTorq > 0) {
    // Move counter clockwise
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CW, LOW);
    //pwm = limitPWM(pwm);
    analogWrite(PWM, max(pwm,15));
    return pwm;
  } else {
    // Move counter clockwise
    digitalWrite(ENABLE, HIGH);
    digitalWrite(CW, HIGH);
    //pwm = limitPWM(pwm);
    analogWrite(PWM,max(abs(pwm),15));
    //delayMicroseconds(10);
    return pwm;
  }
 }


 byte limitPWM(float p) {
   if (p < 0) {
    p = -p;
  }
  if(p !=0)
  p = max(0.1, min(0.9, p));
  else
    p= 0.1;
  return (byte) (p * 255);
}
