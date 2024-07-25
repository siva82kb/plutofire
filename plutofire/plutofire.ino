#include <PID_v1.h>
#include <RGBLed.h>
#include <Encoder.h>
#include "variables.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include"SoftwareSerial.h"
#include <Bounce2.h>

//## PWM intialising 
//Define Variables we'll be connecting to
float _pwm = 0.0;
double Setpoint, Input, Output;
float target_pwm =25;
//Specify the links and initial tuning parameters
float Kp= .01, Ki= 0.09, Kd= .01;
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, P_ON_E,DIRECT);

// input button setup
Bounce bounce = Bounce();

//output status of device through triLED
RGBLed led(19, 18, 20, RGBLed::COMMON_CATHODE);


//Note if both reading and writing is handled by software serial module,
// the required rate of transmission is not possible
//Hence reading is through an interrupt timer and writing is async through softserial 
//Timer interrupt for reading serial data from BT
IntervalTimer readStream;
//Software serial to write data to BT. 
SoftwareSerial bt(0,1);


//initalise Encoder to recieve input through PIN_A, PIN_B
Encoder myEnc(PIN_A,PIN_B );
long oldPosition  = -999;

//variable to control sensor update
elapsedMillis sincePrint;

//float dir;

void setup() {
  
Serial.begin(115200);
bt.begin(115200);
analogReadResolution(12); // current and speed from escon
analogWriteResolution(8); // 8 bit PWM
analogWriteFrequency(PWM,2000); // 2KHs PWM freq


pinMode(PWM, OUTPUT);   // sets the pin as output
pinMode(ENABLE, OUTPUT);   // sets the pin as output
pinMode(CW, OUTPUT); 
digitalWrite(ENABLE, LOW);


// DEBOUNCE INTERVAL IN MILLISECONDS
bounce.attach(IO_SWITCH , INPUT_PULLUP ); // USE INTERNAL PULL-UP
bounce.interval(5); // interval in ms

  // LED SETUP
pinMode(LED_PIN, OUTPUT);

// PID setup
Setpoint = neutral_ang;
myPID.SetTunings(Kp, Ki, Kd); 
myPID.SetOutputLimits(-5,5); // -5A = 1.3Nm, (-5,5) is the current limits of escon controller
// Setpoint = 50;

myPID.SetSampleTime(5); //5ms sampling frequency 


readStream.begin(readHandleIncomingMessage, 10);// reading incoming data from unity via BT at 100Hz (10ms)

// // check if actuated this section is for JUNO, if JUNO isActuated = 0
// pinMode(ACTUATED,INPUT_PULLUP);
// isActuated = digitalRead(ACTUATED);
// delay(1);
// isActuated = digitalRead(ACTUATED);

isActuated = 1;// actuated = 1 (PLUTO); Nonactuated = 0; (JUNO)

// digitalWrite(ENABLE,HIGH);
// myPID.SetMode(AUTOMATIC);
ctrlType = NONE;
//Setpoint = 10;
//Kp =0.1;
//Ki=0.05;
//Kd=0.003;

}

void loop() {


//Setpoint  = Setpoint+inc;
//
//if (Setpoint>50)
// inc =-.1;
//
// if (Setpoint<-50)
// inc = +.1;

////

//uint32_t start = millis();
//Serial.println(start - previous );
//previous = start;
// SerialUSB.print("KP,KI,KD: "); SerialUSB.print(Kp, 4);
//SerialUSB.print(" Output: "); SerialUSB.print(Output);
//SerialUSB.print(" maxTorq: "); SerialUSB.print(maxTorq, 4);
//SerialUSB.print(" Angle: "); SerialUSB.print(ang.val(0, false));
//SerialUSB.print(" Setpoint: "); SerialUSB.println(Setpoint, 4);
//SerialUSB.print(", Ki: "); SerialUSB.print(Ki, 4);
//SerialUSB.print(", Kd: "); SerialUSB.print(Kd, 4);
//SerialUSB.print(", Output: "); SerialUSB.println(Output, 4);
//SerialUSB.print(", PWM: "); SerialUSB.print(target_pwm, 4);
//Serial.println(digitalRead(IO_SWITCH));



// use this blocck for quick check
//analogWrite(PWM, 25);   // sets the pin as output
//digitalWrite(ENABLE, HIGH);   // sets the pin as output
//digitalWrite(CW, HIGH); 

//SerialUSB.println((analogRead(MOTORCURR)*MCURRGAIN - maxCurrent)* 0.231);

  // Update sensor values.
updateSensorData();


////   Check if the program is currently in calibration mode.
  if (ctrlType == CALIBRATION) {
    // Handle the calibration procedure.
    calibProcess();
  }
   // Error check
  checkForErrors(); // to mitigate the possible risks like angle exceeding physiological limits, high torque etc

////     Clear control type
//   if (error == YESERR) {
////      Clear control mode.
//    // ctrlType = NONE;
//    }

//SerialUSB.println(ctrlType);

    
 //updateControl and write sensor data every 20ms
 if(sincePrint > 20){
  sincePrint =0;
 writeSensorStream(); 
 }
 updateControlLaw();

 delay(10);
}
