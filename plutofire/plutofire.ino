#include <Encoder.h>
#include "RGBLed.h"
#include "variables.h"
#include "CustomDS.h"
#include "SerialReader.h"

// input button setup
#include <Bounce2.h>
Bounce bounce = Bounce();

//Timer interrupt for reading serial data
IntervalTimer readStream;


#include"SoftwareSerial.h"

SoftwareSerial bt(0,1);
RGBLed led(19, 18, 20, RGBLed::COMMON_CATHODE);
Encoder myEnc(PIN_A,PIN_B );

long oldPosition  = -999;
elapsedMillis sincePrint;
bool dir;

void setup() {
  
Serial.begin(115200);
bt.begin(115200);
analogReadResolution(12);
analogWriteResolution(8);
analogWriteFrequency(4,2000);


pinMode(PWM, OUTPUT);   // sets the pin as output
pinMode(ENABLE, OUTPUT);   // sets the pin as output
pinMode(CW, OUTPUT); 
digitalWrite(ENABLE, LOW);


// DEBOUNCE INTERVAL IN MILLISECONDS
bounce.attach(IO_SWITCH , INPUT_PULLUP ); // USE INTERNAL PULL-UP
bounce.interval(5); // interval in ms

  // LED SETUP
pinMode(LED_PIN, OUTPUT);


//initSensorParam(); 
readStream.begin(readHandleIncomingMessage, 1000);

// check if actuated
pinMode(ACTUATED,INPUT_PULLUP);
isActuated = digitalRead(ACTUATED);
delay(1);
isActuated = digitalRead(ACTUATED);

isActuated = 1;// actuated = 1; Nonactuated = 0;


}

void loop() {
  //dir = !dir;

// use this blocck for quick check
//analogWrite(PWM, 229);   // sets the pin as output
//digitalWrite(ENABLE, HIGH);   // sets the pin as output
//digitalWrite(CW, HIGH); 

// for checking angle
//SerialUSB.println(encAngle());

  // Update sensor values.
updateSensorData();



////   Check if the program is currently in calibration mode.
  if (ctrlType == CALIBRATION) {
    // Handle the calibration procedure.
    calibProcess();
  }
//    // Error check
//   checkForErrors();
//
////     Clear control type
//   if (error == YESERR) {
////      Clear control mode.
//    // ctrlType = NONE;
//    }

 
    
// updateControl and write sensor data every 20ms
 if(sincePrint > 10){
  sincePrint =0;

   writeSensorStream();
  updateControlLaw();
    
   
 }
 

 delay(10);
}
