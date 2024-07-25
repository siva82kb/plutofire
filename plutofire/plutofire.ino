#include "variables.h"
#include "CustomDS.h"
#include "SerialReader.h"


void setup() {
    // Basic device setup
    Serial.begin(115200);
    bt.begin(115200);
    analogReadResolution(12);
    analogWriteResolution(8);
    analogWriteFrequency(4, 2000);

    // Motor control pins
    pinMode(PWM, OUTPUT);     // sets the pin as output
    pinMode(ENABLE, OUTPUT);  // sets the pin as output
    pinMode(CW, OUTPUT);
    digitalWrite(ENABLE, LOW);

    // Device button settiongs
    bounce.attach(IO_SWITCH, INPUT_PULLUP);  // Use internal pull-up
    bounce.interval(5);                      // interval in ms

    // LED setup
    pinMode(LED_PIN, OUTPUT);

    /* Read incoming packets on an interval timer.
     * Reads incoming data every 1000usec. */ 
    readStream.begin(readHandleIncomingMessage, 1000);
    
    // Set actuated/unactuated
    isActuated = 1;  // actuated = 1; Nonactuated = 0;
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
  if (sincePrint > 10) {
    sincePrint = 0;

    writeSensorStream();
    updateControlLaw();
  }


  delay(10);
}
