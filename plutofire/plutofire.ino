/*
 * PLUTOFIRE
 * Firmware code implementing all device features.
 * 
 * Author: Aravind Nehrujee, Ann David, Sivakumar Balasubramanian
 * Email: {anndavid293, siva82kb} @ @gmail.com 
 */

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
    pinMode(PWM, OUTPUT);
    pinMode(ENABLE, OUTPUT);
    pinMode(CW, OUTPUT);
    digitalWrite(ENABLE, LOW);

    // Device button settiongs
    bounce.attach(IO_SWITCH, INPUT_PULLUP);
    bounce.interval(5);

    // LED setup
    pinMode(LED_PIN, OUTPUT);

    /* Read incoming packets on an interval timer.
     * Reads incoming data every 1000usec. */ 
    readStream.begin(readHandleIncomingMessage, 1000);
    // digitalWrite(LED_PIN, 1);
    
    // Set actuated/unactuated
    isActuated = 1;  // actuated = 1; Nonactuated = 0;

    // No current mechanism.
    currMech = NOMECH;

    // Set targets to invalid value 999.
    target.add(INVALID_TARGET);
}


void loop() {
    //dir = !dir;

    // use this blocck for quick check
    //analogWrite(PWM, 229);   // sets the pin as output
    //digitalWrite(ENABLE, HIGH);   // sets the pin as output
    //digitalWrite(CW, HIGH);

    // for checking angle
    //SerialUSB.println(encAngle());

    // Read and update sensor values.
    updateSensorData();

    //   Check if the program is currently in calibration mode.
    // if (ctrlType == CALIBRATION) {
    //     // Handle the calibration procedure.
    //     calibProcess();
    // }
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
        // Serial.println(VERSION);
    }

    delay(10);
}
