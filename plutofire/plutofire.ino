/*
 * PLUTOFIRE
 * Firmware code implementing all device features.
 * 
 * Author: Sivakumar Balasubramanian
 * Email: siva82kb@gmail.com 
 */

#include "variables.h"
#include "CustomDS.h"
#include "SerialReader.h"

void setup() {
  // Basic device setup
  Serial.begin(115200);
  bt.begin(115200);
  analogReadResolution(12);
  analogWriteResolution(PWMRESOLN);
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

  // Set actuated/unactuated
  isActuated = 1;  // actuated = 1; Nonactuated = 0;

  // No current mechanism.
  currMech = NOMECH;

  // Set targets to invalid value 999.
  target.add(INVALID_TARGET);

  // Initialize variable.
  streamType = SENSORSTREAM;
  stream = true;
  ctrlType = NONE;
  calib = NOCALIB;
  deviceError.num = 0x0000;

  // Reset packet number and run time.
  packetNumber.num = 0;
  startTime = millis();
  runTime.num = 0;

  // Last received heart beat time
  lastRxdHeartbeat = millis();
}


void loop() {
  // Check heartbeat
  checkHeartbeat();

  // Read and update sensor values.
  updateSensorData();

  // Send sensordata
  if (stream) {
    writeSensorStream();
  }

  // Update control
  if (deviceError.num == 0) {
    updateControlLaw();
  }

  // Relax. You only need to work at around 200Hz
  delay(2);
  packetNumber.num += 1;
  runTime.num = millis() - startTime;
}