/*  the PC or tablet.
 *  
 *  Author: Sivakumar Balasubramanian
 *  Date: 23 Aug 2018
 */


// Read and handle incoming messages.
// Many commands will be ignored if there is a device error.
void readHandleIncomingMessage() {
  byte _details;
  int plSz = serReader.readUpdate();

  // Read handle incoming data
  if (plSz > 0) {
    _details = serReader.payload[1];
    // Handle new message.
    // Check the command type.
    switch (serReader.payload[0]) {
      case START_STREAM:
        stream = true;
        streamType = SENSORSTREAM;
        break;
      case STOP_STREAM:
        stream = false;
        break;
      case SET_DIAGNOSTICS:
        stream = true;
        streamType = DIAGNOSTICS;
        break;
      case SET_CONTROL_TYPE:
        // This can be set only if there is not error.
        if (deviceError.num != 0) break;
        // No Error
        setControlType(_details);
        break;
      case SET_CONTROL_TARGET:
        // This can be set only if there is not error.
        if (deviceError.num != 0) break;
        // No Error
        // Check if the current control type is POSITION.
        if ((ctrlType == POSITION)
            || (ctrlType == POSITIONAAN)
            || (ctrlType == TORQUE)) {
          // Set target.
          setTarget(serReader.payload, 1, ctrlType);
        }
        break;
      case SET_CONTROL_BOUND:
        // This can be set only if there is not error.
        if (deviceError.num != 0) break;
        // No Error
        // Check if the current control type is POSITION.
        ctrlBound = 0.0;
        if ((ctrlType == POSITION)
            || (ctrlType == POSITIONAAN)) {
          ctrlBound = _details / 255.0;
        }
        break;
      case SET_CONTROL_DIR:
        // This can be set only if there is not error.
        if (deviceError.num != 0) break;
        // No Error
        // Check if the current control type is POSITIONAAN.
        ctrlDir = 0;
        if (ctrlType == POSITIONAAN) {
          ctrlDir = _details;
        }
        break;
      case CALIBRATE:
        // This can be set only if there is not error.
        if (deviceError.num != 0) break;
        // No Error
        // Reset calibration
        // Check the calibration value
        calib = NOCALIB;
        currMech = _details;
        if (currMech != NOMECH) {
          // Set the encoder offset value
          encOffsetCount = plutoEncoder.read();
          calib = YESCALIB;
        }
        break;
      case GET_VERSION:
        stream = false;
        // Send the current firmware version.
        sendVersionDetails();
        break;
      case HEARTBEAT:
        lastRxdHeartbeat = millis();
        break;
    }
    serReader.payloadHandled();
  }
}

void writeSensorStream() {
  // Format:
  // 255 | 255 | No. of bytes | Status | Error Val 1 | Error Val 2 | ...
  // [Current Mechanism][isActuated] | Payload | Chksum
  byte header[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 };
  byte chksum = 0xFE;
  byte _temp;

  // Update Out data Buffer
  outPayload.newPacket();
  outPayload.add(ang.val(0));
  outPayload.add(0);
  outPayload.add(control.val(0));
  outPayload.add(target.val(0));

  // Add additional data if in DIAGNOSTICS mode
  if (streamType == DIAGNOSTICS) {
    outPayload.add(err.val(0));
    outPayload.add(errdiff.val(0));
    outPayload.add(errsum.val(0));
  }

  // Send packet.
  header[2] = (4                      // Four headers
               + 2                    // Packet number int16
               + 4                    // Run time
               + outPayload.sz() * 4  // Float sensor data
               + 1                    // Control bound data
               + 1                    // Control direction data
               + 1                    // PLUTO button data
               + 1                    // Checksum
  );
  header[3] = getProgramStatus(streamType);
  header[4] = deviceError.bytes[0];
  header[5] = deviceError.bytes[1];
  header[6] = getMechActType();
  chksum += header[2] + header[3] + header[4] + header[5] + header[6];

  // Send the header.
  bt.write(header[0]);
  bt.write(header[1]);
  bt.write(header[2]);
  bt.write(header[3]);
  bt.write(header[4]);
  bt.write(header[5]);
  bt.write(header[6]);

  // Send packet number
  for (int i = 0; i < 2; i++) {
    bt.write(packetNumber.bytes[i]);
    chksum += packetNumber.bytes[i];
  }

  // Send current run time
  for (int i = 0; i < 4; i++) {
    bt.write(runTime.bytes[i]);
    chksum += runTime.bytes[i];
  }

  // Send the payload with the floats first
  for (int i = 0; i < outPayload.sz() * 4; i++) {
    _temp = outPayload.getByte(i);
    bt.write(_temp);
    chksum += _temp;
  }

  // Write the current control Bound.
  byte _ctrlb = (byte)(255 * ctrlBound);
  bt.write(_ctrlb);
  chksum += _ctrlb;

  // Send the control direction byte
  bt.write(ctrlDir);
  chksum += ctrlDir;

  // Send the PLUTO buttons state byte
  bt.write(plutoButton);
  chksum += plutoButton;

  // Send Checksum
  bt.write(chksum);
  bt.flush();
}

void sendVersionDetails() {
  // Format:
  // 255 | 255 | No. of bytes | Status | Error Val 1 | Error Val 2 | ...
  // [Current Mechanism][isActuated] | Payload | Chksum
  byte header[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00 };
  byte chksum = 0xFE;

  // Send packet.
  header[2] = (4                      // Four headers
               + strlen(fwVersion)    // Version string length
               + 1                    // Comma separator
               + strlen(deviceId)     // Deviice ID string length
               + 1                    // Comma separator
               + strlen(compileDate)  // Compilation date.
               + 1                    // Checksum
  );
  header[3] = getProgramStatus(VERSION);
  header[4] = deviceError.bytes[0];
  header[5] = deviceError.bytes[1];
  header[6] = getMechActType();
  chksum += header[2] + header[3] + header[4] + header[5] + header[6];

  // Send the header.
  bt.write(header[0]);
  bt.write(header[1]);
  bt.write(header[2]);
  bt.write(header[3]);
  bt.write(header[4]);
  bt.write(header[5]);
  bt.write(header[6]);

  // Send Device ID
  for (unsigned int i = 0; i < strlen(deviceId); i++) {
    bt.write(deviceId[i]);
    chksum += deviceId[i];
  }
  bt.write(',');
  chksum += ',';
  // Send firmware version
  for (unsigned int i = 0; i < strlen(fwVersion); i++) {
    bt.write(fwVersion[i]);
    chksum += fwVersion[i];
  }
  bt.write(',');
  chksum += ',';
  // Send firmware compilation date
  for (unsigned int i = 0; i < strlen(compileDate); i++) {
    bt.write(compileDate[i]);
    chksum += compileDate[i];
  }
  // Send Checksum
  bt.write(chksum);
  bt.flush();
}

void sendControlParameters(byte ctype) {
  byte header[] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00 };
  byte chksum = 0xFE;
  byte _temp;

  // Get status byte.
  header[3] = getProgramStatus(CONTROLPARAM);
  header[3] = header[3] & (0b01110001);
  header[3] = header[3] | (ctype << 1);

  // Update Out data Buffer
  outPayload.newPacket();
  switch (ctype) {
    // case ACTIVE:
    //   outPayload.add(acKp);
    //   break;
    case POSITION:
      // outPayload.add(pcKp);
      outPayload.add(target.val(0));
      break;
    case TORQUE:
      // outPayload.add(tcKp);
      outPayload.add(target.val(0));
      break;
  }

  // Send packet.
  header[2] = outPayload.sz() * 4 + 4;
  header[4] = deviceError.bytes[0];
  header[5] = deviceError.bytes[1];
  chksum += header[2] + header[3] + header[4] + header[4];

  // Send header
  bt.write(header[0]);
  bt.write(header[1]);
  bt.write(header[2]);
  bt.write(header[3]);
  bt.write(header[4]);
  bt.write(header[5]);
  // Send payload
  for (int i = 0; i < outPayload.sz() * 4; i++) {
    _temp = outPayload.getByte(i);
    bt.write(_temp);
    chksum += _temp;
  }
  bt.write(chksum);
}
