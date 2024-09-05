 /*  the PC or tablet.
 *  
 *  Author: Sivakumar Balasubramanian
 *  Date: 23 Aug 2018
 */

void writeSensorStream() {
    // Format:
    // 255 | 255 | No. of bytes | Status | Error Val 1 | Error Val 2 | ...
    // [Current Mechanism][isActuated] | Payload | Chksum
    byte header[] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,0x00};
    byte chksum = 0xFE;
    byte _temp;
    
    // Update Out data Buffer
    outPayload.newPacket();
    outPayload.add(ang.val(0, false));
    outPayload.add(torque_est);
    outPayload.add(control.valf(0, false));
    
    // Add desired value of the controller,
    // depending on the control mode.
    if (ctrlType == POSITION) {
        outPayload.add(desAng);
    } else if (ctrlType == TORQUE) {
        outPayload.add(desTorq);
    } else {
        outPayload.add(0.0);
    }

    // Send packet.
    header[2] = 5 + outPayload.sz() * 4 + 1;
    header[3] = getProgramStatus(SENSORSTREAM);
    header[4] = errorval[0];
    header[5] = errorval[1];
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
    
    // Send the payload with the floats first
    for (int i = 0; i < outPayload.sz() * 4; i++) {
        _temp = outPayload.getByte(i);
        bt.write(_temp);
        chksum += _temp;
    }
    // Send the PLUTO buttons state byte
    bt.write(plutoButton);
    chksum += plutoButton;
    
    // Send Checksum
    bt.write(chksum);
    bt.flush();
}

// Read and handle incoming messages.
void readHandleIncomingMessage() {
    byte _details;
    int plSz = serReader.readUpdate();
    byte _ctrltype, _ctrldet;

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
        case SET_CONTROL_TYPE:
            // Check if there is a change in control mode.
            if (ctrlType != _details) {
                // Change control mode.
                ctrlType = _details;
                // Reset desired angle and torque values.
                desTorq = 0.;
                desAng = 999;
                prevError = 999;
                errorSum = 0.0;
            }
            break;
        case SET_CONTROL_TARGET:
            // Check if the current control type is POSITION.
            if ((ctrlType == POSITION) || ((ctrlType == TORQUE))) {
                // Set target.
                setTarget(serReader.payload, 1, ctrlType);
            }
            break;
        case CALIBRATE:
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
        }
        serReader.payloadHandled();
    }
}

void sendControlParameters(byte ctype) {
  byte header[] = {0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00};
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
      outPayload.add(pcKp);
      outPayload.add(desAng);
      break;
    case TORQUE:
      outPayload.add(tcKp);
      outPayload.add(desTorq);
      break;
  }

  // Send packet.
  header[2] = outPayload.sz() * 4 + 4;
  header[4] = errorval[0];
  header[5] = errorval[1];
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
