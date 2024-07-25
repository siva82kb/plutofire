/*
 * SerailReader.cpp implements the class for handling reading, parsing
 * and handling of serial data.
 * 
 * Author: Sivakumar Balasubramanian
 * Created on: August 01 2017
 */
 
#include "Arduino.h"
#include "SerialReader.h"
#include"SoftwareSerial.h"
SoftwareSerial bt1(0,1);
SerialReader::SerialReader() {
  _state = WAITFORPACKET;
}

int SerialReader::readUpdate() {
  while (bt1.available() > 0) {
    // Read current byte
    _currByte = bt1.read();
    switch (_state) {
      case WAITFORPACKET:
        if (_currByte == 0xFF) {
          // First header found.
          _state = HEADER1;
        }
        break;
      case HEADER1:
        if (_currByte == 0xFF) {
          // Second header found.
          _state = HEADER2;
        } else {
          _state = WAITFORPACKET;
        }
        break;
      case HEADER2:
        // Get the size of the payload.
        // It cannot be zero or greater than maxPayloadSize.
        if (_currByte == 0x00 || _currByte > maxPayloadSize) {
          _state = WAITFORPACKET;
        } else {
          _currPlSz = _currByte;
          _plCntr = 0;
          _state = PAYLOAD;
          _chksum = 0xFF + 0xFF + _currByte;
        }        
        break;
      case PAYLOAD:
    
        payload[_plCntr++] = _currByte;
        _chksum += _currByte;
        if (_plCntr == _currPlSz) {
          _state = CHKSUM;
        }
        break;
        
      case CHKSUM:
        if (_chksum == _currByte) {
          _state = WAITFORHANDLING;
        } else {
          _state = WAITFORPACKET;
        }
        break;
      

    
      case WAITFORHANDLING:
        break;

       
    }
    // Return payload size only if the current state is WAITFORHANDLING
    if (_state == WAITFORHANDLING) {
      return _currPlSz;      
    } else {
      return -1;
    }
  }
  
}

void SerialReader::payloadHandled() {
  _currPlSz = -1;
  _state = WAITFORPACKET;
}
