 /* SerailReader.h defines the class for handling reading, parsing
 * and handling of serial data.
 * 
 * Author: Sivakumar Balasubramanian
 * Created on: August 01 2017
 */
 
#ifndef SerialReader_h
#define SerialReader_h

#include "Arduino.h"

enum ReaderState {
  WAITFORPACKET,
  HEADER1,
  HEADER2,
  PAYLOAD,
  CHKSUM,
  WAITFORHANDLING
};

class SerialReader {
  public:
    static const byte maxPayloadSize = 255;
    bool temp = false;
    byte payload[maxPayloadSize];
    SerialReader();
    int readUpdate();
    void payloadHandled();
  private:
    // Serial communciation related dataa
    byte _currByte;
    unsigned int _currPlSz;
    unsigned int _plCntr;
    byte _chksum;
    ReaderState _state;
};

#endif
