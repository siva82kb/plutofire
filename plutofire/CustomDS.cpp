/*
 * CustomDS.cpp implements the classes the classes for different 
 * data structures used for different applications in the BioRehab Group.
 * 
 * Author: Sivakumar Balasubramanian
 * Created on: August 15 2018
 * Updated on: September 06 2024
 */

#include "Arduino.h"
#include "CustomDS.h"

// Buffer class definitions
Buffer::Buffer() {
  _inx = 0xFF;
}

void Buffer::add(float val) {
    // Check if the buffer is empty. If so, fill the entire,
    // buffer with the first value.
    if (_inx == 0xFF) {
        for (int i = 0; i < N; i++) {
            _data[i] = val;
        }
        _inx = 0;
    } else {
        _inx += 1;
        _inx = _inx & Nmask;    
        _data[_inx] = val;
    }
}

bool Buffer::isEmpty(void) {
  return _inx == 0xFF;
}

float Buffer::val(byte pos) {
  // Cap position to the max. possible value.
  pos = (pos >= N) ? N - 1 : pos;
  // Negative values get capped to 0. They will always
  // return the current value.
  pos = (pos < 0) ? 0 : pos;
  return _data[(_inx - pos) & Nmask];
}

// OutDataBuffer class definitions
OutDataBuffer4Float::OutDataBuffer4Float() {
  _sz = 0;
}

void OutDataBuffer4Float::newPacket() {
  _sz = 0;
}

void OutDataBuffer4Float::add(float num) {
  _data[_sz++].num = num;
}

byte OutDataBuffer4Float::sz() {
  return _sz;
}

byte OutDataBuffer4Float::getByte(byte inx) {
  byte q = inx / 4;
  byte r = inx % 4;
  return _data[q].bytes[r];
}
