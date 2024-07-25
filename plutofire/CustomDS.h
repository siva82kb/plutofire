/*
 * CustomDS.h defines the classes implemenenting different data structures
 * used for different applications in the BioRehab Group.
 * 
 * Author: Sivakumar Balasubramanian
 * Created on: August 14 2018
 */
 
#ifndef CustomDS_h
#define CustomDS_h

#include "Arduino.h"

typedef union {
  float num;
  uint8_t bytes[4];
} floatunion_t;

// I2C Value.
typedef union {
  int num;
  uint8_t bytes[2];
} i2cval_t;


// Buffer for holding time series data.
class Buffer {
  public:
    Buffer();
    void add(float);
    byte inx(void);
    bool isFull(void);
    float val(byte, bool);
    float valf(byte, bool);
    void setconvfac(float, float);
    
  private:
    static const byte N = 16;
    static const byte Nmask = N - (byte)1;
    float _m, _c;
    float _data[N];
    float _dataf[N];
    byte _inx;
    bool _full;
    byte _nf = 5;
    float _acc;

    float conval(float);
};

// Buffer for packaging float data into a byte array.
class OutDataBuffer4Float {
  public:
    OutDataBuffer4Float();
    void newPacket();
    void add(float);
    byte sz();
    byte getByte(byte);
  private:
    floatunion_t _data[64];
    byte _sz;
};

//
//// Pressure controller Class
//class PressureController {
//  public:
//    PressureController(float);
//    void setControllerParam(float, float, float, float, float);
//    float getControl(float, float);
//    float errorIntegral(void);
//  private:
//    Buffer _err;
//    float dT;
//    float _u;
//    float _kp, _kd, _ki;
//    float _ulims[2];
//};

#endif