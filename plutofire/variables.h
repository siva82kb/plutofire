
/* variables.h
 *  Header file that contains all the variable declarations for the 
 *  PLUTO CONTROL program.
 *  
 *  Sivakumar Balasubramanian.
 */
#include <Bounce2.h>
#include <Encoder.h>

#include "RGBLed.h"
#include "CustomDS.h"
#include "SerialReader.h"
#include "SoftwareSerial.h"

// Control type
#define NONE                0x00
#define POSITION            0x01
#define RESIST              0x02
#define TORQUE              0x03

// Mechanisms
#define WFE                 0x00
#define WUD                 0x01
#define WPS                 0x02
#define HOC                 0x03
#define NOMECH              0x04

// Out data type
#define SENSORSTREAM        0x00
#define CONTROLPARAM        0x01
#define DIAGNOSTICS         0x02

// In data type
#define GET_VERSION         0x00
#define CALIBRATE           0x01
#define START_STREAM        0x02
#define STOP_STREAM         0x03
#define SET_CONTROL_TYPE    0x04
#define SET_CONTROL_TARGET  0x05
#define SET_DIAGNOSTICS     0x06

// Control/Target Parameter Detail
#define POSITIONTGT         0x08
#define FEEDFORWARDTGT      0x20   

// Control Law Related Definitions
#define INVALID_TARGET      999.0
#define INTEGRATOR_LIMIT    4.0
#define MINPWM              26
#define MAXPWM              229
#define MAXDELPWM           5

// Error types
#define ANGSENSERR          0x0001
#define VELSENSERR          0x0002
#define TORQSENSERR         0x0004
#define MCURRSENSERR        0x0008

// Operation status
#define NOERR               0x00
#define YESERR              0x01

// Kinematic calib status
#define NOCALIB             0x00
#define YESCALIB            0x01

// Control related variables
#define POS_CTRL_DBAND      2

#define IO_SWITCH           17
#define LED_PIN             LED_BUILTIN

#define ACTUATED            21
byte isActuated;

// torque sensor pind
#define torqueData          8
#define torqueClock         7

// Encoder reading pins
#define PIN_A               2//40// 29// 36//
#define PIN_B               3//42//  27//38//
#define TORQSENSOR          0
#define MOTORCURR           14//grey
#define MOTORSPEED          15//pink; blue=ground

// Motor control pins
#define CW                  6//18//38//49// 47//42 //green
#define PWM                 4//19
#define ENABLE              5//20//36//40//51 brown

#define RESET               39  

// Sensor gains
#define ANGVELGAIN          1//50000. * 360. / (60 * 4096.)
#define MCURRGAIN           2 * maxCurrent / 4096.0
#define MCURROFFSET         -maxCurrent

#define mechnicalConstant   0.231 //for 48v 0.231; // for 24V 0.077;
#define maxCurrent          8

#define VERSION             "24.09"
#define DEVID               "PLUTO240725"

// ofset angle
int encOffsetCount = 0;
int enPPRActuated = 4096;   //6400 for new motor 4096 for old motor
int enPPRnonActuated = 4096 ;

// Sensor data buffers
Buffer ang;
Buffer torque;
Buffer control;
Buffer target;
// Additional buffers
Buffer err;
Buffer errdiff;
Buffer errsum;

float loadCell1 = 12.3;
float loadCell2 = 21.3;
float loadCell3 = 32.1;

// Variable to hold the current PLUTO button state.
volatile byte plutoButton = 1;
bool ledState = 1;

float prev_time = 0, prev_torque = 0;
float torq_df = 0;

// Active assist parameters
float arom[] = {999,999};
float prom[] = {999,999};
float asssitProfile[10] = {0};

// Mehanism
byte currMech = NOMECH;

// Sensor parameters
struct SensorParam {
  float m;
  float c;
};
SensorParam torqParam, angvelParam, mcurrParam;

// Program status
byte streamType = SENSORSTREAM;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
byte error = NOERR;
byte errorval[] = {0x00, 0x00};

// Serial Reader object
SerialReader serReader;

// Out data buffer
OutDataBuffer4Float outPayload;

// Control variables
float dT = 0.01;

// Admittance Control
float acKp = 0.1;
float torqTh = 0.05;
float prev_ang;

// Poition Control
float pcKp = 0.1;
float pcKd = 0.01;
float pcKi = 0.001;
// float desAng = 0.0;

// Torque Control
float tcKp = 0.0;
float tcKd = 0.2;
// float desTorq = 0.0;

// Resistance control
float kp = -1;
float kd = -1;
float km = -1;
float tor;
float neutral_ang;

// float offsetTorque;
// float currPWM;
// float prevPWM;
// float prevError;
// float errorSum;
// unsigned long prevLoopTime;
// int count;

// // Variables for calibration
// byte maxCalibCount = 16;
// byte calibCount = 0;

/* Tempoary section : To be formated later */
Bounce bounce = Bounce();

// Timer interrupt for reading serial data
IntervalTimer readStream;

SoftwareSerial bt(0, 1);
RGBLed led(19, 18, 20, RGBLed::COMMON_CATHODE);
Encoder plutoEncoder(PIN_A, PIN_B);