
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
#define POSITIONAAN         0x04

// Mechanisms
#define NOMECH              0x00
#define WFE                 0x01
#define WUD                 0x02
#define FPS                 0x03
#define HOC                 0x04
#define FME1                0x05
#define FME2                0x06

// Out data type
#define SENSORSTREAM        0x00
#define CONTROLPARAM        0x01
#define DIAGNOSTICS         0x02
#define VERSION             0x03

// In data type
#define GET_VERSION         0x00
#define CALIBRATE_START     0x01
#define START_STREAM        0x02
#define STOP_STREAM         0x03
#define SET_CONTROL_TYPE    0x04
#define SET_CONTROL_TARGET  0x05
#define SET_DIAGNOSTICS     0x06
#define SET_CONTROL_BOUND   0x07
#define RESET_PACKETNO      0x08
#define SET_CONTROL_DIR     0x09
#define SET_AAN_TARGET      0x0A
#define RESET_AAN_TARGET    0x0B
#define SET_CONTROL_GAIN    0x0C
#define CALIBRATE_END       0x0D
#define HEARTBEAT           0x80

// Control Law Related Definitions
#define INVALID_TARGET      999.0
#define INTEGRATOR_LIMIT    4.0
#define PWMRESOLN           12      // This has been changed from 8. Suggestions from Aravind.
#define MINPWM              410     // 10% of 4095
#define MAXPWM              3686    // 90% of 4095
#define MAXDELPWM           40      // Changed from 5

// Error types 
#define ANGSENSERR          0x0001
#define MCURRSENSERR        0x0002
#define NOHEARTBEAT         0x0004

// Kinematic calib status
#define NOCALIB             0x00
#define YESCALIB            0x01

// Control related variables
#define POS_CTRL_DBAND      2
#define MAX_CTRL_GAIN       10.0

#define IO_SWITCH           17
#define LED_PIN             LED_BUILTIN

#define ACTUATED            21

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

//LED PINS
#define RED_PIN             19 // red color
#define BLUE_PIN            18 // blue
#define GREEN_PIN           20 // green


// Motor constants
#define MECHANICAL_CONST    0.231 //for 48v 0.231; // for 24V 0.077;
#define MAX_CURRENT         8

// Heart beat related variable
#define MAX_HBEAT_INTERVAL  1.0 // Seconds

// Some useful function
#define sgn(x) ((x) < 0 ? -1 : ((x) > 0 ? 1 : 0))

// Nonlinear PID controller functions.
#define linclip(x) ((x) < 0 ? 0 : ((x) > 1 ? 1 : x))

// Mechanism offset and range.
const float mechOffsetValue[] = { 
  0,    // Dummy. No mechanism 
  68,   // Wrist Flexion/Extension     
  68,   // Wrist Ulnar/Radial Deviation
  90,   // Forearm Prono/Sunpination
  0,    // Hand Opening/Closing
  90,    // Functional mechanism 1
  90,    // Functional mechanism 2
};
const float mechRangeValue[] = { 
  0,     // Dummy. No mechanism 
  136,   // Wrist Flexion/Extension     
  136,   // Wrist Ulnar/Radial Deviation
  180,   // Forearm Prono/Sunpination
  0,    // Hand Opening/Closing
  180,    // Functional mechanism 1
  180,    // Functional mechanism 2
};

// Actuated device?
byte isActuated;

// Version and device ID.
const char* fwVersion = "HB-SW-1.0";
const char* deviceId  = "PLUTO250130";
const char* compileDate = __DATE__ " " __TIME__;

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// ofset angle
int encOffsetCount = 0;
int enPPRActuated = 6400;   //6400 for new motor 4096 for old motor
int enPPRnonActuated = 4096 ;

bool isCalibrating = false;
float angleCorrection = 0;        // Delta angle to correct for any deviation in angle range.

// Sensor data buffers
Buffer ang;
Buffer torque;
Buffer control;
Buffer desired;
// Target is set once and this is used to derive the desired value.
// All controllers that require a desired position will need to use 
// the data from the desired buffer.
float target;

// Additional buffers
Buffer err;
Buffer errdiff;
Buffer errsum;

// Variable to hold the current PLUTO button state.
volatile byte plutoButton = 1;
bool ledState = 1;

//led
static int* lastColor = nullptr;
int* newColor;

// Packet Counter.
uint16union_t packetNumber;

// run time
unsigned long startTime;
ulongunion_t runTime;

// Mechanism
byte currMech = NOMECH;

// Program status
byte streamType = SENSORSTREAM;
bool stream = true;
byte ctrlType = NONE;
byte calib = NOCALIB;
uint16union_t deviceError;
// byte errorval[] = {0x00, 0x00};

// Serial Reader object
SerialReader serReader;

// Out data buffer
OutDataBuffer4Float outPayload;

// Poition Control
float pcKp = 0.1;
float pcKd = 0.01;
float pcKi = 0.001;
// The parameter to bound the PWM/Current value to within +/- ctrlBound.
// Its a value between 0 and 1: 0 means < 10% PWM, and 1 means 90% PWM.
float ctrlBound = 1.0;
// Direction variables for assymetric control of assistance.
int8_t ctrlDir = 0;
// Position controller scale.
uint8_t ctrlGain = 0;

// Defining the mechanism dependent controller gains
const float mechKp[] = { 0.1, 0.1, 0.1, 0.1 };
const float mechKd[] = { 0.01, 0.01, 0.01, 0.01 };  
const float mechKi[] = { 0.001, 0.001, 0.001, 0.001 };

// Resistance control
float kp = -1;
float kd = -1;
float km = -1;
float tor;
float neutral_ang;

// Desired target generator filter for PositionAAN.
// Filter gain
const float K_filt = 1.0 / 7139.0;
const float b_filt[] = { 1.0, 2.0, 1.0 }; 
const float a_filt[] = { 7139.0, -13776.0, 6641.0 };

// HOMER Assist-As-Needed trajectory parameters
float strtPos;
float strtTime;
float initTime;
float reachDur;

/* Tempoary section : To be formated later */
Bounce bounce = Bounce();

// Timer interrupt for reading serial data
IntervalTimer readStream;

SoftwareSerial bt(0, 1);
RGBLed led(19, 20, 18, RGBLed::COMMON_CATHODE);
Encoder plutoEncoder(PIN_A, PIN_B);