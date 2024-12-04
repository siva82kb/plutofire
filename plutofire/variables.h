
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
#define WPS                 0x03
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
#define CALIBRATE           0x01
#define START_STREAM        0x02
#define STOP_STREAM         0x03
#define SET_CONTROL_TYPE    0x04
#define SET_CONTROL_TARGET  0x05
#define SET_DIAGNOSTICS     0x06
#define SET_CONTROL_BOUND   0x07
#define RESET_PACKETNO      0x08
#define SET_CONTROL_DIR     0x09
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

// Motor constants
#define MECHANICAL_CONST    0.231 //for 48v 0.231; // for 24V 0.077;
#define MAX_CURRENT         8

// Heart beat related variable
#define MAX_HBEAT_INTERVAL  1.0 // Seconds

// Actuated device?
byte isActuated;

// Version and device ID.
const char* fwVersion = "24.12";
const char* deviceId  = "PLUTO240725";
const char* compileDate = __DATE__ " " __TIME__;

// Last received heartbeat time.
float lastRxdHeartbeat = 0.0f;

// ofset angle
int encOffsetCount = 0;
int enPPRActuated = 6400;   //6400 for new motor 4096 for old motor
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

// Variable to hold the current PLUTO button state.
volatile byte plutoButton = 1;
bool ledState = 1;

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

/* Tempoary section : To be formated later */
Bounce bounce = Bounce();

// Timer interrupt for reading serial data
IntervalTimer readStream;

SoftwareSerial bt(0, 1);
RGBLed led(19, 18, 20, RGBLed::COMMON_CATHODE);
Encoder plutoEncoder(PIN_A, PIN_B);