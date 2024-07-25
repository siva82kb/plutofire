

/* variables.h
 *  Header file that contains all the variable declarations for the 
 *  PLUTO CONTROL program.
 *  
 *  Sivakumar Balasubramanian.
 */

#include "CustomDS.h"
#include "SerialReader.h"

// Control type
#define NONE                0x00
#define POSITION            0x01
#define TORQUE              0x06
#define ACTIVEASSIST        0x03
#define RESIST              0x04
#define CALIBRATION         0X05
#define ACTIVE              0x02
#define SPEED               0x07

//Mechanisms
#define NONE  0x00
#define WFE   0x02
#define WUD   0x04
#define WPS   0x06
#define HOC   0x08


// Out data type
#define SENSORSTREAM        0x05
#define SENSORPARAM         0x01
#define DEVICEERROR         0x02
#define CONTROLPARAM        0x03
#define DIAGNOSTICS         0x04


// In data type
#define SET_ERROR           0x00
#define START_STREAM        0x01
#define STOP_STREAM         0x02
#define SET_SENSOR_PARAM    0x03
#define GET_SENSOR_PARAM    0x04
#define SET_CONTROL_PARAM   0x05
#define GET_CONTROL_PARAM   0x06
#define CALIBRATE           0x07


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

#define IO_SWITCH       17
#define LED_PIN LED_BUILTIN

#define ACTUATED     21
byte isActuated;

// torque sensor pind
#define torqueData          8
#define torqueClock         7

// Encoder reading pins
#define PIN_A             2//40// 29// 36//
#define PIN_B             3//42//  27//38//
#define TORQSENSOR          0
#define MOTORCURR           14//grey
#define MOTORSPEED          15//pink; blue=ground

// Motor control pins
#define CW                 6//18//38//49// 47//42 //green
#define PWM                 4//19
#define ENABLE            5//20//36//40//51 brown

#define RESET               39

// Sensor gains
#define ANGVELGAIN          1//50000. * 360. / (60 * 4096.)
#define MCURRGAIN           2*maxCurrent/4096.0
#define MCURROFFSET         -maxCurrent

#define mechnicalConstant   0.231 //for 48v 0.231; // for 24V 0.077;
#define maxCurrent       5



// ofset angle
int encOffsetCount = 0;
int enPPRActuated = 6400;   //6400 for new motor 4096 for old motor
int enPPRnonActuated = 4096 ;

// Sensor data buffers
Buffer ang;
Buffer angvel;
Buffer mcurr;
Buffer torque;
float transformed_torque = 0;
float previous_torque = 0;
float offset_torque = 0;
Buffer control;

volatile byte inputButton = 1;
bool ledState = 1;


float desPos;
float maxTorq;

float prev_time = 0, prev_torque = 0;
float torq_df = 0;

// Active assist parameters
float arom[] = {999,999};
float prom[] = {999,999};
float asssitProfile[10] = {0};
float maxTorquePID = 3;

//Mehanism
byte currentMechanism = -99;

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
float pcKp = 0.0;
float desAng = 0.0;

// Torque Control
float tcKp = 0.0;
float tcKd = 0.2;
float desTorq = 0.0;

// resistance control

//float kp = 0.05;
//float kd = 0.05;
//float km = 0.001;
float tor;
float neutral_ang;




//float pwmval;

float offsetTorque;
float prevError;
unsigned long prevLoopTime;
float errorSum;
int count;
// Variables for calibration
byte maxCalibCount = 16;
byte calibCount = 0;
