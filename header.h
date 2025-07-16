#include "I2Cdev.h"
#include <IRremote.h>
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include "MPU6050_6Axis_MotionApps20.h" //https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050

MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


/*********Tune these 4 values for your BOT*********/
double setpoint = 177; //176.5 set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kp = 15; //15 Set this first
double Kd = 0.8; //0.8 Set this secound
double Ki = 325; //325 Finally set this 
/******End of values setting*********/

//Motor Pin
const int RIGHT_MOTOR_PIN1 = 5;
const int RIGHT_MOTOR_PIN2 = 6;
const int LEFT_MOTOR_PIN1 = 9;
const int LEFT_MOTOR_PIN2 = 10;


// MPU Interrupt
double input, output;
PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high


// Remote Control
const int IR_RECEIVE_PIN = 4;
unsigned long lastRemote = 0;
unsigned long start = 0;
unsigned long startForward = 0;
unsigned long startBackward = 0;
unsigned long startLeft = 0;
unsigned long startRight = 0;
int controlVitesse = 100;
int vitesse = 100;


// Mode Button
const int BUTTON_PIN = 2;
volatile bool buttonPressed = true;
int mode = 3;


// Led RGB PIN
const int RED_PIN = 11;
const int GREEN_PIN = 12;
//const int BLUE_PIN = 13;

// Tracking Sensor
const int LEFT_SENSOR_PIN = 7;
const int RIGHT_SENSOR_PIN = 8;
//const int SENSOR3_PIN = 13;
int leftValue;
int rightValue;
unsigned long startTurn = 0;


uint32_t cmd = 0;
