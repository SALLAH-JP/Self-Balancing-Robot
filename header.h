#include <IRremote.h>
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"


// IMU instance
MPU6050 mpu;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

static float pitch;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector

float ypr[3];


/*********Tune these 4 values for your BOT*********/
#define EQUILIBRE 0.1

double Kp_angle = 15; // 13 => 19
double Ki_angle = 150; // 100 => 250
double Kd_angle = 0.8; //  0.4 => 0.9

// PID Angle (interne)
double input_angle, output_angle, setpoint_angle;
PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, REVERSE);
volatile bool mpuInterrupt = false;

//Motor Pin
const int RIGHT_MOTOR_PIN1 = 5;
const int RIGHT_MOTOR_PIN2 = 6;
const int LEFT_MOTOR_PIN1 = 9;
const int LEFT_MOTOR_PIN2 = 10;
float steering;

// Remote Control
const int IR_RECEIVE_PIN = 4;
unsigned long lastRemote = 0;
unsigned long startForward = 0;
unsigned long startBackward = 0;
unsigned long startLeft = 0;
unsigned long startRight = 0;
uint32_t cmd = 0;
int mode = 0;

// Mode Button
const int BUTTON_PIN = 2;
volatile bool buttonPressed = true;
bool state = false;


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
unsigned long lastStop = 0;


void dmpDataReady();
void buttonInt();
void changeMode( int newMode = -1 );
void driveMotors();
int hasDataIMU();
float getPitchIMU();
void initIMU();