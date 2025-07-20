#include <IRremote.h>
#include <PID_v1.h> //From https://github.com/br3ttb/Arduino-PID-Library/blob/master/PID_v1.h


/*********Tune these 4 values for your BOT*********/
#define EQUILIBRE  176.7

double Kp_angle = 13;
double Ki_angle = 180;
double Kd_angle = 0.8;

double Kp_vitesse = 0.05;
double Ki_vitesse = 0.001;
double Kd_vitesse = 0.0;

// PID Angle (interne)
double input_angle, output_angle, setpoint_angle;
PID pid_angle(&input_angle, &output_angle, &setpoint_angle, Kp_angle, Ki_angle, Kd_angle, DIRECT);

// PID Vitesse (externe)
double input_vitesse, output_vitesse, setpoint_vitesse;
PID pid_vitesse(&input_vitesse, &output_vitesse, &setpoint_vitesse, Kp_vitesse, Ki_vitesse, Kd_vitesse, DIRECT);


//Motor Pin
const int RIGHT_MOTOR_PIN1 = 5;
const int RIGHT_MOTOR_PIN2 = 6;
const int LEFT_MOTOR_PIN1 = 9;
const int LEFT_MOTOR_PIN2 = 10;
float steering;
float pwmL;
float pwmR;

// Remote Control
const int IR_RECEIVE_PIN = 4;
unsigned long lastRemote = 0;
uint32_t cmd = 0;

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


float last_angle = EQUILIBRE;
float estimated_velocity = 0;
unsigned long last_time = 0;
float dt = 0;