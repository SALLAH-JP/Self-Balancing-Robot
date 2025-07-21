#include "Arduino.h"

// IMU funcitons declarations
/**
 * Function that checks if the IMU has received new data
 *  - returns 0 if not and 1 if yes
 */
int hasDataIMU();
/*
 * Function initialises the IMU 
 *  - I2C communication
 *  - DMP 
 *  - Configures offsets
 *  - IMU calibration
 *  
 *  returns 0 if error and 1 if well configured
 */
int initIMU();

/**
 * Function reading the pitch value from the IMU
 */
float getPitchIMU();

float getVelocityIMU(float dt);
