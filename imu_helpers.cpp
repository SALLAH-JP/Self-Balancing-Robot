#include "imu_helpers.h"
#include <I2Cdev.h>
#include "MPU6050_6Axis_MotionApps20.h"

// IMU instance
MPU6050 mpu;
// MPU control/status vars
bool imuReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

static float pitch;
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaRaw, aaLin;
float ypr[3];


float vx = 0.0f;

// imu related finctions
// check if IMU has received data
int hasDataIMU() {
  return imuReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer);
}

// read the pitch value from the IMU
float getPitchIMU() {

  // read the package
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q); //get value for gravity
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
  pitch = ypr[1] * 180/M_PI;
  
  return pitch; 
}

float getVelocityIMU(float dt) {

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaLin, &aaRaw, &gravity);
    
  // 3) Conversion de l’axe X en m/s²
  // Remplacer par la valeur réelle pour votre configuration
  // ±2g = 16384 LSB/g, ±4g = 8192 LSB/g, etc.
  const float LSB_PER_G = mpu.getFullScaleAccelRange() == MPU6050_ACCEL_FS_2 ? 16384.0f :
                          mpu.getFullScaleAccelRange() == MPU6050_ACCEL_FS_4 ? 8192.0f :
                          mpu.getFullScaleAccelRange() == MPU6050_ACCEL_FS_8 ? 4096.0f : 2048.0f;
                          
  const float G_MS2     = 9.80665f;
  float ax = (float)aaLin.x / LSB_PER_G * G_MS2;

  // 4) Filtre passe‑bas simple sur l’accélération
  const float alpha = 0.85f;
  static float filtAx = 0.0f;
  filtAx = alpha * filtAx + (1 - alpha) * ax;

  // 5) Seuil anti‑drift
  if (fabs(filtAx) < 0.02f) filtAx = 0.0f;

  // 6) Intégration
  vx += filtAx * dt;

  return vx;
}

// initialise and configure the IMU with DMP
int initIMU() {

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();


  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  mpu.setXGyroOffset(-76);
  mpu.setYGyroOffset(-54);
  mpu.setZGyroOffset(2);
  mpu.setZAccelOffset(1384); 

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(6);
    Serial.println();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    mpuIntStatus = mpu.getIntStatus();
    imuReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
  return imuReady;
}
