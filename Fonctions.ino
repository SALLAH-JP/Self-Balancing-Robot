void dmpDataReady() {
  mpuInterrupt = true;
}

void buttonInt() {
  state = !state;
}

void changeMode( int newMode = -1 ) {
  if (!state) {
    // ETEINT
    digitalWrite(RED_PIN, HIGH);
    digitalWrite(GREEN_PIN, HIGH);
    return;
  }

  if (newMode < 0) mode = (mode + 1) % 3;
  else mode = newMode % 3;

  switch (mode) {
    case 0:
      // Mode Balancing : JAUNE
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, LOW);
      //digitalWrite(BLUE_PIN, HIGH);
      break;

    case 1:
      // Mode RemoteControl : VERT
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(GREEN_PIN, LOW);
      //digitalWrite(BLUE_PIN, HIGH);
      break;

    case 2:
      // Mode LineTracking : ROUGE
      digitalWrite(RED_PIN, LOW);
      digitalWrite(GREEN_PIN, HIGH);
      //digitalWrite(BLUE_PIN, HIGH);
      break;
  }

}

void driveMotors(float torque, int turn) {

  int pwmL = torque - turn;
  int pwmR = torque + turn;

  pwmL = constrain(pwmL, -255, 255);
  pwmR = constrain(pwmR, -255, 255);

  if (abs(pwmL) < 60) pwmL = 0;
  if (abs(pwmR) < 60) pwmR = 0;

  // Gauche
  if (pwmL >= 0) {
    analogWrite(LEFT_MOTOR_PIN1,  0);
    analogWrite(LEFT_MOTOR_PIN2, (int)pwmL);
  } else {
    analogWrite(LEFT_MOTOR_PIN1, (int)(-pwmL));
    analogWrite(LEFT_MOTOR_PIN2, 0);
  }
  // Droite
  if (pwmR >= 0) {
    analogWrite(RIGHT_MOTOR_PIN1, 0);
    analogWrite(RIGHT_MOTOR_PIN2, (int)pwmR);
  } else {
    analogWrite(RIGHT_MOTOR_PIN1, (int)(-pwmR));
    analogWrite(RIGHT_MOTOR_PIN2, 0);
  }
}

int hasDataIMU() {
  return !mpuInterrupt && fifoCount < packetSize;
}

// read the pitch value from the IMU
float getPitchIMU() {
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
    mpu.dmpGetGravity(&gravity, &q); //get value for gravity
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
    pitch = ypr[1] * 180/M_PI;
  }
  return pitch; 
}

// initialise and configure the IMU with DMP
void initIMU() {

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
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(3), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
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

}