#include "header.h"

void setup() {
    Serial.begin(115200);


    // IR Receiver Setup and interrupt
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-76);
    mpu.setYGyroOffset(-54);
    mpu.setZGyroOffset(2);
    mpu.setZAccelOffset(1384); 

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
        //setup PID
        pid.SetMode(AUTOMATIC);
        pid.SetSampleTime(10);
        pid.SetOutputLimits(-255, 255);
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }


    // Button Seput and interrupt with the module PinChangeInterrupt
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInt, 2);


    // Led RGB Setup
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    //pinMode(BLUE_PIN, OUTPUT);
    LedMode(); // POUR SYNCHRRONISER LE MODE ET LA LED


    // Tracking Sensor Setup
    pinMode(LEFT_SENSOR_PIN, INPUT);
    pinMode(RIGHT_SENSOR_PIN, INPUT);

    //Initialise the Motor output pins
    pinMode(LEFT_MOTOR_PIN1, OUTPUT);
    pinMode(LEFT_MOTOR_PIN2, OUTPUT);

    pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
    pinMode(RIGHT_MOTOR_PIN2, OUTPUT);

    //By default turn off both the motors
    analogWrite(LEFT_MOTOR_PIN1, LOW);
    analogWrite(LEFT_MOTOR_PIN2, LOW);

    analogWrite(RIGHT_MOTOR_PIN1, LOW);
    analogWrite(RIGHT_MOTOR_PIN2, LOW);
}

void loop() {

    
    if ( mode == 3 ) {
        Stop();
    } else {
        balancing();
    }

    remoteControl();
}

void remoteControl() {
    // **on décode réellement ici**  
    if (IrReceiver.decode()) {

        cmd = IrReceiver.decodedIRData.command;
        Serial.print(F("Nouvelle cmd = 0x")); Serial.println(cmd, HEX);

        if ( ( (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) && (millis() - lastRemote > 200) ) || !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ) {


            if ( mode == 1 || mode == 2) {
                if ( mode == 1) {
                    if ( cmd == 0x18 ) startForward = millis();
                    if ( cmd == 0x08 ) startLeft = millis();
                    if ( cmd == 0x5A ) startRight = millis();
                    if ( cmd == 0x52 ) startBackward = millis();
                }
            }

            if ( cmd == 0x45 ) mode = 3;
            if ( cmd == 0x44 ) mode = 1;
            if ( cmd == 0x19 ) mode = 2;
            if ( cmd == 0x07 ) mode = 0;
            if ( cmd == 0x46 ) buttonInt();

            if ( cmd == 0x0 )  Serial.println(F("Touche inconnue"));

            LedMode();
            lastRemote = millis();
        }

        IrReceiver.resume();
    }
}


void balancing() {

    if (!dmpReady) return;


    if (!mpuInterrupt && fifoCount < packetSize) {
    
        pid.Compute();
        setpoint = EQUILIBRE;
        delta = 0.0;
        
        Serial.print(input); Serial.print(" =>"); Serial.println(output);
        if (input > EQUILIBRE - 40 && input < EQUILIBRE + 40) {

            if ( mode == 1 ) {
                directionRemoteControl();
            } else if ( mode == 2 ) {
                lineTracking();
            }

            driveMotors();

        } else {
            Stop();
        }
    } else {

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
            input = degrees(ypr[1]) + 180;
        }
    }
}


void directionRemoteControl() {

    if (millis() - startForward < 100) setpoint = EQUILIBRE + 5;
    else if (millis() - startBackward < 100) setpoint = EQUILIBRE - 5;
    else setpoint = EQUILIBRE;

    if (millis() - startLeft  < 100) delta = -1.0;
    else if (millis() - startRight < 100) delta = +1.0;


}


void lineTracking() {

    leftValue = digitalRead(LEFT_SENSOR_PIN);
    rightValue = digitalRead(RIGHT_SENSOR_PIN);

    if ( leftValue == LOW && rightValue == LOW ) {
        setpoint = EQUILIBRE + 1.50;
    }
    

    if (millis() - lastStop > 1000) lastStop = millis();
    if (millis() - lastStop < 500) setpoint = EQUILIBRE - 1.50;

    if ( leftValue == HIGH) { 
        Serial.println("Left Line detected");
        delta = -1.0;
        setpoint = EQUILIBRE;
    } else if (rightValue == HIGH ) {
        Serial.println("Right Line detected");
        delta = +1.0;
        setpoint = EQUILIBRE;
    }

}
