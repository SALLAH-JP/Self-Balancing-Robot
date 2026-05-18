#include "header.h"


void setup() {
    Serial.begin(115200);


    // IR Receiver Setup and interrupt
    IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

    initIMU();

    // PID Angle
    setpoint_angle = EQUILIBRE;
    pid_angle.SetMode(AUTOMATIC);
    pid_angle.SetSampleTime(10);
    pid_angle.SetOutputLimits(-200, 200);
    

    // Button Seput and interrupt with the module PinChangeInterrupt
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInt, 2);


    // Led RGB Setup
    pinMode(RED_PIN, OUTPUT);
    pinMode(GREEN_PIN, OUTPUT);
    //pinMode(BLUE_PIN, OUTPUT);
    changeMode();



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

    
    if ( !state ) {
        driveMotors(0, 0);
    } else {
        balancing();
    }

    changeMode(mode);
    remoteControl();
}


void balancing() {

    if (!dmpReady) return;

    if ( hasDataIMU() ) {

        pid_angle.Compute();

        steering = 0;
        setpoint_angle = EQUILIBRE;
                
        Serial.print(input_angle); Serial.print(" =>"); Serial.println(output_angle);
        if (abs(input_angle - EQUILIBRE) > 40) {
            driveMotors(0, 0);
            return;
        }

        if (mode == 1) directionRemoteControl();
        else if (mode == 2) lineTracking();

        driveMotors(output_angle, steering);

    } else {
        input_angle = getPitchIMU();
    }
}


void lineTracking() {


    leftValue = digitalRead(LEFT_SENSOR_PIN);
    rightValue = digitalRead(RIGHT_SENSOR_PIN);

    if ( leftValue == LOW && rightValue == LOW ) setpoint_angle = EQUILIBRE + 1;
    else {
        startBackward = millis();
        if ( leftValue == HIGH) steering = +75;
        else if (rightValue == HIGH ) steering = -75;
    }

    if (millis() - startBackward < 2000) setpoint_angle = EQUILIBRE;

}


void directionRemoteControl() {

    if (millis() - startForward < 100) setpoint_angle = EQUILIBRE + 5;
    else if (millis() - startBackward < 100) setpoint_angle = EQUILIBRE - 5;
    else if (millis() - startLeft < 100) steering = +75;
    else if (millis() - startRight < 100) steering = -75;

}


void remoteControl() {
    // **on décode réellement ici**  
    if (IrReceiver.decode()) {

        cmd = IrReceiver.decodedIRData.command;

        if ( mode == 1 ) {
            if ( cmd == 0x18 ) startForward = millis();
            if ( cmd == 0x08 ) startLeft = millis();
            if ( cmd == 0x5A ) startRight = millis();
            if ( cmd == 0x52 ) startBackward = millis();
        }

        if ( ( (IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) && (millis() - lastRemote > 200) ) || !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ) {

            if ( !(IrReceiver.decodedIRData.flags & IRDATA_FLAGS_IS_REPEAT) ) {
                if ( cmd == 0x45 ) buttonInt();
                else if ( cmd == 0x44 ) changeMode(1);
                else if ( cmd == 0x19 ) changeMode(2);
                else if ( cmd == 0x07 ) changeMode(0);
            }
            
            if ( cmd == 0x46 ) changeMode();

            lastRemote = millis();
        }

        IrReceiver.resume();
    }
}